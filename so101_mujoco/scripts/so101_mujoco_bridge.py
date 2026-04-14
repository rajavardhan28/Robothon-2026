#!/usr/bin/python3

import argparse
import os
import threading
import time

# EGL: headless GPU offscreen rendering.  Must use a single Renderer instance
# (= one EGL context) and a SingleThreadedExecutor so the context is always
# current on the same OS thread.
os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField


class So101MujocoBridge(Node):
    # Camera intrinsic constants (match scene.xml camera fovy=55, 640x480)
    _CAM_NAME = 'd435i'
    _CAM_W = 640
    _CAM_H = 480
    _CAM_FOVY_DEG = 55.0

    def __init__(self, model_path: str, publish_rate: float, startup_pose: str = 'home'):
        super().__init__('so101_mujoco_bridge')

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Joint set used by MoveIt / ros2_control configs.
        self.joint_names = [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex',
            'wrist_flex', 'wrist_roll', 'gripper',
        ]

        self._joint_qpos_addr = {}
        self._actuator_id = {}
        for name in self.joint_names:
            j_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j_id < 0:
                raise RuntimeError(f'Joint not found in MuJoCo model: {name}')
            self._joint_qpos_addr[name] = self.model.jnt_qposadr[j_id]
            a_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if a_id < 0:
                raise RuntimeError(f'Actuator not found in MuJoCo model: {name}')
            self._actuator_id[name] = a_id

        # Apply startup pose before first step.
        if startup_pose == 'upright':
            init = {
                'shoulder_pan': 0.0, 'shoulder_lift': -1.57,
                'elbow_flex': 0.0, 'wrist_flex': 0.0,
                'wrist_roll': 0.0, 'gripper': 0.0,
            }
        else:  # home – all zeros
            init = {name: 0.0 for name in self.joint_names}

        self._lock = threading.Lock()
        self._targets = init.copy()
        for name, val in init.items():
            self.data.qpos[self._joint_qpos_addr[name]] = val
            self.data.ctrl[self._actuator_id[name]] = val
        mujoco.mj_forward(self.model, self.data)

        # Single lazy renderer – one EGL context, owned exclusively by the
        # camera thread (never touched by the ROS executor threads).
        self._renderer: mujoco.Renderer | None = None
        self._cam_thread_stop = threading.Event()

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.rgb_pub        = self.create_publisher(Image,       '/d435i/image',        10)
        self.depth_pub      = self.create_publisher(Image,       '/d435i/depth_image',  10)
        self.cam_info_pub   = self.create_publisher(CameraInfo,  '/d435i/camera_info',  10)
        self.points_pub     = self.create_publisher(PointCloud2, '/d435i/points',       10)

        # Physics timer (100 Hz default).
        # Camera publishing is handled by a dedicated thread – NOT a ROS timer –
        # so EGL context stays on one OS thread and never blocks joint_states.
        period = 1.0 / max(publish_rate, 1.0)
        self.create_timer(period, self._step_and_publish)

        self._arm_server = ActionServer(
            self, FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_arm,
            goal_callback=self._goal_arm,
            cancel_callback=self._cancel_cb,
        )
        self._gripper_server = ActionServer(
            self, FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            execute_callback=self._execute_gripper,
            goal_callback=self._goal_gripper,
            cancel_callback=self._cancel_cb,
        )

        self.get_logger().info(
            f'Loaded MuJoCo model: {model_path}  startup_pose={startup_pose}')

    # ── action server helpers ────────────────────────────────────────────────

    def _goal_arm(self, goal_request):
        expected = {'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'}
        if not set(goal_request.trajectory.joint_names).issubset(expected):
            self.get_logger().warn('Rejecting arm trajectory with unexpected joint names')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _goal_gripper(self, goal_request):
        if not set(goal_request.trajectory.joint_names).issubset({'gripper'}):
            self.get_logger().warn('Rejecting gripper trajectory with unexpected joint names')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _duration_to_sec(self, dur: Duration) -> float:
        return float(dur.sec) + float(dur.nanosec) * 1e-9

    def _apply_point_targets(self, joint_names, positions):
        with self._lock:
            for name, pos in zip(joint_names, positions):
                self._targets[name] = float(pos)

    def _execute_common(self, goal_handle, allowed_joints):
        trajectory = goal_handle.request.trajectory
        points = trajectory.points
        joint_names = trajectory.joint_names

        if not points or not joint_names:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        start = time.time()
        prev_t = 0.0
        for point in points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            t = self._duration_to_sec(point.time_from_start)
            t = max(t, prev_t)
            prev_t = t

            if point.positions:
                names_to_set, pos_to_set = [], []
                for name, pos in zip(joint_names, point.positions):
                    if name in allowed_joints:
                        names_to_set.append(name)
                        pos_to_set.append(pos)
                if names_to_set:
                    self._apply_point_targets(names_to_set, pos_to_set)

            while time.time() - start < t:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()
                time.sleep(0.002)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def _execute_arm(self, goal_handle):
        return self._execute_common(
            goal_handle,
            {'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'},
        )

    def _execute_gripper(self, goal_handle):
        return self._execute_common(goal_handle, {'gripper'})

    # ── physics step ─────────────────────────────────────────────────────────

    def _step_and_publish(self):
        with self._lock:
            for name, target in self._targets.items():
                self.data.ctrl[self._actuator_id[name]] = target
            mujoco.mj_step(self.model, self.data)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.joint_names)
            msg.position = [
                float(self.data.qpos[self._joint_qpos_addr[n]]) for n in self.joint_names
            ]
            msg.velocity = [
                float(self.data.qvel[self._joint_qpos_addr[n]]) for n in self.joint_names
            ]
            msg.effort = []
        self.joint_state_pub.publish(msg)

    # ── camera ───────────────────────────────────────────────────────────────

    def _init_renderer(self) -> bool:
        try:
            self._renderer = mujoco.Renderer(
                self.model, height=self._CAM_H, width=self._CAM_W)
            return True
        except Exception as exc:
            self.get_logger().warn(
                f'Camera renderer init failed: {exc}', throttle_duration_sec=10.0)
            return False

    def _camera_thread_loop(self):
        """Runs in its own OS thread.  Owns the EGL context exclusively."""
        period = 1.0 / 30.0
        while not self._cam_thread_stop.is_set():
            t0 = time.monotonic()
            self._publish_camera()
            elapsed = time.monotonic() - t0
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _camera_info(self, stamp) -> CameraInfo:
        fovy_rad = np.deg2rad(self._CAM_FOVY_DEG)
        f  = (self._CAM_H / 2.0) / np.tan(fovy_rad / 2.0)
        cx = self._CAM_W / 2.0
        cy = self._CAM_H / 2.0
        info = CameraInfo()
        info.header.stamp      = stamp
        info.header.frame_id   = 'd435i_link'
        info.width             = self._CAM_W
        info.height            = self._CAM_H
        info.distortion_model  = 'plumb_bob'
        info.d                 = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k                 = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.r                 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p                 = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _depth_to_pointcloud(self, depth: np.ndarray, stamp) -> PointCloud2:
        fovy_rad = np.deg2rad(self._CAM_FOVY_DEG)
        f  = (self._CAM_H / 2.0) / np.tan(fovy_rad / 2.0)
        cx = self._CAM_W / 2.0
        cy = self._CAM_H / 2.0
        u, v = np.meshgrid(np.arange(self._CAM_W), np.arange(self._CAM_H))
        z    = depth.astype(np.float32)
        mask = (z > 0.05) & (z < 5.0)
        x    = ((u - cx) * z / f)[mask]
        y    = ((v - cy) * z / f)[mask]
        z    = z[mask]
        pts  = np.column_stack([x, y, z]).astype(np.float32)
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg = PointCloud2()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'd435i_link'
        msg.height          = 1
        msg.width           = len(pts)
        msg.fields          = fields
        msg.is_bigendian    = False
        msg.point_step      = 12
        msg.row_step        = 12 * len(pts)
        msg.data            = pts.tobytes()
        msg.is_dense        = True
        return msg

    def _publish_camera(self):
        if self._renderer is None and not self._init_renderer():
            return
        stamp = self.get_clock().now().to_msg()
        try:
            with self._lock:
                # RGB pass
                self._renderer.update_scene(self.data, camera=self._CAM_NAME)
                rgb = self._renderer.render().copy()
                # Depth pass (reuse same renderer/context)
                self._renderer.enable_depth_rendering()
                self._renderer.update_scene(self.data, camera=self._CAM_NAME)
                depth = self._renderer.render().copy()
                self._renderer.disable_depth_rendering()
        except Exception as exc:
            self.get_logger().warn(
                f'Camera render error: {exc}', throttle_duration_sec=5.0)
            return

        # RGB image
        img            = Image()
        img.header.stamp      = stamp
        img.header.frame_id   = 'd435i_link'
        img.height            = self._CAM_H
        img.width             = self._CAM_W
        img.encoding          = 'rgb8'
        img.is_bigendian      = False
        img.step              = self._CAM_W * 3
        img.data              = rgb.tobytes()
        self.rgb_pub.publish(img)

        # Depth image (32FC1, metres)
        d_img                 = Image()
        d_img.header.stamp    = stamp
        d_img.header.frame_id = 'd435i_link'
        d_img.height          = self._CAM_H
        d_img.width           = self._CAM_W
        d_img.encoding        = '32FC1'
        d_img.is_bigendian    = False
        d_img.step            = self._CAM_W * 4
        d_img.data            = depth.astype(np.float32).tobytes()
        self.depth_pub.publish(d_img)

        # Camera info + point cloud
        self.cam_info_pub.publish(self._camera_info(stamp))
        self.points_pub.publish(self._depth_to_pointcloud(depth, stamp))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True, help='Path to MuJoCo scene.xml')
    parser.add_argument('--publish-rate', type=float, default=100.0)
    parser.add_argument('--startup-pose', default='home', choices=['home', 'upright'],
                        help='Initial arm pose')
    args = parser.parse_args()

    rclpy.init()
    node = So101MujocoBridge(args.model, args.publish_rate, args.startup_pose)

    # SingleThreadedExecutor: ensures all callbacks (physics, camera) run on
    # the same OS thread, which is required for EGL context correctness.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # Start camera thread AFTER node is created (publishers exist) but before
    # spinning.  It owns the EGL context for its entire lifetime.
    cam_thread = threading.Thread(
        target=node._camera_thread_loop, name='mujoco_camera', daemon=True)
    cam_thread.start()

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node._cam_thread_stop.set()
        cam_thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
