#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time


class PickPlace(Node):

    def __init__(self):
        super().__init__('pick_place')

        # Objects to process (you can add more)
        self.objects = ["eraser", "pen", "pencil", "sharpener"]

        self.get_logger().info("🚀 Pick & Place Pipeline Started")

        # wait for system to stabilize
        time.sleep(5)

        self.run_pipeline()

    def run_pipeline(self):

        for obj in self.objects:
            self.get_logger().info(f"\n===== Processing: {obj} =====")

            # STEP 1: PICK
            self.get_logger().info("Picking...")
            self.call_service('/pick_front')
            time.sleep(5)

            # STEP 2: ATTACH (logical attach)
            self.get_logger().info("Attaching (logical)...")
            self.attached = True
            time.sleep(2)

            # STEP 3: MOVE
            self.get_logger().info("Moving...")
            self.call_service('/move_to_joint_states')
            time.sleep(5)

            # STEP 4: PLACE
            self.get_logger().info("Placing...")
            self.call_service('/place_object')
            time.sleep(5)

            # STEP 5: DETACH
            self.get_logger().info("Detaching...")
            self.attached = False
            time.sleep(2)

        self.get_logger().info("✅ ALL OBJECTS COMPLETED")

    def call_service(self, service_name):

        self.get_logger().info(f"⏳ Waiting for {service_name}...")

        # wait until service is available
        while True:
            result = subprocess.run(
                "ros2 service list",
                shell=True,
                capture_output=True,
                text=True
            )

            if service_name in result.stdout:
                break

            time.sleep(1)

        self.get_logger().info(f"📡 Calling {service_name}")

        subprocess.run(
            f"ros2 service call {service_name} std_srvs/srv/Empty",
            shell=True
        )


def main():
    rclpy.init()
    node = PickPlace()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
