#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Voice Command to Twist Node (Headless).  
Periodically (1 Hz) fetches the latest “throttle”/“angle” JSON from a remote HTTP endpoint 
and publishes it as geometry_msgs/Twist to /voice_cmd_twist.  

Expected environment variable:
  DONKEYCAR_ENDPOINT="http://<your_ip>:<your_port>"

Usage:
  chmod +x voice_cmd_to_vesc.py
  export DONKEYCAR_ENDPOINT="http://<your_ip>:<your_port>"
  ./voice_cmd_to_vesc.py
"""

import os
import requests

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VoiceCmdToVESC(Node):
    def __init__(self):
        super().__init__('voice_cmd_to_vesc')

        # Read HTTP endpoint from environment variable
        self.endpoint = os.getenv("DONKEYCAR_ENDPOINT")
        if not self.endpoint:
            self.get_logger().error("❌ DONKEYCAR_ENDPOINT not set! Exiting node.")
            rclpy.shutdown()
            return

        # This node will publish Commands to /voice_cmd_twist (instead of /cmd_vel)
        # so that an integration node can combine it with StopSign signals.
        self.publisher = self.create_publisher(Twist, '/voice_cmd_twist', 10)

        # Create a timer to fetch a new command every second (1 Hz)
        self.timer = self.create_timer(1.0, self.check_command)
        self.get_logger().info("🚀 VoiceCmdToVESC node started, publishing to /voice_cmd_twist")

    def check_command(self):
        """
        Called once per second. Performs an HTTP GET to {endpoint}/drive/last_cmd,
        expects a JSON payload like {"throttle": 0.5, "angle": 0.1}.
        Converts it to a Twist message and publishes to /voice_cmd_twist.
        """
        try:
            url = f"{self.endpoint}/drive/last_cmd"
            response = requests.get(url, timeout=2.0)
            if response.status_code == 200:
                cmd = response.json()
                self.handle_command(cmd)
            else:
                self.get_logger().warn(f"⚠️ HTTP returned status {response.status_code}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to fetch command from {self.endpoint}: {e}")

    def handle_command(self, cmd):
        """
        Parses JSON fields "throttle" and "angle" (fallback to 0.0 if missing or invalid),
        packs into a Twist message, and publishes to /voice_cmd_twist.
        """
        try:
            throttle = float(cmd.get("throttle", 0.0))
            angle = float(cmd.get("angle", 0.0))
        except Exception:
            throttle = 0.0
            angle = 0.0

        twist_msg = Twist()
        twist_msg.linear.x = throttle
        twist_msg.angular.z = 0.5*angle
        self.publisher.publish(twist_msg)

        self.get_logger().info(f"▶ Published /voice_cmd_twist: linear.x={throttle:.2f}, angular.z={angle:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdToVESC()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
