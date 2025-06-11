#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 Integration Node: Subscribes to /voice_cmd_twist and /stop_cmd_twist,
and publishes the final Twist to /cmd_vel.  

Logic:
  - If /stop_cmd_twist publishes a Twist with linear.x == 0.0 (and angular.z == 0.0),
    this node continuously publishes a zero Twist to /cmd_vel (emergency stop).
  - Otherwise, it publishes the most recent Twist from /voice_cmd_twist to /cmd_vel.

Run:
  chmod +x integration_node.py
  ./integration_node.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class IntegrationNode(Node):
    def __init__(self):
        super().__init__('integration_node')

        # Store the most recent Twist from voice commands
        self.last_voice_twist = Twist()  # default initialized to zeros

        # Store the most recent Twist from stop detection
        self.last_stop_twist = Twist()   # default initialized to zeros

        # Publisher: final Twist to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: listen to /voice_cmd_twist for new velocity commands
        self.voice_sub = self.create_subscription(
            Twist,
            '/voice_cmd_twist',
            self.voice_callback,
            10
        )

        # Subscriber: listen to /stop_cmd_twist for stop/no-stop signal
        self.stop_sub = self.create_subscription(
            Twist,
            '/stop_cmd_twist',
            self.stop_callback,
            10
        )

        # Timer: publish at 10 Hz (every 0.1 seconds)
        timer_period = 0.1  # seconds => 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("IntegrationNode started: listening on /voice_cmd_twist and /stop_cmd_twist")

    def voice_callback(self, msg: Twist):
        """
        Callback for /voice_cmd_twist.
        Stores the latest voice-generated Twist.
        """
        self.last_voice_twist = msg
        # Optional debug log:
        # self.get_logger().info(f"[Integration] Voice Twist: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")

    def stop_callback(self, msg: Twist):
        """
        Callback for /stop_cmd_twist.
        Stores the latest stop-detection Twist.
        If both linear.x and angular.z are zero, that signals an emergency stop.
        """
        self.last_stop_twist = msg
        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.get_logger().warn("[Integration] StopSign detected → overriding voice commands!")

    def timer_callback(self):
        """
        Called at 10 Hz. Decides which Twist to publish to /cmd_vel:
          - If last_stop_twist indicates stop (linear.x == 0.0 & angular.z == 0.0),
            publish a zero Twist to /cmd_vel.
          - Otherwise, publish last_voice_twist to /cmd_vel.
        """
        out_twist = Twist()

        # Check for emergency stop condition
        if (self.last_stop_twist.linear.x == 0.0 and
            self.last_stop_twist.angular.z == 0.0):
            # Publish zero Twist (stop)
            out_twist.linear.x = 0.0
            out_twist.linear.y = 0.0
            out_twist.linear.z = 0.0
            out_twist.angular.x = 0.0
            out_twist.angular.y = 0.0
            out_twist.angular.z = 0.0
        else:
            # Otherwise, pass through the last voice command
            out_twist = self.last_voice_twist

        self.cmd_vel_pub.publish(out_twist)

    def destroy(self):
        """Cleanup on shutdown."""
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()


if __name__ == '__main__':
    main()
