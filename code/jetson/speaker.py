#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pyttsx3
import threading
import queue
import time

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        self.get_logger().info('Speaker node started. Listening to /cmd_vel')

        # Speech engine + queue setup
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        self.speech_queue = queue.Queue()
        self.speaker_thread = threading.Thread(target=self.speak_loop, daemon=True)
        self.speaker_thread.start()

        # Track last command to avoid repeating
        self.last_lin = None
        self.last_ang = None

    def cmd_callback(self, msg):
        lin = round(msg.linear.x, 2)
        ang = round(msg.angular.z, 2)

        if lin == self.last_lin and ang == self.last_ang:
            return

        self.last_lin = lin
        self.last_ang = ang

        # Determine speech based on priority
        if lin == 0.0 and ang == 0.0:
            sentence = "Stop"
        elif ang > 0.0:
            sentence = "Turning right"
        elif ang < 0.0:
            sentence = "Turning left"
        elif lin > 0.0:
            sentence = "Moving forward"
        elif lin < 0.0:
            sentence = "Reversing"
        else:
            sentence = "Unknown command"

        self.get_logger().info(f'Speaking: \"{sentence}\"')
        self.speech_queue.put(sentence)

    def speak_loop(self):
        while True:
            sentence = self.speech_queue.get()
            try:
                self.engine.say(sentence)
                self.engine.runAndWait()
            except Exception as e:
                print(f"Speech error: {e}")
            time.sleep(0.1)  # Optional debounce


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
