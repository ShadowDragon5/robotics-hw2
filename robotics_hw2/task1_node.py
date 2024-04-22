import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import sys


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.ticks = 0
        self.direction = "right"

    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1 / 60, self.update_callback)

    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def update_callback(self):
        ticks_duration = 960
        # Change direction after ticks_duration ticks
        if self.ticks >= ticks_duration:
            self.ticks = 0
            if self.direction == "right":
                self.direction = "left"
            else:
                self.direction = "right"

        cmd_vel = Twist()
        if self.direction == "right":
            cmd_vel.linear.x = 0.3  # [m/s]
            cmd_vel.angular.z = 1.0  # [rad/s]
        else:  # left
            cmd_vel.linear.x = 0.3  # [m/s]
            cmd_vel.angular.z = -1.0  # [rad/s]

        self.ticks += 1

        # Publish the command
        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = ControllerNode()
    node.start()

    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == "__main__":
    main()
