import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Range


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.done_future = None

        self.prox_center = -1
        self.prox_center_left = -1
        self.prox_center_right = -1
        self.prox_left = -1
        self.prox_right = -1

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.prox_center_subscriber = self.create_subscription(
            Range, "proximity/center", self.prox_center_callback, 10
        )
        self.prox_center_left_subscriber = self.create_subscription(
            Range, "proximity/center_left", self.prox_center_left_callback, 10
        )
        self.prox_center_right_subscriber = self.create_subscription(
            Range, "proximity/center_right", self.prox_center_right_callback, 10
        )
        self.prox_left_subscriber = self.create_subscription(
            Range, "proximity/left", self.prox_left_callback, 10
        )
        self.prox_right_subscriber = self.create_subscription(
            Range, "proximity/right", self.prox_right_callback, 10
        )

    def start(self) -> Future:
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1 / 60, self.update_callback)

        self.done_future = Future()
        return self.done_future

    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def prox_center_callback(self, msg):
        self.prox_center = msg.range

    def prox_center_left_callback(self, msg):
        self.prox_center_left = msg.range

    def prox_center_right_callback(self, msg):
        self.prox_center_right = msg.range

    def prox_left_callback(self, msg):
        self.prox_left = msg.range

    def prox_right_callback(self, msg):
        self.prox_right = msg.range

    def update_callback(self):
        # add the proximity ranges for each side
        prox_left_side = self.prox_left + self.prox_center_left
        prox_right_side = self.prox_right + self.prox_center_right
        prox_sum = prox_left_side + prox_right_side
        prox_difference = prox_left_side - prox_right_side

        cmd_vel = Twist()

        speed = max(0, (-prox_sum / 8) - 0.4)
        cmd_vel.linear.x = float(speed)  # [m/s]
        cmd_vel.angular.z = float(prox_difference)  # 1.0  # [rad/s]

        if speed == 0 and abs(prox_difference) <= 0.03:  # reached final position
            self.get_logger().info("stopping...")
            self.stop()
            self.done_future.set_result(True)
        else:
            # Publish the command
            self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = ControllerNode()
    done = node.start()

    rclpy.spin_until_future_complete(node, done)

    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == "__main__":
    main()
