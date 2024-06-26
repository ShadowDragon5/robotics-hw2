import sys
from enum import IntEnum

import rclpy
import tf_transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Range
import random


class Strategy(IntEnum):
    ALWAYS_RIGHT = 0
    RANDOM_DIRECTION = 1


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.done_future = None

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None

        self.prox_center = -1
        self.prox_center_left = -1
        self.prox_center_right = -1
        self.prox_left = -1
        self.prox_right = -1
        self.prox_rear_left = -1
        self.prox_rear_right = -1

        self.random_rotation = None
        self.strategy = Strategy.ALWAYS_RIGHT

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

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
        self.prox_rear_left_subscriber = self.create_subscription(
            Range, "proximity/rear_left", self.prox_rear_left_callback, 10
        )
        self.prox_rear_right_subscriber = self.create_subscription(
            Range, "proximity/rear_right", self.prox_rear_right_callback, 10
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

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist

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

    def prox_rear_left_callback(self, msg):
        self.prox_rear_left = msg.range

    def prox_rear_right_callback(self, msg):
        self.prox_rear_right = msg.range

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w,
        )

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw,  # theta orientation
        )

        return pose2

    def update_callback(self):
        # add the proximity ranges for each side
        prox_left_side = self.prox_center_left
        prox_right_side = self.prox_center_right

        # sum ranges from -4 to 0
        prox_sum = prox_left_side + prox_right_side

        cmd_vel = Twist()
        eps = 0.03

        speed = 0
        rotation = 0

        # randomly switch strategy
        if random.randint(0, 10) == 0:
            self.strategy = (self.strategy + 1) % (len(Strategy) + 1)

        # obstacle is in front
        if prox_sum + 2 > eps:
            speed = 0
            if self.strategy == Strategy.ALWAYS_RIGHT:
                rotation = 2
            elif self.random_rotation:
                rotation = self.random_rotation
            else:
                rotation = 2 * (-1 if random.randint(0, 50) == 0 else 1)
                self.random_rotation = rotation
        else:
            speed = 1
            self.random_rotation = None
            if self.strategy == Strategy.ALWAYS_RIGHT and random.randint(0, 50) == 0:
                rotation = -3

        cmd_vel.linear.x = float(speed)  # [m/s]
        cmd_vel.angular.z = float(rotation)  # [rad/s]

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
