import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

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

        # NOTE: we're using relative names to specify the topics (i.e., without a
        # leading /). ROS resolves relative names by concatenating them with the
        # namespace in which this node has been started, thus allowing us to
        # specify which Thymio should be controlled.

        self.ticks = 0
        self.direction = "right"

    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1 / 60, self.update_callback)

    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(
                *pose2d
            ),
            throttle_duration_sec=0.5,  # Throttle logging frequency to max 2Hz
        )

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

    def update_callback_figure_8(self):
        ticks_duration = 480
        print("update_callback")

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

        # print(f"cmd: {cmd_vel} {cmd_vel.linear.x}")
        self.ticks += 1

        # Publish the command
        self.vel_publisher.publish(cmd_vel)

    def update_callback(self):
        print("update_callback")

        # add the proximity ranges for each side
        prox_left_side = self.prox_left + self.prox_center_left
        prox_right_side = self.prox_right + self.prox_center_right
        prox_sum = prox_left_side + prox_right_side
        prox_difference = prox_left_side - prox_right_side

        cmd_vel = Twist()

        cmd_vel.linear.x = max(0, -prox_sum / 4)  # [m/s]
        cmd_vel.angular.z = float(prox_difference)  # 1.0  # [rad/s]

        print(f"prox sum: {-prox_sum/4}")
        print(f"prox difference: {prox_difference}")
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
