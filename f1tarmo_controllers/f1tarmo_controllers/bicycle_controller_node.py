# This node implements a kinematic bicycle model to convert Twist messages 
# (screw motion) to Ackermann steering commands (steering angle and rear wheel
# velocity).

import numpy as np
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


# Class to maintain the kinematic bicycle model following
# https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#car-like-bicycle-model
class KinematicBicycleModel:

    def __init__(self, wheelbase: float):
        self._wheelbase = wheelbase

    # Forward kinematics function.
    def forward_kinematics(self, steering_angle, rear_wheel_velocity):
        v_x = rear_wheel_velocity*np.cos(steering_angle)
        v_y = rear_wheel_velocity*np.sin(steering_angle)
        yaw_rate = (rear_wheel_velocity/self._wheelbase)*np.tan(steering_angle)
        return v_x, v_y, yaw_rate

    # Inverse kinematics function.
    def inverse_kinematics(self, cmd_v_x, cmd_v_yaw):
        # # If the commanded velocities are zero, return a positive zero.
        # if cmd_v_x == -0.0:
        #     cmd_v_x = 0.0
        # if cmd_v_yaw == -0.0:
        #     cmd_v_yaw = 0.0
        steering_angle = np.arctan2(cmd_v_yaw*self._wheelbase, np.abs(cmd_v_x))
        # Wrap the steering angle to the range [-pi/2, pi/2].
        # if steering_angle > np.pi/2:
        #     steering_angle -= np.pi
        # elif steering_angle < -np.pi/2:
        #     steering_angle += np.pi
        rear_wheel_velocity = cmd_v_x
        return steering_angle, rear_wheel_velocity
    
    # TODO: Figure out what is wrong with my angle wrapping function and why I
    # have to negate the steering angle!!!!

# TODO: I wonder if it would be practical / possible to use something like
# http://wiki.ros.org/kdl to produce the forward and inverse kinematics from an
# arbitrary URDF file. This would be a more general solution. I.e., maybe you
# tell it which joints you want to control and it gives you the forward and 
# inverse kinematics for those joints?


# Basic test node to translate Twist (screw motion) messages to steering and
# rear wheel velocity commands via bicycle model.
# This should be replaced with the inverse kinematics implemented as a part of
# ros2_control here:
# https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#car-like-bicycle-model
# The inverse kinematics for the more accurate Ackermann Steering model can be
# added later if desired.

# A downstream node will be listening downstream of this node to take those
# Ackermann commands (steering angle and rear wheel velocity) and send those off
# to the respective actuators (steering servo and rear wheel motor--controlled
# by the VESC).
class BicycleControllerNode(Node):

    def __init__(self):
        super().__init__('f1tarmo_ctrl_node')

        # Define any parameters for the kinematic bicycle model.
        self.declare_parameter("wheelbase", 0.33)
        self._wheelbase = self.get_parameter("wheelbase").value

        # Create a Kinematic Bicycle Model object.
        self._kinematic_bicycle_model = KinematicBicycleModel(self._wheelbase)

        # Create an Ackermann publisher.
        self.ackermann_cmd_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)

        # Create a subscriber to listen for Twist messages.
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)


    def listener_callback(self, twist_msg: Twist):

        # Print the Twist message for debugging.
        self.get_logger().debug(f"Received Twist message: {twist_msg}")

        # Use the bicycle model to compute the bicycle model inputs from the
        # commanded Twist message.
        steering_angle, rear_wheel_velocity = self._kinematic_bicycle_model.inverse_kinematics(twist_msg.linear.x, 
                                                                                               twist_msg.angular.z)

        # Print the computed steering angle and rear wheel velocity for
        # debugging.
        self.get_logger().debug(f"Computed steering angle: {steering_angle}, rear wheel velocity: {rear_wheel_velocity}")
        

        # Create an Ackermann message.
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = 'base_link'
        ackermann_msg.drive.steering_angle = steering_angle
        ackermann_msg.drive.speed = rear_wheel_velocity

        # Publish the Ackermann message.
        self.ackermann_cmd_pub.publish(ackermann_msg)


def main(args=None):
    rclpy.init(args=args)
    f1tarmo_ctrl_node = BicycleControllerNode()
    rclpy.spin(f1tarmo_ctrl_node)
    f1tarmo_ctrl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()