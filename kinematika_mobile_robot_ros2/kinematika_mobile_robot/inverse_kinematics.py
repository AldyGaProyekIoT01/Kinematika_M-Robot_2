#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # Parameter robot robin
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('wheel_separation', 0.45)

        self.wheel_radius = self.get_parameter(
            'wheel_radius'
        ).get_parameter_value().double_value

        self.wheel_separation = self.get_parameter(
            'wheel_separation'
        ).get_parameter_value().double_value

        # Input: kecepatan robot
        self.create_subscription(
            Twist,
            '/input_ik',
            self.velocity_callback,
            10
        )

        # Output: kecepatan angular roda
        self.left_publisher = self.create_publisher(
            Float64,
            '/model/robin/joint/base_left_wheel_joint/cmd_vel',
            10
        )

        self.right_publisher = self.create_publisher(
            Float64,
            '/model/robin/joint/base_right_wheel_joint/cmd_vel',
            10
        )

        self.get_logger().info(
            f'Inverse kinematics aktif | '
            f'r={self.wheel_radius:.3f} m, '
            f's={self.wheel_separation:.3f} m'
        )

    def velocity_callback(self, msg: Twist):

        v = msg.linear.x
        omega = msg.angular.z

        r = self.wheel_radius
        s = self.wheel_separation

        if r <= 0.0 or s <= 0.0:
            self.get_logger().error(
                'wheel_radius dan wheel_separation harus > 0.'
            )
            return

        # Inverse Kinematics differential drive
        #
        # omega_L = V/r - (s*omega)/(2r)
        # omega_R = V/r + (s*omega)/(2r)

        left_velocity = (
            v / r
            - (s * omega) / (2.0 * r)
        )

        right_velocity = (
            v / r
            + (s * omega) / (2.0 * r)
        )

        left_msg = Float64()
        right_msg = Float64()

        left_msg.data = left_velocity
        right_msg.data = right_velocity

        self.left_publisher.publish(left_msg)
        self.right_publisher.publish(right_msg)

        self.get_logger().info(
            f'V={v:.3f} m/s | '
            f'omega={omega:.3f} rad/s | '
            f'omega_L={left_velocity:.3f} rad/s | '
            f'omega_R={right_velocity:.3f} rad/s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = InverseKinematics()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
