#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')

        # Parameter robot robin
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('wheel_separation', 0.45)

        self.wheel_radius = self.get_parameter(
            'wheel_radius'
        ).get_parameter_value().double_value

        self.wheel_separation = self.get_parameter(
            'wheel_separation'
        ).get_parameter_value().double_value

        # Kecepatan angular roda
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Input dari Gazebo
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Output hasil Forward Kinematics
        self.publisher = self.create_publisher(
            Twist,
            '/output_fk',
            10
        )

        self.get_logger().info(
            f'Forward kinematics aktif | '
            f'r={self.wheel_radius:.3f} m, '
            f's={self.wheel_separation:.3f} m'
        )

    def joint_state_callback(self, msg: JointState):

        # Cari index joint roda kiri
        if 'base_left_wheel_joint' in msg.name:
            left_index = msg.name.index('base_left_wheel_joint')

            if len(msg.velocity) > left_index:
                self.left_wheel_velocity = msg.velocity[left_index]

        # Cari index joint roda kanan
        if 'base_right_wheel_joint' in msg.name:
            right_index = msg.name.index('base_right_wheel_joint')

            if len(msg.velocity) > right_index:
                self.right_wheel_velocity = msg.velocity[right_index]

        self.calculate_forward_kinematics()

    def calculate_forward_kinematics(self):

        r = self.wheel_radius
        s = self.wheel_separation

        if r <= 0.0 or s <= 0.0:
            self.get_logger().error(
                'wheel_radius dan wheel_separation harus > 0.'
            )
            return

        wl = self.left_wheel_velocity
        wr = self.right_wheel_velocity

        # Forward Kinematics differential drive
        #
        # V = r/2 * (wl + wr)
        # omega = r/s * (wr - wl)

        linear_velocity = (r / 2.0) * (wl + wr)
        angular_velocity = (r / s) * (wr - wl)

        msg = Twist()

        msg.linear.x = linear_velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_velocity

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ForwardKinematics()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
