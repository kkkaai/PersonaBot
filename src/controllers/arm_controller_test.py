import rclpy
from rclpy.node import Node
from .arm_controller import ArmController

def test_arm_controller():
    rclpy.init()
    arm_controller = ArmController('body_arm_controller')

    # Set joint positions and velocities for testing
    positions = [0.1] * 16
    velocities = [0.1] * 16

    arm_controller.set_joint_positions(positions, velocities)

    rclpy.spin_once(arm_controller, timeout_sec=10)

    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    test_arm_controller()
