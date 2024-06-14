import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        # Publisher for sending commands to the robot arm
        self.cmdsub_publisher = self.create_publisher(Int16MultiArray, '/cmdsub', 10)
        
        # Subscriber for receiving the state of the robot arm
        self.robot_state_subscription = self.create_subscription(JointState, '/robot_state', self.robot_state_callback, 10)
        
        # Variable to store the latest robot state
        self.robot_state = None
        self.get_logger().info('ArmController node has been started.')

        # Timer to periodically check the connection status
        self.timer = self.create_timer(2.0, self.check_topic)

    def check_topic(self):
        # Check if the publisher is matched with any subscribers
        if not self.cmdsub_publisher.get_subscription_count():
            self.get_logger().error('No subscribers found for /cmdsub topic.')
        else:
            self.get_logger().info('Subscribers found for /cmdsub topic.')

    def send_command(self, enable, mode, joint_positions, joint_velocities):
        # Prepare the command message
        msg = Int16MultiArray()
        msg.data = enable + [mode] + [int(pos * 1000) for pos in joint_positions] + [int(vel * 1000) for vel in joint_velocities]
        # Debug print to show the command being sent
        self.get_logger().info(f'Sending command: enable={enable}, mode={mode}, joint_positions={joint_positions}, joint_velocities={joint_velocities}')
        self.cmdsub_publisher.publish(msg)
        # Additional debug print after publishing the message
        self.get_logger().info('Command published successfully.')

    def robot_state_callback(self, msg):
        # Store the received robot state
        self.robot_state = msg
        # Debug print to show the received robot state
        self.get_logger().info(f'Received robot state: name={msg.name}, position={msg.position}, velocity={msg.velocity}, effort={msg.effort}')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create an instance of the ArmController node
    arm_controller = ArmController()

    # Example usage of sending a command to the robot arm
    try:
        # Enable array for the joints (all enabled)
        enable = [1] * 16
        # Mode for joint position control
        mode = 1  # Joint position mode
        # Example joint positions (all set to 0.5)
        joint_positions = [0.5] * 16
        # Example joint velocities (all set to 0.1)
        joint_velocities = [0.1] * 16

        # Send the command
        arm_controller.send_command(enable, mode, joint_positions, joint_velocities)

        # Spin the node to keep it running and processing callbacks
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        # Graceful shutdown on keyboard interrupt
        pass
    finally:
        # Destroy the node explicitly
        arm_controller.destroy_node()
        # Shutdown the rclpy library
        rclpy.shutdown()

if __name__ == '__main__':
    main()
