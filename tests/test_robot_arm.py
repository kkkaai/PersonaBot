import rclpy
from rclpy.node import Node
from interface.msg import RobotCommd, Robotstate
from sensor_msgs.msg import JointState
import time
import threading
import pandas as pd
import os


class TestArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Publisher for sending commands to the robot arm
        self.cmdsub_publisher = self.create_publisher(RobotCommd, '/cmdsub', 10)

        # Subscriber for receiving the state of the robot arm
        #self.robot_state_subscription = self.create_subscription(Robotstate, '/robot_state', self.robot_state_callback, 10)
        self.robot_state_subscription = self.create_subscription(JointState, '/robot_state', self.robot_state_callback, 10)

        # Variable to store the latest robot state
        self.robot_state = None
        self.get_logger().info('ArmController node has been started.')

        # To store the initial joint positions
        self.initial_joint_positions = [0.0] * 16
        self.initial_joint_positions_set = False

        # Timer to periodically check the connection status
        self.check_topic_timer = self.create_timer(2.0, self.check_topic)

        self.wait_init_reading()


    def wait_init_reading(self, max_wait_time=5):
        # Wait until initial joint positions are set
        time.sleep(1)

        start_time = time.time()
        while not self.initial_joint_positions_set:
            if time.time() - start_time > max_wait_time:  # Timeout after max_wait_time seconds
                self.get_logger().warning('Initial joint positions not set, assuming all zeros.')
                self.initial_joint_positions = [0.0] * 16
                break
            self.get_logger().info('Waiting for initial joint positions...')
            time.sleep(1)

    def check_topic(self):
        # Check if the publisher is matched with any subscribers
        if not self.cmdsub_publisher.get_subscription_count():
            self.get_logger().warning('No subscribers found for /cmdsub topic.')
        else:
            self.get_logger().info('Subscribers found for /cmdsub topic.')
            self.check_topic_timer.cancel()

    def robot_state_callback(self, msg):
        # Store the received robot state
        self.robot_state = msg
        if not self.initial_joint_positions_set:
            self.initial_joint_positions = list(msg.position) if msg.position else [0.0] * 16
            self.initial_joint_positions_set = True
            self.get_logger().info(f'Initial joint positions set: {self.initial_joint_positions}')
        # Debug print to show the received robot state
        #self.get_logger().info(f'Received robot state: name={msg.name}, position={msg.position}, velocity={msg.velocity}, effort={msg.effort}')

    def send_command(self, enable, mode, joint_positions, joint_velocities):
        # Prepare the command message
        msg = RobotCommd()
        msg.enable = enable
        msg.mode = mode
        msg.joint = joint_positions
        msg.robotvel = joint_velocities
        msg.posel = []  # Assuming posel and poser are not used
        msg.poser = []
        
        # Debug print to show the command being sent
        self.get_logger().info(f'Sending command: enable={enable}, mode={mode}, joint_positions={joint_positions}, joint_velocities={joint_velocities}')
        self.cmdsub_publisher.publish(msg)
        # Additional debug print after publishing the message
        self.get_logger().info('Command published successfully.')

    def load_joint_data(self, file_path):
        # Read the CSV file containing joint angles
        df = pd.read_csv(file_path)
        return df
    
    def calculate_velocities(self, current_positions, next_positions, time_interval, max_velocity):
        # Calculate velocities and apply constraints
        velocities = [(abs(next_positions[i] - current_positions[i]) / time_interval) for i in range(len(current_positions))]
        velocities = [min(v, max_velocity) for v in velocities]  # Apply the max velocity threshold
        return velocities

    def perform_one(self):
        # Enable array for the joints (all enabled)
        enable = [1] * 16

        # Mode for joint position control
        mode = 1  # Joint position mode
        
        # Example joint positions
        joint_positions = [0.0] * 16
        #joint_positions = [0.0, 0.0, 148.7, 25.2, 0.0, -73.0, 13.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Example joint velocities (all set to 1.0)
        joint_velocities = [1.0] * 16

        # Send the command
        self.send_command(enable, mode, joint_positions, joint_velocities)

    def perform_sequence(self, velocity=3.0, sleep_time=3.0):
        # Define the sequence of joint positions
        positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 45.0, 0.0, 0.0, 9.0, 15.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 59.0, 0.163, 0.0, -21.2, 40.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 148.0, 25.2, 0.0, -73.0, 13.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 143.0, 14.3125, 0.0, -22.7, -1.66, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 148.7, 25.2, 0.0, -73.0, 13.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 142.0, 14.3, 0.0, -23.0, -1.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 148.6, 25.2, 0.0, -73.0, 17.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 142.0, 14.3, 0.0, -22.9, -1.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 59.0, 9.2, 0.0, -21.2, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 45.8, -1.7, 0.0, 9.0, 15.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]
        # Define the velocity profile
        enable = [1] * 16
        mode = 1  # Joint position mode
        joint_velocities = [1.0, 1.0, velocity, velocity, velocity, velocity, velocity, velocity, velocity, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

        for pos in positions:
            self.send_command(enable, mode, pos, joint_velocities)
            time.sleep(sleep_time)

    def calculate_velocities(self, current_positions, next_positions, time_interval, max_velocity, min_velocity):
        # Ensure both lists have the same length
        if len(current_positions) != len(next_positions):
            self.get_logger().error(f'Length mismatch: current_positions={len(current_positions)}, next_positions={len(next_positions)}')
            return [min_velocity] * len(current_positions)

        # Calculate velocities and apply constraints
        velocities = [(abs(next_positions[i] - current_positions[i]) / time_interval) for i in range(len(current_positions))]
        velocities = [max(min(v, max_velocity), min_velocity) for v in velocities]  # Apply the max and min velocity thresholds
        return velocities

    def perform_sequence_from_file(self, file_path, time_interval=5.0, max_velocity=10.0, min_velocity=1.0):
        mode = 1  # Joint position mode
        
        # Load predefined joint angle sequence file.
        df = self.load_joint_data(file_path)
        if len(df.columns) > 16:
            self.get_logger().error(f'Too many columns in CSV file: expected 16 or fewer, got {len(df.columns)}')
            return

        # Enable array for the joints (all enabled)
        enable = [1] * 16
        #enable = [1 if i < len(df.columns) else 0 for i in range(16)]

        # Check if initial joint positions are known.  if not, set to 0.
        if not self.initial_joint_positions_set:
            self.get_logger().warning('Initial joint positions not set, assuming all zeros.')
            self.initial_joint_positions = [0.0] * 16

        previous_positions = self.initial_joint_positions

        for index, row in df.iterrows():
            # expand position array to 16
            next_positions = row.values.tolist()
            next_positions.extend(self.initial_joint_positions[len(next_positions):])
            next_positions[0] = 0.0
            next_positions[1] = 0.0

            # Ensure the next_positions list has 16 elements
            if len(next_positions) != 16:
                self.get_logger().error(f'Row {index} length mismatch: expected 16, got {len(next_positions)}')
                continue

            joint_velocities = self.calculate_velocities(previous_positions, next_positions, time_interval, max_velocity, min_velocity)
            joint_velocities = [v if i < len(df.columns) else min_velocity for i, v in enumerate(joint_velocities)]
            
            self.send_command(enable, mode, next_positions, joint_velocities)

            previous_positions = next_positions
            time.sleep(time_interval)

        # Send the final command with last positions and zero velocity
        #time.sleep(time_interval)
        final_velocities = [1.0] * 16
        self.send_command(enable, mode, previous_positions, final_velocities)
        self.get_logger().info('Final command sent with zero velocities.')

        ## Send the Final Final command with last positions and velocity=1
        #time.sleep(time_interval)
        #final_velocities = [1.0] * 16
        #self.send_command(enable, mode, previous_positions, final_velocities)
        #self.get_logger().info('Final Final command sent with zero velocities.')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the ArmController node
    test_arm_controller = TestArmController()

    # Start the rclpy spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(test_arm_controller,))
    spin_thread.start()

    # Send a command to the robot arm
    try:
        # 1. Perform ONE action
        #test_arm_controller.perform_one()

        # 2. Perform a sequence of actions
        #test_arm_controller.perform_sequence(velocity=5.0, sleep_time=5.0)

        # 3. Perform a sequence of actions from CSV file
        relative_file_path = '../data/arm_motions/wave_hands_kp.csv'
        absolute_file_path = os.path.join(os.path.dirname(__file__), relative_file_path)
        test_arm_controller.perform_sequence_from_file(absolute_file_path, time_interval=7.0, max_velocity=15.0)

    except KeyboardInterrupt:
        # Graceful shutdown on keyboard interrupt
        pass

    finally:
        # Destroy the node explicitly
        test_arm_controller.destroy_node()
        # Shutdown the rclpy library
        rclpy.shutdown()
        # Ensure the spin thread finishes
        spin_thread.join()

if __name__ == '__main__':
    main()
