import time
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray, Float64MultiArray, Int16


# 关节信息, 名称 : 0-1腰部 2-8右臂 9-15左臂#
# mode : -1 去使能; 3全使能; 1 关节位置模式 ;2 关节速度模式
# joint : 大小1×16 关节位置模式下的给定位置 0-1腰部 2-8右臂 9-15左臂
# robotvel : 大小1×16 在关节速度模式下 关节按照给定速度运行;  在关节位置模式下 关节按照给定速度运行至给定位置后停止

class ArmController(Node):

    def __init__(self, name):
        super().__init__(name)
        
        self.publisher_ = self.create_publisher(JointState, '/cmdsub', 10)
        self.subscription = self.create_subscription(JointState, '/robot_state', self.listener_callback, 10)
        self.joint_state = JointState()
        self.enable = Int16MultiArray()
        self.mode = Int16()
        self.joint_positions = Float64MultiArray()
        self.robot_velocities = Float64MultiArray()
        
        # Load the preset positions
        self.preset_paths = {
            "generated_action_1": "data/arm_motions/wave_hands.csv",
            "generated_action_2": "data/arm_motions/shake_hands.csv",
            "generated_action_3": "data/arm_motions/goodbye.csv",
            "generated_action_4": "data/arm_motions/standby.csv",
        }
        for key, path in self.preset_paths.items():
            with open(path, 'r') as f:
                self.preset_paths[key] = f.readlines()
            self.preset_paths[key] = [list(map(float, line.strip().split(','))) for line in self.preset_paths[key]]
            self.preset_paths[key] = np.array(self.preset_paths[key])
        

    def listener_callback(self, msg):
        self.get_logger().info(f'Received state data: {msg}')
        
    def set_joint_rotations(self, rotations, fps=20):
        """
        Set joint rotations
        :param rotations: np.ndarray, shape (T, 9)
        """
        # Rotation values are in radians with T rows and 9 columns
        # [q1, q2, q3, q4, q5, q6, q7, q8, q9]
        for i in range(rotations.shape[0]):
            # TODO: ROS2 code to set joint rotations
            raise NotImplementedError("This method should be implemented by subclasses")
            time.sleep(1/fps)
    
    def set_joint_positions(self, positions, velocities):
        self.mode.data = 1  # 关节位置模式
        self.joint_positions.data = positions
        self.robot_velocities.data = velocities
        self.publish_joint_command()

    def set_joint_velocities(self, velocities):
        self.mode.data = 2  # 关节速度模式
        self.robot_velocities.data = velocities
        self.publish_joint_command()

    def publish_joint_command(self):
        msg = JointState()
        msg.position = self.joint_positions.data
        msg.velocity = self.robot_velocities.data
        self.publisher_.publish(msg)

    def move_preset(self, preset_name):
        """
        Move to a preset position
        :param preset_name: str, name of the preset position
        """
        preset = self.preset_paths[preset_name]
        self.set_joint_rotations(preset)
    
