import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


class ArmController(Node):
    def __init__(self, namespace):
        super().__init__('arm_controller')
        self.namespace = namespace

        # joints control
        #self.joint_publishers = [
        #    self.create_publisher(Float64, f'{namespace}/joint_{i}/command', 10)
        #    for i in range(1, 8)
        #]

        # end-effector pose control
        #self.pose_publisher = self.create_publisher(Pose, f'{namespace}/end_effector/pose_command', 10)

        # joint read out
        #self.joint_state_subscriber = self.create_subscription(JointState, f'{namespace}/joint_states', self.joint_state_callback, 10)

        self.current_joint_positions = {}
    
    # joint read out
    def joint_state_callback(self, msg):
        self.current_joint_positions = dict(zip(msg.name, msg.position))

    def move_joint(self, joint_index, position):
        if 0 <= joint_index < len(self.joint_publishers):
            msg = Float64()
            msg.data = position
            self.joint_publishers[joint_index].publish(msg)

    def move_all_joints(self, positions):
        for i, position in enumerate(positions):
            if i < len(self.joint_publishers):
                self.move_joint(i, position)

    def move_end_effector(self, pose):
        if isinstance(pose, Pose):
            self.pose_publisher.publish(pose)
    
