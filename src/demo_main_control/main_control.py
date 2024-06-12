import rclpy        # ROS 2
import asyncio
import threading
from .state_control import StateControl                     # State Machine
from ..controllers.arm_controller import ArmController      # Robotic control

class MainControl:
    def __init__(self):
        # State Machine
        self.state_control = StateControl()

        # Robotics
        rclpy.init()
        self.arm_controller = ArmController('left_arm')

        # Create a separate thread to run ROS 2 spinning
        self.ros2_thread = threading.Thread(target=self.spin_ros2)
        self.ros2_thread.start()

    # Running the ArmController
    def spin_ros2(self):
        rclpy.spin(self.arm_controller)

    # Running the state machine
    def run(self):
        asyncio.run(self.state_control.start())

    #async def run(self):
    #    await self.state_control.start()

    def shutdown(self):
        # Add shutdown procedures here
        rclpy.shutdown()
        self.ros2_thread.join()
        pass