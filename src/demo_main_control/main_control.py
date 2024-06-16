import rclpy        # ROS 2
#import rospy        # ROS 1
import asyncio
import threading
from .state_control import StateControl                     # State Machine
from ..controllers.arm_controller import ArmController      # Robotic control
from ..controllers.navigation_controller import NavigationController
from ..controllers.hand_controller import HandController

class MainControl:
    def __init__(self):
        # State Machine
        self.state_control = StateControl()

        # Robotics ROS2
        rclpy.init()
        self.arm_controller = ArmController()

        # Create a separate thread to run ROS 2 spinning
        self.ros2_thread = threading.Thread(target=self.spin_ros2)
        self.ros2_thread.start()

        # Robotics ROS1
#        self.hand_controller = HandController('')
#        self.nav_controller = NavigationController('')

        # Create a separate thread to run ROS 1 spinning
#        self.ros1_thread = threading.Thread(target=self.spin_ros1)
#        self.ros1_thread.start()

    # Running the ArmController
    def spin_ros2(self):
        rclpy.spin(self.arm_controller)

#    def spin_ros1(self):
#        rospy.spin()

    # Running the state machine
    def run(self):
        asyncio.run(self.state_control.start())

    #async def run(self):
    #    await self.state_control.start()

    def shutdown(self):
        # Add shutdown procedures here
        rclpy.shutdown()
#        rospy.signal_shutdown("Main control shutdown")
        self.ros2_thread.join()
#        self.ros1_thread.join()
        pass
