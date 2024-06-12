import rclpy
from geometry_msgs.msg import Pose
from src.controllers.arm_controller import ArmController

def main(args=None):
    print("rclpy.init")
    rclpy.init(args=args)

    print("Create ArmController")
    arm_controller = ArmController('left_arm')

    pose = Pose()
    pose.position.x = 0.1
    pose.position.y = 0.2
    pose.position.z = 0.3
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    print("ArmController set EE pose")
    arm_controller.move_end_effector(pose)

    # use a thread to maintain the node's running state.  later we will destroy the node.
    def spin_arm_controller():
        rclpy.spin(arm_controller)  # Keep the node running to handle callbacks and services

    # start spin_arm_controller, and stop after 5 seconds
    from threading import Thread
    spin_thread = Thread(target=spin_arm_controller)
    spin_thread.start()
    print("ArmController thread started.")

    try:
        import time
        time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        print("Destroy ArmController node.")
        arm_controller.destroy_node()
        rclpy.shutdown()
        spin_thread.join()
        print("rclpy shutdown.")
        

if __name__ == '__main__':
    main()
