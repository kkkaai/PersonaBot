import rospy
from .hand_controller import HandController


def main():
    hand_controller = HandController()

    # Example: Set angles using publisher
    angles = [500, 500, 500, 500, 0, 0]
    hand_controller.set_angles(angles)
    rospy.sleep(2)

    # Example: Set angles using service
    response = hand_controller.set_angle_service_call(angles)
    print("Set angle service response:", response)

    # Example: Set forces using service
    forces = [100, 100, 100, 100, 50, 50]
    response = hand_controller.set_force_service_call(forces)
    print("Set force service response:", response)

    # Example: Get current angles
    current_angles = hand_controller.get_current_angles()
    print("Current angles:", current_angles)

    # Example: Get current forces
    current_forces = hand_controller.get_current_forces()
    print("Current forces:", current_forces)

    # Example: Get current status
    current_status = hand_controller.get_current_status()
    print("Current status:", current_status)

    rospy.spin()

if __name__ == '__main__':
    main()
