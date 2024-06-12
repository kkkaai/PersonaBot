import rospy
from .navigation_controller import NavigationController

def main():
    navigation_controller = NavigationController()

    # Example usage
    navigation_controller.move(0.5, 0.0)  # Move forward with 0.5 m/s
    rospy.sleep(1)
    navigation_controller.stop()  # Stop
    rospy.sleep(1)
    navigation_controller.move_to_goal('A')  # Move to goal A

    rospy.spin()

if __name__ == '__main__':
    main()
