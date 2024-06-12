import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
#from move_base_msgs.msg import MoveBaseActionGoal
#from actionlib_msgs.msg import GoalID

class NavigationController:
    def __init__(self, namespace=''):
        # Initialize ROS node
        #rospy.init_node('navigation_controller')

        # Publishers
        # use rospy.Publisher to get publishers

        # Define your goal locations here
        self.predefined_goals = {}


    def move(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        # Call publisher to move with speed

    def stop(self):
        self.move(0.0, 0.0)

    def move_to_goal(self, goal_name):
        if goal_name in self.predefined_goals:
            goal = self.predefined_goals[goal_name]
            # Call publisher to move to a location
        else:
            rospy.logerr(f"Goal {goal_name} not found")

