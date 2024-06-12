import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

class NavigationController:
    def __init__(self, namespace=''):
        # Initialize ROS node
        rospy.init_node('navigation_controller')

        # Publishers
        self.cmd_vel_pub = rospy.Publisher(f'{namespace}/cmd_vel', Twist, queue_size=10)
        self.move_base_pub = rospy.Publisher(f'{namespace}/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.move_base_cancel_pub = rospy.Publisher(f'{namespace}/move_base/cancel', GoalID, queue_size=10)

        self.predefined_goals = {
            'A': MoveBaseActionGoal(),
            'B': MoveBaseActionGoal()
        }
        # Define your goals here
        self.predefined_goals['A'].goal.target_pose.header.frame_id = 'map'
        self.predefined_goals['A'].goal.target_pose.pose.position.x = 1.0
        self.predefined_goals['A'].goal.target_pose.pose.position.y = 2.0
        self.predefined_goals['A'].goal.target_pose.pose.orientation.w = 1.0

        self.predefined_goals['B'].goal.target_pose.header.frame_id = 'map'
        self.predefined_goals['B'].goal.target_pose.pose.position.x = 3.0
        self.predefined_goals['B'].goal.target_pose.pose.position.y = 4.0
        self.predefined_goals['B'].goal.target_pose.pose.orientation.w = 1.0

    def move(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.cmd_vel_pub.publish(twist_msg)

    def stop(self):
        self.move(0.0, 0.0)

    def move_to_goal(self, goal_name):
        if goal_name in self.predefined_goals:
            goal = self.predefined_goals[goal_name]
            self.move_base_pub.publish(goal)
        else:
            rospy.logerr(f"Goal {goal_name} not found")

    def cancel_goal(self):
        cancel_msg = GoalID()
        self.move_base_cancel_pub.publish(cancel_msg)
