#import rospy
#from inspire_hand.msg import angle_set, status
#from inspire_hand.srv import set_angle, get_angle_act, get_status, set_force, get_force_act

class HandController:
    def __init__(self, namespace=''):
        # Initialize ROS node
        #rospy.init_node('hand_controller')

        # Publisher to set angles
        #self.angle_pub = rospy.Publisher(f'{namespace}/setangle_topic', angle_set, queue_size=10)

        # Subscriber to get status
        #self.status_sub = rospy.Subscriber(f'{namespace}/status_topic', status, self.status_callback)

        # Service clients
        #rospy.wait_for_service(f'{namespace}/set_angle')
        #rospy.wait_for_service(f'{namespace}/get_angle_act')
        #rospy.wait_for_service(f'{namespace}/get_status')
        #rospy.wait_for_service(f'{namespace}/set_force')
        #rospy.wait_for_service(f'{namespace}/get_force_act')

        #self.set_angle_service = rospy.ServiceProxy(f'{namespace}/set_angle', set_angle)
        #self.get_angle_act_service = rospy.ServiceProxy(f'{namespace}/get_angle_act', get_angle_act)
        #self.get_status_service = rospy.ServiceProxy(f'{namespace}/get_status', get_status)
        #self.set_force_service = rospy.ServiceProxy(f'{namespace}/set_force', set_force)
        #self.get_force_act_service = rospy.ServiceProxy(f'{namespace}/get_force_act', get_force_act)

        self.current_status = None

    def status_callback(self, msg):
        self.current_status = msg

    def set_angles(self, angles):
        #angles_msg = angle_set(
        #    angle1_set=angles[0],
        #    angle2_set=angles[1],
        #    angle3_set=angles[2],
        #    angle4_set=angles[3],
        #    angle5_set=angles[4],
        #    angle6_set=angles[5],
        #)
        #self.angle_pub.publish(angles_msg)
        return

    def set_angle_service_call(self, angles):
        #try:
            #response = self.set_angle_service(
            #    angle1=angles[0],
            #    angle2=angles[1],
            #    angle3=angles[2],
            #    angle4=angles[3],
            #    angle5=angles[4],
            #    angle6=angles[5],
            #)
            #return response
        #except rospy.ServiceException as e:
        #    rospy.logerr("Service call failed: %s" % e)
        return

    def set_force_service_call(self, forces):
        return

    def get_current_angles(self):
        return
    
    def get_current_forces(self):
        return
    
    def get_current_status(self):
        return
        