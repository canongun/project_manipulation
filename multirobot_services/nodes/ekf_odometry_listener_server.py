#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from multirobot_services.srv import GroundTruthListener, GroundTruthListenerResponse

class EKFOdometryListener:
    """
    The Multirobot Topic Listener
    """

    def __init__(self):
        rospy.init_node('ekf_odometry_listener_server')

        self.pose = Odometry()

        self.message_received = False  # Flag to indicate whether a message has been received

        rospy.Service('ekf_odometry_listener',    # service name
                        GroundTruthListener,                # service type
                        self.ekf_odom_listener           # function service provides
                        )

    def callback(self, msg):
        
        self.pose = msg
        self.message_received = True  # Set the flag to True when a message is received

    def ekf_odom_listener(self, req):
        
        if req.start:

            rospy.Subscriber('/odometry/filtered', Odometry, self.callback)

            # Wait until a message is received before printing
            while not self.message_received:
                rospy.sleep(0.1)

            link_info = []

            link_info.extend([self.pose.pose.pose.position.x,
                            self.pose.pose.pose.position.y,
                            self.pose.pose.pose.position.z,
                            self.pose.pose.pose.orientation.x,
                            self.pose.pose.pose.orientation.y,
                            self.pose.pose.pose.orientation.z,
                            self.pose.pose.pose.orientation.w])
        
            return GroundTruthListenerResponse(link_info)
    
    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
            


def main():
    print("Listening to '/odometry/filtered' topic")
    node = EKFOdometryListener()
    node.run()
    
if __name__ == '__main__':
    main()