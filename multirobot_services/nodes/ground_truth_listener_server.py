#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from multirobot_services.srv import GroundTruthListener, GroundTruthListenerResponse

class GroundTruthListenerNode:
    """
    The Multirobot Topic Listener
    """

    def __init__(self):
        rospy.init_node('ground_truth_listener_server')

        self.pose = Odometry()

        self.message_received = False  # Flag to indicate whether a message has been received

        rospy.Service('ground_truth_listener',    # service name
                                GroundTruthListener,                # service type
                                self.ground_truth_listener           # function service provides
                                )

    def callback(self, msg):
        
        self.pose = msg.pose
        self.message_received = True  # Set the flag to True when a message is received

    def ground_truth_listener(self, req):
        
        if req.start:

            rospy.Subscriber('/ground_truth/mobile_tool0_state', Odometry, self.callback)

            # Wait until a message is received before printing
            while not self.message_received:
                rospy.sleep(0.1)

            link_info = []

            link_info.extend([self.pose.pose.position.x,
                            self.pose.pose.position.y,
                            self.pose.pose.position.z,
                            self.pose.pose.orientation.x,
                            self.pose.pose.orientation.y,
                            self.pose.pose.orientation.z,
                            self.pose.pose.orientation.w])
        
            return GroundTruthListenerResponse(link_info)
    
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            


def main():
    print("Listening to 'ground_truth/mobile_tool0_state' topic")
    node = GroundTruthListenerNode()
    node.run()
    
if __name__ == '__main__':
    main()
