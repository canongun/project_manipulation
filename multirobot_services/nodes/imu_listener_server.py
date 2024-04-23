#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Imu
from multirobot_services.srv import GroundTruthListener, GroundTruthListenerResponse

class ImuListenerNode:
    """
    The Multirobot Topic Listener
    """

    def __init__(self):
        rospy.init_node('imu_listener_server')

        self.pose = Imu()

        self.message_received = False  # Flag to indicate whether a message has been received

        rospy.Service('imu_listener',    # service name
                        GroundTruthListener,                # service type
                        self.imu_listener           # function service provides
                        )

    def callback(self, msg):
        
        self.pose = msg
        self.message_received = True  # Set the flag to True when a message is received

    def imu_listener(self, req):
        
        if req.start:

            rospy.Subscriber("/imu", Imu, self.callback)

            # Wait until a message is received before printing
            while not self.message_received:
                rospy.sleep(0.1)

            link_info = []

            link_info.extend([self.pose.orientation.x,
                            self.pose.orientation.y,
                            self.pose.orientation.z,
                            self.pose.orientation.w,
                            self.pose.angular_velocity.x,
                            self.pose.angular_velocity.y,
                            self.pose.angular_velocity.z,
                            self.pose.linear_acceleration.x,
                            self.pose.linear_acceleration.y,
                            self.pose.linear_acceleration.z])
        
            return GroundTruthListenerResponse(link_info)
    
    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()

def main():
    print("Listening to '/imu' topic")
    node = ImuListenerNode()
    node.run()
    
if __name__ == '__main__':
    main()
