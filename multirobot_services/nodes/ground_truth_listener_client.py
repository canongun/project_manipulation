#!/usr/bin/python3

import rospy

from multirobot_services.srv import GroundTruthListener

def ground_truth_listener_client():
    rospy.wait_for_service('/ground_truth_listener')
    try:
        ground_truth_listener = rospy.ServiceProxy('/ground_truth_listener', # service name
                                                GroundTruthListener   # service type
                                                )
        resp1 = ground_truth_listener(True)

        return resp1.link_info
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e) 
    

if __name__ == '__main__':
    print("Requesting ground truth listener service")
    print("Result=\n", ground_truth_listener_client())