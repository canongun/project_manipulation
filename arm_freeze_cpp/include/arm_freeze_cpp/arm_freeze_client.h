#ifndef ARM_FREEZE_CLIENT_H
#define ARM_FREEZE_CLIENT_H

#include <ros/ros.h>
#include "actionlib/client/simple_action_client.h"
#include <arm_freeze_cpp/ArmFreezeAction.h>

class ArmFreezeClient
{
    public:
        ArmFreezeClient();
        void sendGoal(bool start);

    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionClient<arm_freeze_cpp::ArmFreezeAction> client_;
};

#endif
