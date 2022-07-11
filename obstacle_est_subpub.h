#ifndef OBSTACLE_EST_SUBPUB_H
#define OBSTACLE_EST_SUBPUB_H

#include "ros/ros.h"
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <afFramework.h>
#include <ambf_msgs/RigidBodyState.h>


class ObstEstSubPub{
public:
    ObstEstSubPub(std::string a_namespace, std::string a_plugin);
    ~ObstEstSubPub();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;
    std::vector<double> js_goal;
    std::vector<double> cost;
    void publish_costs(std::vector<float>& costs);

private:
    void goalSubCallback(ambf_msgs::RigidBodyState msg);
    ros::Subscriber goalSub;
    ros::Publisher costPub;
};

#endif // OBSTACLE_EST_SUBPUB_H
