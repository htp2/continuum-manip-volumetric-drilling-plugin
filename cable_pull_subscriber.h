#ifndef CABLE_PULL_SUBSCRIBER_H
#define CABLE_PULL_SUBSCRIBER_H

#include "ros/ros.h"
#include <string>
#include <std_msgs/Float32.h>
#include <afFramework.h>


class CablePullSubscriber{
public:
    CablePullSubscriber(std::string a_namespace, std::string a_plugin);
    ~CablePullSubscriber();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;
    double cable_pull_target;
    double cable_pull_actual;
    void publishCablePullMeasured(double measured);

private:
    void cablePullCallback(std_msgs::Float32 msg);
    ros::Subscriber cablePullSub;
    ros::Publisher cablePullPub;
};

#endif // CABLE_PULL_SUBSCRIBER_H
