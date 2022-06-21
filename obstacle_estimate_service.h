#ifndef OBSTACLE_ESTIMATE_SERVICE_H
#define OBSTACLE_ESTIMATE_SERVICE_H

#include "ros/ros.h"
#include <string>
#include <std_msgs/Float32.h>
#include <afFramework.h>


class ObsacleEstimateServer{
public:
    ObsacleEstimateServer(std::string a_namespace, std::string a_plugin);
    ~ObsacleEstimateServer();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;
    std::vector<double> joint_angles;
    void publishCablePullMeasured(double measured);

private:
    bool obstacleEstimateServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    ros::ServiceServer obstacle_estimate_service;
};

#endif // OBSTACLE_ESTIMATE_SERVICE_H
