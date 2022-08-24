#ifndef CABLE_PULL_SUBSCRIBER_H
#define CABLE_PULL_SUBSCRIBER_H

#include "ros/ros.h"
#include <string>
#include <afFramework.h>
#include <sensor_msgs/JointState.h>


enum cable_pull_command_type {POSITION, VELOCITY};

class CablePullSubscriber{
public:
    CablePullSubscriber(std::string a_namespace, std::string a_plugin);
    ~CablePullSubscriber();
    void init(std::string a_namespace, std::string a_plugin);
    ros::NodeHandle* m_rosNode;
    double cable_pull_position_target;
    double cable_pull_position_actual;
    double cable_pull_velocity_target;
    double cable_pull_velocity_actual;
    void publish_cablepull_measured_js(double meas_pos, double meas_vel);
    cable_pull_command_type command_type = cable_pull_command_type::POSITION;

private:
    void cablepull_move_jp_callback(sensor_msgs::JointState msg);
    void cablepull_servo_jv_callback(sensor_msgs::JointState msg);
   
    ros::Subscriber cablepull_move_jp_sub;
    ros::Subscriber cablepull_servo_jv_sub;
    ros::Publisher cablepull_measured_js_publisher;

};

#endif // CABLE_PULL_SUBSCRIBER_H
