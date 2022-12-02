#include "cable_pull_subscriber.h"
#include <ambf_server/RosComBase.h>
#include <sensor_msgs/JointState.h>

using namespace std;

CablePullSubscriber::CablePullSubscriber(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

CablePullSubscriber::~CablePullSubscriber(){
    cablepull_move_jp_sub.shutdown();
    cablepull_servo_jv_sub.shutdown();
    cablepull_measured_js_publisher.shutdown();
}

void CablePullSubscriber::init(string a_namespace, string a_plugin){
    m_rosNode = afROSNode::getNode();
    cablepull_move_jp_sub = m_rosNode->subscribe<sensor_msgs::JointState>(a_namespace + "/" + a_plugin + "/bend_motor/move_jp/",1, &CablePullSubscriber::cablepull_move_jp_callback, this);
    cablepull_servo_jv_sub = m_rosNode->subscribe<sensor_msgs::JointState>(a_namespace + "/" + a_plugin + "/bend_motor/servo_jv/",1, &CablePullSubscriber::cablepull_servo_jv_callback, this);
    
    cable_pull_position_target = 0.0;
    cable_pull_velocity_target = 0.0;
    cablepull_measured_js_publisher = m_rosNode->advertise<sensor_msgs::JointState>(a_namespace + "/" + a_plugin + "/bend_motor/measured_js/", 1);
}

void CablePullSubscriber::cablepull_move_jp_callback(sensor_msgs::JointState msg){
    cable_pull_position_target = msg.position[0];
    command_type = cable_pull_command_type::POSITION;
}

void CablePullSubscriber::cablepull_servo_jv_callback(sensor_msgs::JointState msg){
    cable_pull_velocity_target = msg.velocity[0];
    command_type = cable_pull_command_type::VELOCITY;
}

void CablePullSubscriber::publish_cablepull_measured_js(double meas_pos, double meas_vel){
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = 'cm_base'; // TODO?
    msg.name = {"bend"};
    msg.position = {meas_pos};
    msg.velocity = {meas_vel};
    cablepull_measured_js_publisher.publish(msg);
}