#include "obstacle_est_subpub.h"
#include <ambf_server/RosComBase.h>
#include <std_msgs/Float32MultiArray.h>
#include <ambf_msgs/RigidBodyState.h>

using namespace std;

ObstEstSubPub::ObstEstSubPub(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

ObstEstSubPub::~ObstEstSubPub(){
    goalSub.shutdown();
}

void ObstEstSubPub::init(string a_namespace, string a_plugin){
    int num_segs = 27;
    m_rosNode = afROSNode::getNode();
    js_goal.resize(num_segs);
    goalSub = m_rosNode->subscribe<ambf_msgs::RigidBodyState>("/ambf/env/snake_stick/State", 1, &ObstEstSubPub::goalSubCallback, this);
    costPub = m_rosNode->advertise<std_msgs::Float32MultiArray>(a_namespace + "/" + a_plugin + "/costs/", 1);
}

void ObstEstSubPub::goalSubCallback(ambf_msgs::RigidBodyState msg){
    for(size_t i = 0; i < js_goal.size(); i++){
        js_goal[i] = msg.joint_positions[i];
    }
}

void ObstEstSubPub::publish_costs(std::vector<float>& out){
    std_msgs::Float32MultiArray msg;
    msg.data = out;
    costPub.publish(msg);
}
