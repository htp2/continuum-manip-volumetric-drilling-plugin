#include "cable_pull_subscriber.h"
#include <ambf_server/RosComBase.h>
#include <std_msgs/Float32.h>

using namespace std;

CablePullSubscriber::CablePullSubscriber(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

CablePullSubscriber::~CablePullSubscriber(){
    cablePullSub.shutdown();
}

void CablePullSubscriber::init(string a_namespace, string a_plugin){
    m_rosNode = afROSNode::getNode();
    cablePullSub = m_rosNode->subscribe<std_msgs::Float32>(a_namespace + "/" + a_plugin + "/bend_motor/move_jp/",1, &CablePullSubscriber::cablePullCallback, this);
    cable_pull_target = 0.0;
    cablePullPub = m_rosNode->advertise<std_msgs::Float32>(a_namespace + "/" + a_plugin + "/bend_motor/measured_js/", 1);
}

void CablePullSubscriber::cablePullCallback(std_msgs::Float32 msg){
    cable_pull_target = msg.data;
}

void CablePullSubscriber::publishCablePullMeasured(double measured){
    std_msgs::Float32 msg;
    msg.data = measured;
    cablePullPub.publish(msg);
}