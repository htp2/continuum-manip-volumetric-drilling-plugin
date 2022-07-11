#include "obstacle_estimate_service.h"
#include <ambf_server/RosComBase.h>
#include <std_msgs/Float32.h>

using namespace std;

ObsacleEstimateServer::ObsacleEstimateServer(string a_namespace, string a_plugin){
    init(a_namespace, a_plugin);
}

ObsacleEstimateServer::~ObsacleEstimateServer(){
    cablePullSub.shutdown();
}

void ObsacleEstimateServer::init(string a_namespace, string a_plugin){
    m_rosNode = afROSNode::getNode();
    obstacle_estimate_service = nh.advertiseService("reset_volume_to_orig", &SnakeImagerWithROS::SetCTVolumeToOriginalServiceCallback, this); 
}

bool ObsacleEstimateServer::obstacleEstimateServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    return true;
}