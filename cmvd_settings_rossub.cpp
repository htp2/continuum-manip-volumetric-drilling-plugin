#include "cmvd_settings_rossub.h"
#include <ambf_server/RosComBase.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Geometry>

using namespace std;

// class afVolmetricDrillingPlugin;

// You might be wondering why these aren't just in continuum_manip_volumetric_drilling_plugin. It seems a bit over-engineered
// The reason is that this allows for quickly changing to e.g. ROS2 without having to change the whole plugin!

CMVDSettingsSub::CMVDSettingsSub(string a_namespace, string a_plugin)
{
    m_rosNode = afROSNode::getNode();
    sub_setShowToolCursors = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/setShowToolCursors", 1, &CMVDSettingsSub::callback_setShowToolCursors, this);
    sub_setDrillControlMode = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/setDrillControlMode", 1, &CMVDSettingsSub::callback_setDrillControlMode, this);
    sub_setVolumeCollisionsEnabled = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/setVolumeCollisionsEnabled", 1, &CMVDSettingsSub::callback_setVolumeCollisionsEnabled, this);
    sub_setCableControlMode = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/setCableControlMode", 1, &CMVDSettingsSub::callback_setCableControlMode, this);
    sub_setPhysicsPaused = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/setPhysicsPaused", 1, &CMVDSettingsSub::callback_setPhysicsPaused, this);
    sub_initToolCursors = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/initToolCursors", 1, &CMVDSettingsSub::callback_initToolCursors, this);
    sub_resetVoxels = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/resetVoxels", 1, &CMVDSettingsSub::callback_resetVoxels, this);
    sub_setBurrOn = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/setBurrOn", 1, &CMVDSettingsSub::callback_setBurrOn, this);
    pub_anatomy_pose = m_rosNode->advertise<geometry_msgs::PoseStamped>(a_namespace + "/" + a_plugin + "/anatomy_pose", 1, true);
}

CMVDSettingsSub::~CMVDSettingsSub()
{
    sub_setShowToolCursors.shutdown();
    sub_setDrillControlMode.shutdown();
    sub_setVolumeCollisionsEnabled.shutdown();
    sub_setCableControlMode.shutdown();
    sub_setPhysicsPaused.shutdown();
    sub_initToolCursors.shutdown();
    sub_resetVoxels.shutdown();
    sub_setBurrOn.shutdown();
}

void CMVDSettingsSub::callback_setShowToolCursors(std_msgs::Bool msg)
{
    setShowToolCursors_changed = true;
    setShowToolCursors_last_val = msg.data;
}

void CMVDSettingsSub::callback_setDrillControlMode(std_msgs::Bool msg)
{
    setDrillControlMode_changed = true;
    setDrillControlMode_last_val = msg.data;
}

void CMVDSettingsSub::callback_setVolumeCollisionsEnabled(std_msgs::Bool msg)
{
    setVolumeCollisionsEnabled_changed = true;
    setVolumeCollisionsEnabled_last_val = msg.data;
}

void CMVDSettingsSub::callback_setCableControlMode(std_msgs::Bool msg)
{
    setCableControlMode_changed = true;
    setCableControlMode_last_val = msg.data;
}

void CMVDSettingsSub::callback_setPhysicsPaused(std_msgs::Bool msg)
{
    setPhysicsPaused_changed = true;
    setPhysicsPaused_last_val = msg.data;
}

void CMVDSettingsSub::callback_initToolCursors(std_msgs::Bool msg)
{
    initToolCursors_changed = true;
}

void CMVDSettingsSub::callback_resetVoxels(std_msgs::Bool msg)
{
    resetVoxels_changed = true;
}

void CMVDSettingsSub::callback_setBurrOn(std_msgs::Bool msg)
{
    resetVoxels_changed = true;
    setBurrOn_changed = msg.data;
}

void CMVDSettingsSub::publish_anatomy_pose(chai3d::cTransform transform, double m_to_ambf_unit)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = transform.getLocalPos().x() / m_to_ambf_unit;
    msg.pose.position.y = transform.getLocalPos().y() / m_to_ambf_unit;
    msg.pose.position.z = transform.getLocalPos().z() / m_to_ambf_unit;
    chai3d::cQuaternion q;
    q.fromRotMat(transform.getLocalRot());
    msg.pose.orientation.w = q.w;
    msg.pose.orientation.x = q.x;
    msg.pose.orientation.y = q.y;
    msg.pose.orientation.z = q.z;
    pub_anatomy_pose.publish(msg);
}