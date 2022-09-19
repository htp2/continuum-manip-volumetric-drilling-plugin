#include "cmvd_settings_rossub.h"
#include <ambf_server/RosComBase.h>
#include <std_msgs/Bool.h>

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