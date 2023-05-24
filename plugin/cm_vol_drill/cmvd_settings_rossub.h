#ifndef CMVD_SETTINGS_ROSSUB_H
#define CMVD_SETTINGS_ROSSUB_H

#include "ros/ros.h"
#include <string>
#include <std_msgs/Bool.h>
#include <afFramework.h>

class CMVDSettingsSub
{
public:
    CMVDSettingsSub(std::string a_namespace, std::string a_plugin);
    ~CMVDSettingsSub();
    ros::NodeHandle *m_rosNode;

    void callback_setShowToolCursors(std_msgs::Bool msg);
    void callback_setDrillControlMode(std_msgs::Bool msg);
    void callback_setVolumeCollisionsEnabled(std_msgs::Bool msg);
    void callback_setCableControlMode(std_msgs::Bool msg);
    void callback_setPhysicsPaused(std_msgs::Bool msg);
    void callback_initToolCursors(std_msgs::Bool msg);
    void callback_resetVoxels(std_msgs::Bool msg);
    void callback_setBurrOn(std_msgs::Bool msg);
    void publish_anatomy_pose(chai3d::cTransform transform, double m_to_ambf_unit=1.0);

    bool setShowToolCursors_changed = false;
    bool setDrillControlMode_changed = false;
    bool setVolumeCollisionsEnabled_changed = false;
    bool setCableControlMode_changed = false;
    bool setPhysicsPaused_changed = false;
    bool initToolCursors_changed = false;
    bool resetVoxels_changed = false;
    bool setBurrOn_changed = false;

    bool setShowToolCursors_last_val;
    bool setDrillControlMode_last_val;
    bool setVolumeCollisionsEnabled_last_val;
    bool setCableControlMode_last_val;
    bool setPhysicsPaused_last_val;
    bool setBurrOn_last_val;

private:
    ros::Subscriber sub_setShowToolCursors;
    ros::Subscriber sub_setDrillControlMode;
    ros::Subscriber sub_setVolumeCollisionsEnabled;
    ros::Subscriber sub_setCableControlMode;
    ros::Subscriber sub_setPhysicsPaused;
    ros::Subscriber sub_initToolCursors;
    ros::Subscriber sub_resetVoxels;
    ros::Subscriber sub_setBurrOn;
    ros::Publisher pub_anatomy_pose;

};

#endif // CMVD_SETTINGS_ROSSUB_H
