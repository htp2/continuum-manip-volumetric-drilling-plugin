// To silence warnings on MacOS
#ifndef CONTINUUM_MANIP_VOLUMETRIC_DRILLING_PLUGIN_H
#define CONTINUUM_MANIP_VOLUMETRIC_DRILLING_PLUGIN_H

#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include "collision_publisher.h"
#include "cable_pull_subscriber.h"
#include "cmvd_settings_rossub.h"

using namespace std;
using namespace ambf;

class afVolmetricDrillingPlugin : public afSimulatorPlugin
{
    virtual int init(int argc, char **argv, const afWorldPtr a_afWorld) override;
    virtual void keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
    virtual void mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes) override;
    virtual void mousePosUpdate(GLFWwindow *a_window, double x_pos, double y_pos) override {}
    virtual void mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;

    DrillingPublisher *m_drillingPub;
    CablePullSubscriber *m_cablePullSub;
    CMVDSettingsSub *m_settingsPub;

protected:
    // Initialize tool cursors
    int toolCursorInit(const afWorldPtr);

    void incrementDevicePos(cVector3d a_pos);

    void incrementDeviceRot(cVector3d a_rot);

    // update position of shaft tool cursors
    void toolCursorsPosUpdate(cTransform a_devicePose);

    void applyCablePull(double dt);

    void UpdateCablePullText();

    bool cTransformAlmostEqual(const cTransform &a, const cTransform &b);

    void sliceVolume(int axisIdx, double delta);

    void setShowToolCursors(bool bool_show);

    void setDrillControlMode(bool bool_set);

    void setVolumeCollisionsEnabled(bool bool_set);

    void setCableControlMode(bool bool_set);

    void setPhysicsPaused(bool bool_set);

    int volumeInit(const afWorldPtr a_afWorld);

    btVector3 calculate_impulse_from_tool_cursor_collision(cToolCursor *tool_cursor, afRigidBodyPtr &body, double dt);

private:
    cTransform T_contmanip_base; // Drills target pose

    double m_to_ambf_unit;
    double mm_to_ambf_unit;

    bool m_CM_moved_by_other = true;

    cVoxelObject *m_voxelObj;

    int m_renderingMode = 0;

    double m_opticalDensity;

    cMutex m_mutexVoxel;

    cCollisionAABBBox m_volumeUpdate;

    cColorb m_zeroColor;

    bool m_volume_collisions_enabled = false;

    bool m_flagMarkVolumeForUpdate = false;

    afRigidBodyPtr m_contManipBaseRigidBody;

    afVolumePtr m_volumeObject;

    // rate of drill movement
    double m_drillRate = 0.0020f;

    // Chai 33 world pointer
    cWorld *m_chaiWorldPtr;

    // camera to render the world
    afCameraPtr m_mainCamera;

    bool m_showCM = true;
    bool m_cableKeyboardControl = true;
    bool m_showGoalProxySpheres = true;
    bool m_burrOn = true;
    bool m_obstacle_estimate_enabled = false;
    int m_obstacle_estimate_idx = 0;

    // list of tool cursors
    vector<cToolCursor *> m_segmentToolCursorList;
    vector<cToolCursor *> m_shaftToolCursorList;
    vector<cToolCursor *> m_burrToolCursorList;
    vector<afRigidBodyPtr> m_segmentBodyList;
    afRigidBodyPtr m_burrBody;
    afBaseObjectPtr m_predrill_reference_object;

    cLabel *m_cablePullMagText;
    cLabel *m_drillControlModeText;
    cLabel *m_cableControlModeText;

    // get color of voxels at (x,y,z)
    cColorb m_storedColor;

    double m_cable_pull_mag_goal = 0.0;
    double m_cable_pull_mag = 0.0;
    double m_cable_pull_velocity = 0.0;

    int m_removalCount = 150;

    cVector3d m_maxVolCorner, m_minVolCorner;
    cVector3d m_maxTexCoord, m_minTexCoord;
    cVector3d m_textureCoordScale; // Scale between volume corners extent and texture coordinates extent

    std::vector<std::vector<std::vector<double>>> m_voxel_hardnesses;
    bool m_debug_print = false;
    bool m_hardness_behavior = false;
    double m_hardness_removal_rate = 1.0/150.0;

    std::mt19937 rand_eng;
    std::uniform_real_distribution<> unif_dist;

    void checkForSettingsUpdate(void);

    int visualInit(const afWorldPtr a_afWorld);

    int hardnessBehaviorInit(const std::string &hardness_spec_file);

    int predrillTrajInit(const std::vector<std::string> &predrill_traj_files, const std::string& predrill_reference_object_name);

    void removeVoxel(cVector3d &pos);

};

int init(int argc, char **argv, const afWorldPtr a_afWorld);

AF_REGISTER_SIMULATOR_PLUGIN(afVolmetricDrillingPlugin)

#endif // CONTINUUM_MANIP_VOLUMETRIC_DRILLING_PLUGIN_H
