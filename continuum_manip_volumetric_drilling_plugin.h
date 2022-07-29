// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include "collision_publisher.h"
#include "cable_pull_subscriber.h"

using namespace std;
using namespace ambf;

enum HapticStates
{
    HAPTIC_IDLE,
    HAPTIC_SELECTION
};

class afVolmetricDrillingPlugin: public afSimulatorPlugin{
    virtual int init(int argc, char** argv, const afWorldPtr a_afWorld) override;
    virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
    virtual void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes) override;
    virtual void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override {}
    virtual void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;

    DrillingPublisher* m_drillingPub;
    CablePullSubscriber* m_cablePullSub;
protected:
    // Initialize tool cursors
    void toolCursorInit(const afWorldPtr);

    void incrementDevicePos(cVector3d a_pos);

    void incrementDeviceRot(cVector3d a_rot);

    // update position of shaft tool cursors
    void toolCursorsPosUpdate(cTransform a_devicePose);

    // check for shaft collision
    void checkShaftCollision(void);

    // update position of drill mesh
    void drillPoseUpdateFromCursors(void);

    // toggles size of the drill burr
    void changeDrillSize(void);

    bool getOverrideDrillControl(){return m_overrideDrillControl;}

    void setOverrideDrillControl(bool val){m_overrideDrillControl = val;}

    void applyCablePull(double dt);

    void UpdateCablePullText();

    cTransform btTransformTocTransform(const btTransform& in);

private:
    cTransform T_contmanip_base; // Drills target pose
    cTransform T_carm; // Carm pose

    cTransform T_burr; // Burr pose

    cTransform T_i; // Input device transform
    cVector3d V_i; // Input device linear velocity

    double m_ambf_scale_to_mm;

    bool m_overrideDrillControl = false;

    cVoxelObject* m_voxelObj;

    cToolCursor* m_targetToolCursor;

    int m_renderingMode = 0;

    double m_opticalDensity;

    cMutex m_mutexVoxel;

    cCollisionAABBBox m_volumeUpdate;

    cColorb m_zeroColor;

    bool m_flagStart = true;
    
    bool m_volume_collisions_enabled = false;
    
    bool m_collect_tip_trace_enabled = false;
    bool m_show_tip_trace_enabled = false;

    int m_counter = 0;

    cGenericObject* m_selectedObject = NULL;

    cTransform m_tool_T_object;

    // a haptic device handler
    cHapticDeviceHandler* m_deviceHandler;
    cMultiSegment* m_traveled_points;
    // a pointer to the current haptic device
    cGenericHapticDevicePtr m_hapticDevice;

    bool m_flagMarkVolumeForUpdate = false;

    afRigidBodyPtr m_contManipBaseRigidBody;
    afRigidBodyPtr m_carmRigidBody;

    afRigidBodyPtr m_lastSegmentRigidBody;

    afVolumePtr m_volumeObject;

    cShapeSphere* m_burrMesh;

    // tool's rotation matrix
    cMatrix3d m_toolRotMat;

    // rate of drill movement
    double m_drillRate = 0.0020f;

    // Local offset between shaft tool cursors
    double m_dX = 0.03;

    // Chai 33 world pointer
    cWorld* m_chaiWorldPtr;

    // camera to render the world
    afCameraPtr m_mainCamera;

    bool m_showDrill = true;
    bool m_cableKeyboardControl = true;
    bool m_showGoalProxySpheres = true;
    bool m_burrOn = true;
    bool m_obstacle_estimate_enabled = false;
    int m_obstacle_estimate_idx = 0;

    // list of tool cursors
    vector<cToolCursor*> m_toolCursorList;

    vector<cToolCursor*> m_segmentToolCursorList;
    vector<cToolCursor*> m_shaftToolCursorList;
    vector<cToolCursor*> m_burrToolCursorList;
    vector<afRigidBodyPtr> m_segmentBodyList;
    vector<afJointPtr> m_segmentJointList;
    afRigidBodyPtr m_burrBody;

    // radius of tool cursors
    vector<double> m_toolCursorRadius{0.02, 0.013, 0.015, 0.017, 0.019, 0.021, 0.023, 0.025};

    // warning pop-up panel
    cPanel* m_warningPopup;
    cLabel* m_warningText;

    // panel to display current drill size
    cPanel* m_drillSizePanel;
    cLabel* m_drillSizeText;
    cLabel* m_cablePullMagText;

    cLabel* m_drillControlModeText;
    cLabel* m_cableControlModeText;

    // current and maximum distance between proxy and goal spheres
    double m_currError = 0;
    double m_maxError = 0;

    // for storing index of follow sphere
    int m_targetToolCursorIdx = 0;

    // toggles whether the drill mesh should move slowly towards the followSphere
    // or make a sudden jump
    bool m_suddenJump = true;

    // index of current drill size
    int m_drillSizeIdx = 0;

    // current drill size
    int m_currDrillSize = 2;

    // color property of bone
    cColorb m_boneColor;

    // get color of voxels at (x,y,z)
    cColorb m_storedColor;

    HapticStates m_controlMode = HAPTIC_IDLE;

    double m_cable_pull_mag_goal = 0.0;
    double m_cable_pull_mag = 0.0;
    double m_cable_pull_velocity = 0.0;

};



AF_REGISTER_SIMULATOR_PLUGIN(afVolmetricDrillingPlugin)
