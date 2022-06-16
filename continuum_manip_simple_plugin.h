// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include "cable_pull_subscriber.h"

using namespace std;
using namespace ambf;

class ContinuumManip{
    public:
    
    ContinuumManip(const std::string& cont_manip_rigid_body_name, const afWorldPtr a_afWorld )
    : m_cont_manip_rigid_body_name(cont_manip_rigid_body_name){
        m_contManipBaseRigidBody = a_afWorld->getRigidBody(m_cont_manip_rigid_body_name);
        if (!m_contManipBaseRigidBody){
           std::cout << "ERROR! FAILED TO FIND RIGID BODY NAMED " << cont_manip_rigid_body_name << endl;
    }
        m_segmentBodyList.resize(m_num_segs);
        m_segmentJointList.resize(m_num_segs);
        for(auto& a : m_contManipBaseRigidBody->m_CJ_PairsAll){
                // For atoi, the input string has to start with a digit, so lets search for the first digit
                auto str = a.m_childBody->getName();
                size_t i = 0;
                for ( ; i < str.length(); i++ ){ if ( isdigit(str[i]) ) break; }

                // remove the first chars, which aren't digits
                str = str.substr(i, str.length() - i );

                // convert the remaining text to an integer
                int id = atoi(str.c_str());
                m_segmentBodyList[id-1] = a.m_childBody;
                m_segmentJointList[id-1] = a.m_childJoint;
            }


                // m_segmentBodyList.push_back(a_afWorld->getRigidBody("/ambf/env/BODY seg" + to_string(i)));
                // m_segmentJointList.push_back(a_afWorld->getJoint("/ambf/env/JOINT joint" + to_string(i)));
    
    }
    ~ContinuumManip(){};
    std::vector<afRigidBodyPtr> m_segmentBodyList;
    std::vector<afJointPtr> m_segmentJointList;
    afRigidBodyPtr m_contManipBaseRigidBody;

    private:
    std::string m_cont_manip_rigid_body_name;
    static const int m_num_segs = 27;

    

};

class afContinuumManipSimplePlugin: public afSimulatorPlugin{
    virtual int init(int argc, char** argv, const afWorldPtr a_afWorld) override;
    virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
    virtual void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes) override;
    virtual void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override {}
    virtual void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;

    CablePullSubscriber* m_cablePullSub;
    std::shared_ptr<ContinuumManip> manip;
    std::vector<std::shared_ptr<ContinuumManip>> manip_list;
    std::vector<afRigidBodyPtr> test_objects_list;

protected:
    void incrementDevicePos(cVector3d a_pos);
    void incrementDeviceRot(cVector3d a_rot);
    void applyCablePull(double dt);
    void UpdateCablePullText();

private:
    int m_num_segs = 27;
    cTransform T_contmanip_base; // Drills target pose
    double m_ambf_scale_to_mm;
    afRigidBodyPtr m_contManipBaseRigidBody;
    vector<afRigidBodyPtr> m_segmentBodyList;
    vector<afJointPtr> m_segmentJointList;

    // rate of drill movement
    double m_drillRate = 0.0020f;

    // camera to render the world
    afCameraPtr m_mainCamera;

    bool m_cableKeyboardControl = true;
    bool m_showGoalProxySpheres = true;
    bool m_obstacle_estimate_enabled = false;
    int m_obstacle_estimate_idx = 0;

    // list of tool cursors
    vector<cToolCursor*> m_toolCursorList;

    vector<double> m_old_jp = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    // text for GUI
    cLabel* m_cablePullMagText;
    cLabel* m_cableControlModeText;

    double m_cable_pull_mag_goal = 0.0;
    double m_cable_pull_mag = 0.0;

    void obstacleEstimate();

};

AF_REGISTER_SIMULATOR_PLUGIN(afContinuumManipSimplePlugin)
