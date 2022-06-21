// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include "cable_pull_subscriber.h"
#include <Eigen/Geometry>

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

    typedef long int Index;
    template<typename Scalar>
    class Callback
    {
    public:
        afContinuumManipSimplePlugin *self;
        Callback(afContinuumManipSimplePlugin* self): self(self){};
        // Callback(){};
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

        bool operator()(const Index i, const Matrix& particles, const Vector &, const Index) const{
            if (i == -1){
                for(Index i = 0; i < particles.cols(); i++){
                    double r = particles(0,i);
                    double th = particles(1,i);
                    auto& man = self->manip_list[i];
                    auto& test_object = self->test_objects_list[i];
                    auto T_contmanipbase = man->m_contManipBaseRigidBody->getLocalTransform();
                    auto T_testobj = test_object->getLocalTransform();
                    auto first_seg_off = 0.3602972;
                    T_testobj.setLocalPos(T_contmanipbase.getLocalPos()+cVector3d(r*std::cos(th),first_seg_off + r*std::sin(th),0.0));
                    test_object->setLocalTransform(T_testobj);
                } 
                for(size_t i=0; i<100; i++){
                    // self->physicsUpdate(0.001);
                    self->m_worldPtr->pluginsPhysicsUpdate(0.01);
                }

            }
        
            // self->physicsUpdate(0.001);
            // self->graphicsUpdate();
            self->m_worldPtr->pluginsPhysicsUpdate(0.001);
            // self->m_worldPtr->pluginsGraphicsUpdate();
            // self->m_mainCamera->update(0.001);
            // std::cout << "CALLBACK" << std::endl;
            return true;
        }
    };
    
    class Fitness
    {
    public:
        afContinuumManipSimplePlugin *self;
        Fitness(afContinuumManipSimplePlugin* self): self(self){};
        
        template<typename Derived>
        double operator()(const Eigen::MatrixBase<Derived> &xval, int i) const 
        {
            std::vector<double> goal_jp = {0.04719538614153862, 0.07040780782699585, 0.0539616122841835, 0.04232475906610489, 0.01811256818473339, 0.010549008846282959, -0.00471153948456049, -0.0024778724182397127, -0.021339787170290947, -0.02025371417403221, -0.02914055995643139, -0.022847743704915047, -0.0291144922375679, -0.023115238174796104, -0.02983745187520981, -0.0232921801507473, -0.02943757176399231, -0.023809820413589478, -0.02989761345088482, -0.02374465949833393, -0.0301345381885767, -0.024002499878406525, -0.030253717675805092, -0.023892860859632492, -0.030802471563220024, -0.02734425663948059, -0.049160074442625046};

            // assert(xval.size() == 2);
            // double r = xval(0);
            // double th = xval(1);

            auto& man = self->manip_list[i];
            std::vector<double> jp;
            for(auto& joint: man->m_segmentJointList){
                jp.push_back(joint->getPosition());
            }
            auto total_err = 0.0;
            auto real_err = 0.0;
            for (size_t i=0; i<self->m_num_segs; i++){
                total_err += jp[i]-goal_jp[i];
                real_err += abs(jp[i]-goal_jp[i]);
                std::cout << jp[i]-goal_jp[i] << ", ";

            }
            std::cout << std::endl;
            return real_err;
            // auto& test_object = test_objects_list[i];
            // auto T_contmanipbase = man->m_contManipBaseRigidBody->getLocalTransform();
            // auto T_testobj = test_object->getLocalTransform();
            // auto first_seg_off = 0.3602972;
            // T_testobj.setLocalPos(T_contmanipbase.getLocalPos()+cVector3d(r*std::cos(th),first_seg_off + r*std::sin(th),0.0));
            // test_object->setLocalTransform(T_testobj);

        };
    };

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

    double GetRandFromUniform();
    // typedef long int Index;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
    
    // template<typename Scalar>
    // bool optCallback(const Index, const Matrix&, const Vector &, const Index);

};

AF_REGISTER_SIMULATOR_PLUGIN(afContinuumManipSimplePlugin)
