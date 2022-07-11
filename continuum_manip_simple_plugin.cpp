//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar

    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam

    \author    <henry.phalen@jhu.edu>
    \author    Henry Phalen
*/
//==============================================================================

#include "continuum_manip_simple_plugin.h"
#include <boost/program_options.hpp>

using namespace std;
#include <random>
#include "psocpp.h"

// Implement an objective functor.
struct Ackley
{
    static double pi()
    { return 3.141592653589; }

    Ackley()
    { }

    template<typename Derived>
    double operator()(const Eigen::MatrixBase<Derived> &xval, int i) const
    {
        // std::cout << i << std::endl;
        assert(xval.size() == 2);
        double x = xval(0);
        double y = xval(1);
        return -20.0 * std::exp(-0.2 * std::sqrt(0.5 * (x * x + y * y))) -
            std::exp(0.5 * (std::cos(2 * pi() * x) + std::cos(2 * pi() * y))) +
            std::exp(1) + 20.0;
    }
};
  /** Integer type for indexing arrays, vectors and matrices. */
    // typedef long int Index;

    /** @brief Dummy callback functor, which always and only returns true. */
    // template<typename Scalar>
    // class Callback
    // {
    // public:
    //     typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    //     typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;

    //     bool operator()(const Index, const Matrix&, const Vector &, const Index) const
    //     {
    //         std::cout <<"CALLBACK" << std::endl;
    //         return true;
    //     }
    // };

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

int afContinuumManipSimplePlugin::init(int argc, char **argv, const afWorldPtr a_afWorld){
    // Bring in ambf world, make adjustments as needed to improve simulation accuracy
    // auto callback  = afContinuumManipSimplePlugin::Callback<double>(this);
    m_worldPtr = a_afWorld;
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp=1.0; // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2=1.0; // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->setGravity(btVector3(0.0,0.0,0.0));
    // Get first camera
    m_mainCamera = m_worldPtr->getCameras()[0];

    // importing continuum manipulator model
    std::string cont_manip_rigid_body_name = "snake_stick";
    // m_contManipBaseRigidBody = m_worldPtr->getRigidBody(cont_manip_rigid_body_name);
    // T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform();

    m_num_manip = 20;
    std::string ros_prefix = "test";
    std::string base_name = ros_prefix+"/ambf/env/BODY snake_stick";
    std::string sphere_name = ros_prefix+"/ambf/env/BODY Sphere";
    manip = std::make_shared<ContinuumManip>(base_name, m_worldPtr);
    manip_list.push_back(manip);
    test_objects_list.push_back(m_worldPtr->getRigidBody(sphere_name));
    for(size_t i=1; i<m_num_manip; i++){
        std::string new_base_name = base_name+std::to_string(i);
        auto ptr = std::make_shared<ContinuumManip>(new_base_name, m_worldPtr);
        manip_list.push_back(ptr);
        std::string new_sphere_name = sphere_name+std::to_string(i);
        test_objects_list.push_back(m_worldPtr->getRigidBody(new_sphere_name));
        
        auto T = ptr->m_contManipBaseRigidBody->getLocalTransform();
        T.setLocalPos(T.getLocalPos()+cVector3d(double(i),0.0,0.0));
        ptr->m_contManipBaseRigidBody->setLocalTransform(T);
    }

    obstacles_estimate_subpub = std::make_shared<ObstEstSubPub>("ambf", "obstacle_estimation");
    // manip = std::make_shared<ContinuumManip>("/ambf/env/BODY snake_stick1", m_worldPtr);

    // if (!m_contManipBaseRigidBody){
    //     cerr << "ERROR! FAILED TO FIND DRILL RIGID BODY NAMED " << cont_manip_rigid_body_name << endl;
    //     return -1;
    // }

    // for(int i = 1; i<=m_num_segs; i++){
    //     manip->m_segmentBodyList.push_back(m_worldPtr->getRigidBody("/ambf/env/BODY seg" + to_string(i)));
    //     manip->m_segmentJointList.push_back(m_worldPtr->getJoint("/ambf/env/JOINT joint" + to_string(i)));
    // }

    m_ambf_scale_to_mm = 0.01;

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();
    m_cablePullMagText = new cLabel(font);
    m_cablePullMagText->setLocalPos(20,70);
    m_cablePullMagText->m_fontColor.setBlack();
    m_cablePullMagText->setFontScale(.5);
    UpdateCablePullText();
    m_mainCamera->getFrontLayer()->addChild(m_cablePullMagText);

    m_cableControlModeText = new cLabel(font);
    m_cableControlModeText->setLocalPos(20,10);
    m_cableControlModeText->m_fontColor.setGreen();
    m_cableControlModeText->setFontScale(.5);
    m_cableControlModeText->setText("Cable Control Mode = Keyboard");
    m_mainCamera->getFrontLayer()->addChild(m_cableControlModeText);

    // Set up cable pull subscriber
    m_cablePullSub = new CablePullSubscriber("ambf", "volumetric_drilling");
    return 1;
}

void afContinuumManipSimplePlugin::graphicsUpdate(){
    UpdateCablePullText();
}

void afContinuumManipSimplePlugin::physicsUpdate(double dt){
    int obst_wait_iters = 100;
    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);
    applyCablePull(dt);
    // if (m_obstacle_estimate_enabled && m_counter > obst_wait_iters){
    //     obstacleEstimate();
    //     m_counter = -1;
    // }
    // m_counter ++;
    if (m_obstacle_estimate_enabled){
        obstacleEstimate();
    }
}

///
/// \brief incrementDevicePos
/// \param a_vel
///
void afContinuumManipSimplePlugin::incrementDevicePos(cVector3d a_vel){
    T_contmanip_base.setLocalPos(T_contmanip_base.getLocalPos() + a_vel);
}


///
/// \brief incrementDeviceRot
/// \param a_rot
///
void afContinuumManipSimplePlugin::incrementDeviceRot(cVector3d a_rot){
    cMatrix3d R_cmd;
    R_cmd.setExtrinsicEulerRotationDeg(a_rot(0), a_rot(1), a_rot(2), C_EULER_ORDER_XYZ);
    R_cmd = T_contmanip_base.getLocalRot() * R_cmd;
    T_contmanip_base.setLocalRot(R_cmd);
}

void afContinuumManipSimplePlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) {
    if (a_mods == GLFW_MOD_CONTROL){

        // controls linear motion of tool
        if (a_key == GLFW_KEY_W) {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol2() * m_drillRate; //z-direction
            incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_D) {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol0() * m_drillRate; //x-direction
            incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_S) {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol2() * m_drillRate; //z-direction
            incrementDevicePos(-dir);
        }

        else if (a_key == GLFW_KEY_A) {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol0() * m_drillRate; //x-direction
            incrementDevicePos(-dir);
        }

        else if (a_key == GLFW_KEY_K) {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol1() * m_drillRate; //y-direction
            incrementDevicePos(-dir);
        }

        else if (a_key == GLFW_KEY_I) {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol1() * m_drillRate; //y-direction
            incrementDevicePos(dir);
        }

        else if (a_key == GLFW_KEY_SEMICOLON) {
            if(m_cableKeyboardControl){
                m_cable_pull_mag_goal += 0.001;
            }
        }
        else if (a_key == GLFW_KEY_APOSTROPHE) {
            if(m_cableKeyboardControl){
                m_cable_pull_mag_goal -= 0.001;
            }
        }
         else if (a_key == GLFW_KEY_SLASH) {
            m_cableKeyboardControl = !m_cableKeyboardControl;
            std::string cable_control_mode = m_cableKeyboardControl?"Keyboard":"Subscriber";
            m_cableControlModeText->setText("Cable Control Mode = " + cable_control_mode);
        }

        else if (a_key == GLFW_KEY_G) {
            if(!m_obstacle_estimate_enabled){
                auto fitness = Fitness(this);
 //               pso::ParticleSwarmOptimization<double, afContinuumManipSimplePlugin::Fitness,
 //                   pso::ConstantWeight<double>,  afContinuumManipSimplePlugin::Callback<double> > optimizer(fitness, callback);
                optimizer = std::make_shared<pso::ParticleSwarmOptimization<double, afContinuumManipSimplePlugin::Fitness,
                    pso::ConstantWeight<double> > >(fitness);

                // Set number of iterations as stop criterion.
                // Set it to 0 or negative for infinite iterations (default is 0).
                optimizer->setMaxIterations(1000);

                // Set the minimum change of the x-values (particles) (default is 1e-6).
                // If the change in the current iteration is lower than this value, then
                // the optimizer stops minimizing.
                optimizer->setMinParticleChange(1e-6);

                // Set the minimum change of the function values (default is 1e-6).
                // If the change in the current iteration is lower than this value, then
                // the optimizer stops minimizing.
                optimizer->setMinFunctionChange(1e-6);

                // Set the number of threads used for evaluation (OpenMP only).
                // Set it to 0 or negative for auto detection (default is 1).
                optimizer->setThreads(2);

                // Turn verbosity on, so the optimizer prints status updates after each
                // iteration.
                optimizer->setVerbosity(2);

                // Set the bounds in which the optimizer should search.
                // Each column vector defines the (min, max) for each dimension  of the
                // particles.
                Eigen::MatrixXd bounds(2, 2);
                // auto first_seg_off = 0.3602972;
                // auto len = 0.6747028 - first_seg_off;
                bounds << 0, -1,
                        1, 1;

                // setup the optimization with a particle count
                optimizer->setup_minimize(bounds,m_num_manip);
        
                auto& particles = optimizer->interim_result.particles;
                // for(Index i = 0; i < particles.cols(); i++){
                // double r = particles(0,i);
                // double th = particles(1,i);
                // auto& man = manip_list[i];
                // auto& test_object = test_objects_list[i];
                // auto T_contmanipbase = man->m_contManipBaseRigidBody->getLocalTransform();
                // auto T_testobj = test_object->getLocalTransform();
                // auto first_seg_off = 0.3602972;
                // double p = 0.001;
                // T_testobj.setLocalPos(T_contmanipbase.getLocalPos()+cVector3d(r*std::cos(th),first_seg_off + r*std::sin(th),0.0));
                // test_object->setLocalTransform(T_testobj);
                // }
            }   
            m_obstacle_estimate_enabled = ! m_obstacle_estimate_enabled;         
        }
        else if (a_key == GLFW_KEY_H) {
            m_obstacle_estimate_idx += 1;
            if (m_obstacle_estimate_idx >= 27){ //TODO: don't hardcode num segs
                m_obstacle_estimate_idx = 0;
            }
            // std::cout << "m_obstacle_estimate_idx: " << m_obstacle_estimate_idx << std::endl;
        }

        else if (a_key == GLFW_KEY_Y){
        }
    }
    else{
        // controls rotational motion of tool
        if(a_key == GLFW_KEY_KP_5) {

            cVector3d rotDir(0, 1, 0) ;
            incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_8) {

            cVector3d rotDir(0, -1, 0);
            incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_4) {

            cVector3d rotDir(0, 0, -1);
            incrementDeviceRot(rotDir);
        }

        else if(a_key == GLFW_KEY_KP_6) {

            cVector3d rotDir(0, 0, 1);
            incrementDeviceRot(rotDir);
        }
    
    }   
}

void afContinuumManipSimplePlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes){

}

void afContinuumManipSimplePlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos){

}

void afContinuumManipSimplePlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
    T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform();
}

bool afContinuumManipSimplePlugin::close()
{
    return true;
}

void afContinuumManipSimplePlugin::applyCablePull(double dt){
    if(!m_cableKeyboardControl){
        m_cable_pull_mag_goal = m_cablePullSub->cable_pull_target;
    }
    double cable_pull_mag_err = (m_cable_pull_mag_goal - m_cable_pull_mag);
    double cable_pull_mag_change = 0.01*cable_pull_mag_err;
    if (abs(cable_pull_mag_change) > 0.00001){
        cable_pull_mag_change =  cable_pull_mag_change/abs(cable_pull_mag_change) * 0.00001;
    }
    m_cable_pull_mag += cable_pull_mag_change;
    m_cablePullSub->publishCablePullMeasured(m_cable_pull_mag);
    auto z = cVector3d(0.0, 0.0, 1.0);
    for(auto& man: manip_list){
        auto last_seg_ptr = man->m_segmentBodyList.back();
        last_seg_ptr->applyTorque((1.0/dt)*0.001*m_cable_pull_mag*last_seg_ptr->getLocalRot().getCol2());
    }
}

void afContinuumManipSimplePlugin::UpdateCablePullText(){
    m_cablePullMagText->setText("Cable Pull Actual(Goal): " + cStr(m_cable_pull_mag,5) + "(" + cStr(m_cable_pull_mag_goal, 5) + ")");
}

// void afContinuumManipSimplePlugin::optCallback(){
//     this->physicsUpdate(0.001);
// }

// template<typename Scalar>
// bool afContinuumManipSimplePlugin::optCallback(const Index, const Matrix&, const Vector &, const Index){
//     this->physicsUpdate(0.001);
//     return true;
// }
// bool afContinuumManipSimplePlugin::Callback::operator()(const Index, const Matrix&, const Vector &, const Index){
//     return true;
// }


// bool afContinuumManipSimplePlugin::Callback::operator()(const Index, const Matrix&, const Vector &, const Index)
//     {
//         this->physicsUpdate(0.001);
//         return true;
//     }

void afContinuumManipSimplePlugin::obstacleEstimate(){  //TODO: Implementation not finished
    /*if(m_counter > 100){    
    optimizer->run_one_interation_minimize(optimizer->interim_result.bestParticles);
    m_counter = -1;
    }
    m_counter ++;
    auto& bestParticles = optimizer->interim_result.bestParticles;
    for(Index i = 0; i < bestParticles.cols(); i++){
        double l_i = bestParticles(0,i);
        double f = bestParticles(1,i);
        auto& man = manip_list[i];
        int seg_idx = round(l_i*(m_num_segs-1));
        auto stiff = 10.0*std::pow(m_num_segs-seg_idx+1,2);
        auto err = man->m_segmentJointList[seg_idx]->getPosition()-m_goal_jp[i];
        auto p = 0.001;
        man->m_segmentBodyList[seg_idx]->applyTorque(cVector3d(0.0,0.0,-p*err));
        // man->m_segmentBodyList[seg_idx]->applyForce(cVector3d(f,0.0,0.0));

        std::cout << "l_i:" << l_i << ", seg_idx: " << seg_idx << ", f: " << f << ", cost: " << optimizer->objective_(optimizer->interim_result.xval, i) << std::endl;

    } 
    


            std::cout << std::endl ;
            */
    if(m_counter > 100){ // publish at slower rate
        m_goal_jp = obstacles_estimate_subpub->js_goal;
    }    

    std::vector<float> obj_value;
    for(Index i = 0; i < m_num_manip; i++){
        auto& man = manip_list[i];
        auto err = man->m_segmentJointList[i]->getPosition()-m_goal_jp[i];
        auto p = 0.1;
        auto dir_sign = (i%2)?1.0:-1.0;
        man->m_mag_cmd += dir_sign*p*err;
        // man->m_segmentBodyList[i]->applyTorque(cVector3d(0.0,0.0,man->m_mag_cmd));
        man->m_segmentBodyList[i]->applyForce(cVector3d(man->m_mag_cmd,0.0,0.0));

        // man->m_segmentJointList[i]->commandEffort(man->m_mag_cmd);
        obj_value.push_back(optimizer->objective_(optimizer->interim_result.xval, i));

        std::cout << "i:" << i << ", err: " << err << ", f: " << man->m_mag_cmd << ", cost: " << obj_value[i] << std::endl;
    }
    std::cout << "Best Estimate: " << std::min_element(obj_value.begin(),obj_value.end()) - obj_value.begin();
    
    if(m_counter > 100){ // publish at slower rate
        obstacles_estimate_subpub->publish_costs(obj_value);
        m_counter = -1;
    }
    m_counter ++;



}

double afContinuumManipSimplePlugin::GetRandFromUniform()
{
    return ((double)rand()/(double)RAND_MAX);
}