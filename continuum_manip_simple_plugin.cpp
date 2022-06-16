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


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

int afContinuumManipSimplePlugin::init(int argc, char **argv, const afWorldPtr a_afWorld){

    // Bring in ambf world, make adjustments as needed to improve simulation accuracy
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

    int num_manip = 3;
    std::string base_name = "/ambf/env/BODY snake_stick";
    std::string sphere_name = "/ambf/env/BODY Sphere";
    manip = std::make_shared<ContinuumManip>(base_name, m_worldPtr);
    manip_list.push_back(manip);
    test_objects_list.push_back(m_worldPtr->getRigidBody(sphere_name));
    for(size_t i=1; i<num_manip; i++){
        std::string new_base_name = base_name+std::to_string(i);
        auto ptr = std::make_shared<ContinuumManip>(new_base_name, m_worldPtr);
        manip_list.push_back(ptr);
        std::string new_sphere_name = sphere_name+std::to_string(i);
        test_objects_list.push_back(m_worldPtr->getRigidBody(new_sphere_name));
        
        auto T = ptr->m_contManipBaseRigidBody->getLocalTransform();
        T.setLocalPos(T.getLocalPos()+cVector3d(double(i),0.0,0.0));
        ptr->m_contManipBaseRigidBody->setLocalTransform(T);
    }


    
    
   


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

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);
    applyCablePull(dt);
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
            m_obstacle_estimate_enabled = ! m_obstacle_estimate_enabled;
            if(m_obstacle_estimate_enabled){
                // auto test_voxel = m_worldPtr->getRigidBody("Sphere");
                for(size_t i=0; i<manip_list.size(); i++){
                    auto& man = manip_list[i];
                    auto& test_object = test_objects_list[i];
                    auto Tseg = man->m_segmentBodyList[m_obstacle_estimate_idx]->getLocalTransform();
                    double cm_rad = 0.03;
                    double test_voxel_rad = 0.005;
                    double offset = 0.001;
                    double dir_sign = (m_obstacle_estimate_idx%2)?1.0:-1.0;
                    Tseg.setLocalPos(Tseg.getLocalPos()+cVector3d( dir_sign*(cm_rad+test_voxel_rad+offset),0.0,0.0));
                    test_object->setLocalTransform(Tseg);
                }
            }        
        }
        else if (a_key == GLFW_KEY_H) {
            m_obstacle_estimate_idx += 1;
            if (m_obstacle_estimate_idx >= 27){ //TODO: don't hardcode num segs
                m_obstacle_estimate_idx = 0;
            }
            std::cout << "m_obstacle_estimate_idx: " << m_obstacle_estimate_idx << std::endl;
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

void afContinuumManipSimplePlugin::obstacleEstimate(){  //TODO: Implementation not finished
    std::vector<double> goal_jp = {0.04719538614153862, 0.07040780782699585, 0.0539616122841835, 0.04232475906610489, 0.01811256818473339, 0.010549008846282959, -0.00471153948456049, -0.0024778724182397127, -0.021339787170290947, -0.02025371417403221, -0.02914055995643139, -0.022847743704915047, -0.0291144922375679, -0.023115238174796104, -0.02983745187520981, -0.0232921801507473, -0.02943757176399231, -0.023809820413589478, -0.02989761345088482, -0.02374465949833393, -0.0301345381885767, -0.024002499878406525, -0.030253717675805092, -0.023892860859632492, -0.030802471563220024, -0.02734425663948059, -0.049160074442625046};
    for(size_t i=0; i<manip_list.size(); i++){
    auto& man = manip_list[i];
    auto& test_object = test_objects_list[i];
        std::vector<double> jp;
       

            for(auto& joint: man->m_segmentJointList){
                jp.push_back(joint->getPosition());
            }
        
        auto total_err = 0.0;
        auto real_err = 0.0;
        for (size_t i=0; i<m_num_segs; i++){
            total_err += jp[i]-goal_jp[i];
            real_err += abs(jp[i]-goal_jp[i]);
            std::cout << jp[i]-goal_jp[i] << ", ";

        }
        std::cout << std::endl;
        // auto test_voxel = m_worldPtr->getRigidBody("Sphere");
        // auto Tvox = test_voxel->getLocalTransform();


            for (size_t i=0; i<m_num_segs; i++){
                if(i == m_obstacle_estimate_idx){
                    auto p = 0.8*0.001;
                    double dir_sign = (i%2)?1.0:-1.0;
                    auto unsigned_cmd = p*(total_err);// + d[i]*(jp[i]-m_old_jp[i]);
                    double cmd = dir_sign*unsigned_cmd;
                    double offset = 0.001;
                    auto Tseg = man->m_segmentBodyList[m_obstacle_estimate_idx]->getLocalTransform();
                    double cm_rad = 0.03;
                    double test_voxel_rad = 0.005;
                    auto x_pos = cVector3d( dir_sign*(cm_rad+test_voxel_rad+offset),0.0,0.0);
                    auto ideal_orig_loc = Tseg*x_pos;
                    auto Tvox = test_object->getLocalTransform();
                    auto y_loc = Tvox.getLocalPos();
                    Tvox.setLocalPos(Tvox.getLocalPos()+cmd*x_pos);
                    test_object->setLocalTransform(Tvox);
                    std::cout << "cmd*x_pos: " << cmd*ideal_orig_loc << std::endl;
                }
            }
            m_old_jp = jp;
            std::cout << std::endl;
            std::cout << "Error: " << total_err << std::endl;
            std::cout << "Mag Error: " << real_err << std::endl;
        }
}
