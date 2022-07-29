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

#include "continuum_manip_volumetric_drilling_plugin.h"
#include <boost/program_options.hpp>
#include <fstream>

using namespace std;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

int afVolmetricDrillingPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld){

    namespace p_opt = boost::program_options;
    p_opt::options_description cmd_opts("drilling_simulator Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info")
            ("anatomy_volume_name", p_opt::value<std::string>()->default_value("cube"), "Name of volume given in yaml. Default spine_test_volume")
            ("base_body_name", p_opt::value<std::string>()->default_value("snake_stick"), "Name of body given in yaml. Default snake_stick")
            ("tool_body_name", p_opt::value<std::string>()->default_value("Burr"), "Name of body given in yaml. Default Burr");
    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    string file_path = __FILE__;
    string cur_path = file_path.substr(0, file_path.rfind("/"));

    std::string anatomy_volume_name = var_map["anatomy_volume_name"].as<std::string>();
    std::string base_body_name = var_map["base_body_name"].as<std::string>();
    std::string tool_body_name = var_map["tool_body_name"].as<std::string>();

    // Bring in ambf world, make adjustments as needed to improve simulation accuracy
    m_worldPtr = a_afWorld;
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp=1.0; // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2=1.0; // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->setGravity(btVector3(0.0,0.0,0.0));
    // Get chai3D world pointer
    m_chaiWorldPtr = m_worldPtr->getChaiWorld();

    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);
    m_boneColor = cColorb(255, 249, 219, 255);
    m_storedColor = cColorb(0x00, 0x00, 0x00, 0x00);
    
    // Get first camera
    m_mainCamera = m_worldPtr->getCameras()[0];
    
    // Importing continuum manipulator model
    m_contManipBaseRigidBody = m_worldPtr->getRigidBody(base_body_name);
    if (!m_contManipBaseRigidBody){
        cerr << "ERROR! FAILED TO FIND RIGID BODY NAMED " << base_body_name << endl;
        return -1;
    }
    T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform();
    
    // Import anatomy volume
    m_volumeObject = m_worldPtr->getVolume(anatomy_volume_name);
    if (!m_volumeObject){
        cerr << "ERROR! FAILED TO FIND VOLUME NAMED " << anatomy_volume_name << endl;
        return -1;
    }
    m_voxelObj = m_volumeObject->getInternalVolume();
 
    // Various scalars needed for other calculation
    m_ambf_scale_to_mm = 0.01;
    double burr_r = m_ambf_scale_to_mm * 6.5/2;

    // create a haptic device handler
    m_deviceHandler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    m_deviceHandler->getDevice(m_hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = m_hapticDevice->getSpecifications();

    // Initializing tool cursors
    toolCursorInit(a_afWorld);
    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = m_shaftToolCursorList[0]->getWorkspaceScaleFactor();
    // stiffness properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // Set voxels surface contact properties
    m_voxelObj->m_material->setStiffness(2.0*maxStiffness);
    m_voxelObj->m_material->setDamping(0.0);
    m_voxelObj->m_material->setDynamicFriction(0.0);
    m_voxelObj->setUseMaterial(true);

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();
    m_cablePullMagText = new cLabel(font);
    m_cablePullMagText->setLocalPos(20,70);
    m_cablePullMagText->m_fontColor.setBlack();
    m_cablePullMagText->setFontScale(.5);
    UpdateCablePullText();
    m_mainCamera->getFrontLayer()->addChild(m_cablePullMagText);

    m_drillControlModeText = new cLabel(font);
    m_drillControlModeText->setLocalPos(20,30);
    m_drillControlModeText->m_fontColor.setGreen();
    m_drillControlModeText->setFontScale(.5);
    m_drillControlModeText->setText("Drill Control Mode = Haptic Device / Keyboard");
    m_mainCamera->getFrontLayer()->addChild(m_drillControlModeText);

    m_cableControlModeText = new cLabel(font);
    m_cableControlModeText->setLocalPos(20,10);
    m_cableControlModeText->m_fontColor.setGreen();
    m_cableControlModeText->setFontScale(.5);
    m_cableControlModeText->setText("Cable Control Mode = Keyboard");
    m_mainCamera->getFrontLayer()->addChild(m_cableControlModeText);

    // Get drills initial pose
    T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform();

    // Set up voxels_removed publisher
    m_drillingPub = new DrillingPublisher("ambf", "volumetric_drilling");
    
    // Volume Properties
    float dim[3];
    dim[0] = m_volumeObject->getDimensions().get(0);
    dim[1]= m_volumeObject->getDimensions().get(1);
    dim[2] = m_volumeObject->getDimensions().get(2);
    int voxelCount[3];
    voxelCount[0] = m_volumeObject->getVoxelCount().get(0);
    voxelCount[1]= m_volumeObject->getVoxelCount().get(1);
    voxelCount[2] = m_volumeObject->getVoxelCount().get(2);
    m_drillingPub -> volumeProp(dim, voxelCount);

    // Set up cable pull subscriber
    m_cablePullSub = new CablePullSubscriber("ambf", "volumetric_drilling");
    return 1;
}

void afVolmetricDrillingPlugin::graphicsUpdate(){
    UpdateCablePullText();
    // update region of voxels to be updated
    if (m_flagMarkVolumeForUpdate)
    {
        m_mutexVoxel.acquire();
        cVector3d min = m_volumeUpdate.m_min;
        cVector3d max = m_volumeUpdate.m_max;
        m_volumeUpdate.setEmpty();
        m_mutexVoxel.release();
        ((cTexture3d*)m_voxelObj->m_texture.get())->markForPartialUpdate(min, max);
        m_flagMarkVolumeForUpdate = false;
    }
}

void afVolmetricDrillingPlugin::physicsUpdate(double dt){

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);
    bool clutch;

    // If a valid haptic device is found, then it should be available
    if (getOverrideDrillControl()){
        T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform();
    }
    else if(m_hapticDevice->isDeviceAvailable()){
        m_hapticDevice->getTransform(T_i);
        m_hapticDevice->getLinearVelocity(V_i);
        m_hapticDevice->getUserSwitch(0, clutch);
        V_i =  m_mainCamera->getLocalRot() * (V_i * !clutch / m_shaftToolCursorList[0]->getWorkspaceScaleFactor());
        T_contmanip_base.setLocalPos(T_contmanip_base.getLocalPos() + V_i);
        T_contmanip_base.setLocalRot(m_mainCamera->getLocalRot() * T_i.getLocalRot());
    }

    toolCursorsPosUpdate(T_contmanip_base);

    if (m_volume_collisions_enabled){
        // check for shaft collision
        checkShaftCollision();

        if (getOverrideDrillControl() == false){
            // updates position of drill mesh
            drillPoseUpdateFromCursors();
        }

        cToolCursor* burr_cursor = m_burrToolCursorList.back();
        if (m_burrOn && burr_cursor->isInContact(m_voxelObj) )//&& m_targetToolCursorIdx == 0 /*&& (userSwitches == 2)*/)
        {
            for (int ci = 0 ; ci < 3 ; ci++){
                // retrieve contact event
                cCollisionEvent* contact = burr_cursor->m_hapticPoint->getCollisionEvent(ci);

                cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);

                m_voxelObj->m_texture->m_image->getVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_storedColor);

                m_voxelObj->m_texture->m_image->setVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_zeroColor);

                //Publisher for voxels removed
                if(m_storedColor != m_zeroColor)
                {
                    double sim_time = m_contManipBaseRigidBody->getCurrentTimeStamp();

                    double voxel_array[3] = {orig.get(0), orig.get(1), orig.get(2)};

                    cColorf color_glFloat = m_storedColor.getColorf();
                    float color_array[4];
                    color_array[0] = color_glFloat.getR();
                    color_array[1] = color_glFloat.getG();
                    color_array[2] = color_glFloat.getB();
                    color_array[3] = color_glFloat.getA();


                    m_drillingPub -> voxelsRemoved(voxel_array,color_array,sim_time);
                }

                m_mutexVoxel.acquire();
                m_volumeUpdate.enclose(cVector3d(uint(orig.x()), uint(orig.y()), uint(orig.z())));
                m_mutexVoxel.release();
                // mark voxel for update
            }

            m_flagMarkVolumeForUpdate = true;
        }
        // compute interaction forces
        for( auto& cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList}){
            for( auto& cursor : cursor_list){
                cursor->computeInteractionForces();
            }
        }
        //apply forces to segments
        for ( int i=0; i<m_segmentBodyList.size(); i++){
            m_segmentBodyList[i]->applyForce(100000.0*m_segmentToolCursorList[i]->getDeviceLocalForce());
        }
        // apply force from burr
        m_burrBody->applyForce(100000.0*m_burrToolCursorList[0]->getDeviceLocalForce());
        

        // check if device remains stuck inside     voxel object
        // Also orient the force to match the camera rotation
        cVector3d force = cTranspose(m_mainCamera->getLocalRot()) * m_targetToolCursor->getDeviceLocalForce();
        // m_toolCursorList[0]->setDeviceLocalForce(force);
        // for ( int i=0; i<m_segmentBodyList.size(); i++){
        //     m_segmentBodyList[i]->applyForce(100000.0*m_segmentToolCursorList[i]->getDeviceLocalForce());
        //     // std::cout << m_toolCursorList[i+1]->getDeviceLocalForce() << std::endl;
        // }
    }

    applyCablePull(dt);

    /////////////////////////////////////////////////////////////////////////
    // MANIPULATION
    /////////////////////////////////////////////////////////////////////////

    // compute transformation from world to tool (haptic device)
    cToolCursor* shaft_cursor = m_shaftToolCursorList.front();

    cTransform world_T_tool = shaft_cursor->getDeviceLocalTransform();
    // std::cout << "WORLD_T_TOOL: " << world_T_tool.getLocalPos() << std::endl;
    // get status of user switch
    bool button = shaft_cursor->getUserSwitch(1);
    //
    // STATE 1:
    // Idle mode - user presses the user switch
    //
    if ((m_controlMode == HAPTIC_IDLE) && (button == true))
    {
        // check if at least one contact has occurred
        if (shaft_cursor->m_hapticPoint->getNumCollisionEvents() > 0)
        {
            // get contact event
            cCollisionEvent* collisionEvent = shaft_cursor->m_hapticPoint->getCollisionEvent(0);

            // get object from contact event
            m_selectedObject = collisionEvent->m_object;
        }
        else
        {
            m_selectedObject = m_voxelObj;
        }

        // get transformation from object
        cTransform world_T_object = m_selectedObject->getLocalTransform();

        // compute inverse transformation from contact point to object
        cTransform tool_T_world = world_T_tool;
        tool_T_world.invert();

        // store current transformation tool
        m_tool_T_object = tool_T_world * world_T_object;

        // update state
        m_controlMode = HAPTIC_SELECTION;
    }


    //
    // STATE 2:
    // Selection mode - operator maintains user switch enabled and moves object
    //
    else if ((m_controlMode == HAPTIC_SELECTION) && (button == true))
    {
        // compute new transformation of object in global coordinates
        cTransform world_T_object = world_T_tool * m_tool_T_object;

        // compute new transformation of object in local coordinates
        cTransform parent_T_world = m_selectedObject->getParent()->getLocalTransform();
        parent_T_world.invert();
        cTransform parent_T_object = parent_T_world * world_T_object;

        // assign new local transformation to object
        if (m_selectedObject == m_voxelObj){
            m_volumeObject->setLocalTransform(parent_T_object);
        }

        // set zero forces when manipulating objects
        shaft_cursor->setDeviceLocalForce(0.0, 0.0, 0.0);

        shaft_cursor->initialize();
    }

    //
    // STATE 3:
    // Finalize Selection mode - operator releases user switch.
    //
    else
    {
        m_controlMode = HAPTIC_IDLE;
    }

    /////////////////////////////////////////////////////////////////////////
    // FINALIZE
    /////////////////////////////////////////////////////////////////////////

    // send forces to haptic device
    if (getOverrideDrillControl() == false){
        shaft_cursor->applyToDevice();
    }

}

///
/// \brief This method initializes the tool cursors.
/// \param a_afWorld    A world that contains all objects of the virtual environment
/// \return
///
void afVolmetricDrillingPlugin::toolCursorInit(const afWorldPtr a_afWorld){
    cWorld* chai_world = a_afWorld->getChaiWorld();
    int num_segs = 27;
    int num_shaft_cursor = 1;
    int num_burr_cursor = 1;

    for(int i = 1; i<=num_segs; i++){
        m_segmentBodyList.push_back(m_worldPtr->getRigidBody("/ambf/env/BODY seg" + to_string(i)));
        m_segmentJointList.push_back(m_worldPtr->getJoint("/ambf/env/JOINT joint" + to_string(i)));
        auto seg_cursor = new cToolCursor(chai_world);
        m_segmentToolCursorList.push_back(seg_cursor);
        // m_worldPtr->addSceneObjectToWorld(seg_cursor);
    }
    for(int i = 0; i<num_shaft_cursor; i++){
        auto shaft_cursor = new cToolCursor(chai_world);
        m_shaftToolCursorList.push_back(shaft_cursor);
        // m_worldPtr->addSceneObjectToWorld(shaft_cursor);
    }
    for(int i = 0; i<num_burr_cursor; i++){
        auto burr_cursor = new cToolCursor(chai_world);
        m_burrToolCursorList.push_back(burr_cursor);
        m_burrBody = m_worldPtr->getRigidBody("/ambf/env/BODY Burr");
        // m_worldPtr->addSceneObjectToWorld(burr_cursor);
    }

    for( auto& shaft_cursor : m_shaftToolCursorList){
        // std::cout << shaft_cursor << std::endl;
        shaft_cursor->setHapticDevice(m_hapticDevice);

        // map the physical workspace of the haptic device to a larger virtual workspace.
        shaft_cursor->setWorkspaceRadius(10.0);
        shaft_cursor->setWaitForSmallForce(true);
        shaft_cursor->start();
        shaft_cursor->m_hapticPoint->m_sphereProxy->setShowFrame(false);

        shaft_cursor->m_name = "snake_shaft";
        shaft_cursor->m_hapticPoint->setShow(m_showGoalProxySpheres, m_showGoalProxySpheres);
        shaft_cursor->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
        shaft_cursor->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();

        // if the haptic device has a gripper, enable it as a user switch
        m_hapticDevice->setEnableGripperUserSwitch(true);
        shaft_cursor->setRadius(m_ambf_scale_to_mm*6/2); 
    }

    for( auto& burr_cursor : m_burrToolCursorList){
        burr_cursor->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
        burr_cursor->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
        burr_cursor->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
        burr_cursor->setRadius(m_ambf_scale_to_mm*6.5/2);
    }

    for( auto& seg_cursor : m_segmentToolCursorList){
        seg_cursor->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
        seg_cursor->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
        seg_cursor->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
        seg_cursor->setRadius(m_ambf_scale_to_mm*6/2);
    }
    // Initialize the start pose of the tool cursors
    toolCursorsPosUpdate(T_contmanip_base);
    for( auto& cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList}){
            for( auto& cursor : cursor_list){
                cursor->initialize();
                m_worldPtr->addSceneObjectToWorld(cursor);

        }
    }
}

///
/// \brief incrementDevicePos
/// \param a_vel
///
void afVolmetricDrillingPlugin::incrementDevicePos(cVector3d a_vel){
    T_contmanip_base.setLocalPos(T_contmanip_base.getLocalPos() + a_vel);
}


///
/// \brief incrementDeviceRot
/// \param a_rot
///
void afVolmetricDrillingPlugin::incrementDeviceRot(cVector3d a_rot){
    cMatrix3d R_cmd;
    R_cmd.setExtrinsicEulerRotationDeg(a_rot(0), a_rot(1), a_rot(2), C_EULER_ORDER_XYZ);
    R_cmd = T_contmanip_base.getLocalRot() * R_cmd;
    T_contmanip_base.setLocalRot(R_cmd);
}

///
/// \brief This method updates the position of the shaft tool cursors
/// which eventually updates the position of the whole tool.
///
void afVolmetricDrillingPlugin::toolCursorsPosUpdate(cTransform a_targetPose){
    for( auto& shaft_cursor : m_shaftToolCursorList){
        shaft_cursor->setDeviceLocalTransform(a_targetPose);
    }

    for( auto& burr_cursor : m_burrToolCursorList){
        burr_cursor->setDeviceLocalTransform(m_burrBody->getLocalTransform());
    }


    for (int i=0; i<m_segmentToolCursorList.size(); i++){
        m_segmentToolCursorList[i]->setDeviceLocalTransform(m_segmentBodyList[i]->getLocalTransform());
    }
}

///
/// \brief This method checks for collision between the tool shaft and the volume.
/// The error between the proxy and goal position of each of the shaft tool cursors is constantly
/// computed. The shaft tool cursor having the maximum error is set as g_targetToolCursor. Further, the
/// position of the drill mesh is set such that it follows the proxy position of the g_targetToolCursor.
/// If there's no collision, the drill mesh follows the proxy position of the shaft tool cursor which is
/// closest to the tip tool cursor.
///
void afVolmetricDrillingPlugin::checkShaftCollision(){

    m_maxError = 0;
    m_targetToolCursor = m_shaftToolCursorList[0];
    m_targetToolCursorIdx = 0;
    for(int i=0; i<m_shaftToolCursorList.size(); i++)
    {

        m_currError = cDistance(m_shaftToolCursorList[i]->m_hapticPoint->getLocalPosProxy(), m_shaftToolCursorList[i]->m_hapticPoint->getLocalPosGoal());

        if(abs(m_currError) > abs(m_maxError + 0.00001))
        {
            m_maxError = m_currError;
            m_targetToolCursor = m_shaftToolCursorList[i];
            m_targetToolCursorIdx = i;
        }
    }
}


///
/// \brief This method updates the position of the drill mesh.
/// After obtaining g_targetToolCursor, the drill mesh adjust it's position and rotation
/// such that it follows the proxy position of the g_targetToolCursor.
///
void afVolmetricDrillingPlugin::drillPoseUpdateFromCursors(){
    cMatrix3d newContManipBaseRot;
    newContManipBaseRot = m_shaftToolCursorList[0]->getDeviceLocalRot();
    cTransform T_newContManipBase;
    T_newContManipBase.setLocalPos(m_shaftToolCursorList[0]->m_hapticPoint->getLocalPosProxy());
    T_newContManipBase.setLocalRot(newContManipBaseRot);
    T_newContManipBase = T_newContManipBase * btTransformTocTransform(m_contManipBaseRigidBody->getInertialOffsetTransform()); // handle offset due to fact that origin is not at center of body
    m_contManipBaseRigidBody->setLocalTransform(T_newContManipBase);
}

void afVolmetricDrillingPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) {
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

        else if (a_key == GLFW_KEY_O) {
            setOverrideDrillControl(!getOverrideDrillControl());
            if (getOverrideDrillControl()){
                m_drillControlModeText->m_fontColor.setRed();
                m_drillControlModeText->setText("Drill Control Mode = External afComm");
            }
            else{
                m_drillControlModeText->m_fontColor.setGreen();
                m_drillControlModeText->setText("Drill Control Mode = Haptic Device / Keyboard");
            }
        }

        else if (a_key == GLFW_KEY_C) {
            m_showGoalProxySpheres = !m_showGoalProxySpheres;
            for( auto& cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList}){
                for( auto& cursor : cursor_list){
                    cursor->m_hapticPoint->setShow(m_showGoalProxySpheres, m_showGoalProxySpheres);
                }
             }
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
        else if (a_key == GLFW_KEY_LEFT_BRACKET) {
            m_volume_collisions_enabled = !m_volume_collisions_enabled;
        }
        else if (a_key == GLFW_KEY_RIGHT_BRACKET) {
            toolCursorInit(m_worldPtr);

        }
        else if (a_key == GLFW_KEY_EQUAL) {
            m_burrOn = !m_burrOn;
            std::cout << "Burr On State: " << m_burrOn << std::endl;
        }

        else if (a_key == GLFW_KEY_SLASH) {
            m_cableKeyboardControl = !m_cableKeyboardControl;
            std::string cable_control_mode = m_cableKeyboardControl?"Keyboard":"Subscriber";
            m_cableKeyboardControl?m_cableControlModeText->m_fontColor.setGreen():m_cableControlModeText->m_fontColor.setRed();
            m_cableControlModeText->setText("Cable Control Mode = " + cable_control_mode);
        }

        else if (a_key == GLFW_KEY_E) {
            bool paused = m_worldPtr->isPhysicsPaused();
            if(paused){
                m_worldPtr->pausePhysics(false);
            }    
            else{
                auto last_seg_ptr = m_segmentBodyList.back();
                last_seg_ptr->applyTorque(cVector3d(0.0,0.0,0.0));
                m_worldPtr->pausePhysics(true);
            }
            std::cout << "Toggled plugin physics paused to: " << m_worldPtr->isPhysicsPaused() << std::endl;
        }


        // option - polygonize model and save to file
        else if (a_key == GLFW_KEY_F9) {
            cMultiMesh *surface = new cMultiMesh;
            m_voxelObj->polygonize(surface, 0.01, 0.01, 0.01);
            double SCALE = 0.1;
            double METERS_TO_MILLIMETERS = 1000.0;
            surface->scale(SCALE * METERS_TO_MILLIMETERS);
            surface->setUseVertexColors(true);
            surface->saveToFile("volume.obj");
            cout << "> Volume has been polygonized and saved to disk                            \r";
            delete surface;
        }

    }
    else{

        // option - reduce size along X axis
        if (a_key == GLFW_KEY_4) {
            double value = cClamp((m_voxelObj->m_maxCorner.x() - 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.x(value);
            m_voxelObj->m_minCorner.x(-value);
            m_voxelObj->m_maxTextureCoord.x(0.5 + value);
            m_voxelObj->m_minTextureCoord.x(0.5 - value);
            cout << "> Reduce size along X axis.                            \r";
        }

        // option - increase size along X axis
        else if (a_key == GLFW_KEY_5) {
            double value = cClamp((m_voxelObj->m_maxCorner.x() + 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.x(value);
            m_voxelObj->m_minCorner.x(-value);
            m_voxelObj->m_maxTextureCoord.x(0.5 + value);
            m_voxelObj->m_minTextureCoord.x(0.5 - value);
            cout << "> Increase size along X axis.                            \r";
        }

        // option - reduce size along Y axis
        else if (a_key == GLFW_KEY_6) {
            double value = cClamp((m_voxelObj->m_maxCorner.y() - 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.y(value);
            m_voxelObj->m_minCorner.y(-value);
            m_voxelObj->m_maxTextureCoord.y(0.5 + value);
            m_voxelObj->m_minTextureCoord.y(0.5 - value);
            cout << "> Reduce size along Y axis.                            \r";
        }

        // option - increase size along Y axis
        else if (a_key == GLFW_KEY_7) {
            double value = cClamp((m_voxelObj->m_maxCorner.y() + 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.y(value);
            m_voxelObj->m_minCorner.y(-value);
            m_voxelObj->m_maxTextureCoord.y(0.5 + value);
            m_voxelObj->m_minTextureCoord.y(0.5 - value);
            cout << "> Increase size along Y axis.                            \r";
        }

        // option - reduce size along Z axis
        else if (a_key == GLFW_KEY_8) {
            double value = cClamp((m_voxelObj->m_maxCorner.z() - 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.z(value);
            m_voxelObj->m_minCorner.z(-value);
            m_voxelObj->m_maxTextureCoord.z(0.5 + value);
            m_voxelObj->m_minTextureCoord.z(0.5 - value);
            cout << "> Reduce size along Z axis.                            \r";
        }

        // option - increase size along Z axis
        else if (a_key == GLFW_KEY_9) {
            double value = cClamp((m_voxelObj->m_maxCorner.z() + 0.005), 0.01, 0.5);
            m_voxelObj->m_maxCorner.z(value);
            m_voxelObj->m_minCorner.z(-value);
            m_voxelObj->m_maxTextureCoord.z(0.5 + value);
            m_voxelObj->m_minTextureCoord.z(0.5 - value);
            cout << "> Increase size along Z axis.                            \r";
        }
        // option - decrease quality of graphic rendering
        else if (a_key == GLFW_KEY_L) {
            double value = m_voxelObj->getQuality();
            m_voxelObj->setQuality(value - 0.01);
            cout << "> Quality set to " << cStr(m_voxelObj->getQuality(), 1) << "                            \r";
        }

        // option - increase quality of graphic rendering
        else if (a_key == GLFW_KEY_U) {
            double value = m_voxelObj->getQuality();
            m_voxelObj->setQuality(value + 0.01);
            cout << "> Quality set to " << cStr(m_voxelObj->getQuality(), 1) << "                            \r";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_UP) {
            double value = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(value + 0.01);
            cout << "> Opacity Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_DOWN) {
            double value = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(value - 0.01);
            cout << "> Opacity Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_RIGHT) {
            double value = m_voxelObj->getIsosurfaceValue();
            m_voxelObj->setIsosurfaceValue(value + 0.01);
            cout << "> Isosurface Threshold set to " << cStr(m_voxelObj->getIsosurfaceValue(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_LEFT) {
            double value = m_voxelObj->getIsosurfaceValue();
            m_voxelObj->setIsosurfaceValue(value - 0.01);
            cout << "> Isosurface Threshold set to " << cStr(m_voxelObj->getIsosurfaceValue(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_ENTER) {
            m_renderingMode++;
            if (m_renderingMode > 7) {
                m_renderingMode = 0;
            }
            switch (m_renderingMode) {
            case 0:
                m_voxelObj->setRenderingModeBasic();
                std::cerr << "setRenderingModeBasic" << std::endl;
                break;
            case 1:
                m_voxelObj->setRenderingModeVoxelColors();
                std::cerr << "setRenderingModeVoxelColors" << std::endl;
                break;
            case 2:
                m_voxelObj->setRenderingModeVoxelColorMap();
                std::cerr << "setRenderingModeVoxelColorMap" << std::endl;
                break;
            case 3:
                m_voxelObj->setRenderingModeIsosurfaceColors();
                std::cerr << "setRenderingModeIsosurfaceColors" << std::endl;
                break;
            case 4:
                m_voxelObj->setRenderingModeIsosurfaceMaterial();
                std::cerr << "setRenderingModeIsosurfaceMaterial" << std::endl;
                break;
            case 5:
                m_voxelObj->setRenderingModeIsosurfaceColorMap();
                std::cerr << "setRenderingModeIsosurfaceColorMap" << std::endl;
                break;
            case 6:
                m_voxelObj->setRenderingModeDVRColorMap();
                std::cerr << "setRenderingModeDVRColorMap" << std::endl;
                break;
            case 7:
                m_voxelObj->setRenderingModeCustom();
                std::cerr << "setRenderingModeCustom" << std::endl;
                break;
            default:
                break;
            }
        } else if (a_key == GLFW_KEY_PAGE_UP) {
            m_opticalDensity += 0.1;
            m_voxelObj->setOpticalDensity(m_opticalDensity);
            cout << "> Optical Density set to " << cStr(m_opticalDensity, 1) << "                            \n";
        } else if (a_key == GLFW_KEY_PAGE_DOWN) {
            m_opticalDensity -= 0.1;
            m_voxelObj->setOpticalDensity(m_opticalDensity);
            cout << "> Optical Density set to " << cStr(m_opticalDensity, 1) << "                            \n";
        } else if (a_key == GLFW_KEY_HOME) {
            float val = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(val + 0.1);
            cout << "> Optical Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        } else if (a_key == GLFW_KEY_END) {
            float val = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(val - 0.1);
            cout << "> Optical Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // controls rotational motion of tool
        else if(a_key == GLFW_KEY_KP_5) {

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

        // toggles the functionality of sudden jumping of drill mesh towards the followSphere
        else if(a_key == GLFW_KEY_X){

            if(m_suddenJump)
            {
                m_suddenJump = false;
            }

            else
            {
                m_suddenJump = true;
            }
        }

        // toggles the visibility of drill mesh in the scene
        else if (a_key == GLFW_KEY_B){
            m_showDrill = !m_showDrill;
            m_contManipBaseRigidBody->m_visualMesh->setShowEnabled(m_showDrill);
            for(auto& seg: m_segmentBodyList){
                seg->m_visualMesh->setShowEnabled(m_showDrill);
            }
            m_burrBody->m_visualMesh->setShowEnabled(m_showDrill);
        }
    }
}


void afVolmetricDrillingPlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes){

}

void afVolmetricDrillingPlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos){

}

void afVolmetricDrillingPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
    T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform();
}

bool afVolmetricDrillingPlugin::close()
{
    for(auto tool : m_toolCursorList)
    {
        tool->stop();
    }

    delete m_deviceHandler;

    return true;
}

void afVolmetricDrillingPlugin::applyCablePull(double dt){    
    // Set position goal if applicable
    double cable_pull_mag_change;
    if(m_cableKeyboardControl || m_cablePullSub->command_type == cable_pull_command_type::POSITION){
        if(m_cableKeyboardControl){}//m_cable_pull_mag_goal already set by keyboard commands
        else{
            if(m_cablePullSub->command_type != cable_pull_command_type::POSITION) //sanity check
                {std::cerr << "conflicting cable pull commands" << std::endl;}
            m_cable_pull_mag_goal = m_cablePullSub->cable_pull_position_target;
        }
        double cable_pull_mag_err = (m_cable_pull_mag_goal - m_cable_pull_mag);
        cable_pull_mag_change = 0.01*cable_pull_mag_err;
        if (abs(cable_pull_mag_change) > 0.00001){
            cable_pull_mag_change =  cable_pull_mag_change/abs(cable_pull_mag_change) * 0.00001;
        }
    }
    else{ 
        if(m_cablePullSub->command_type != cable_pull_command_type::VELOCITY) //sanity check
            {std::cerr << "conflicting cable pull commands" << std::endl;}
        cable_pull_mag_change = m_cablePullSub->cable_pull_velocity_target*dt; // currently assuming instantaneous 'accel', TODO?
    }

    m_cable_pull_velocity = cable_pull_mag_change / dt;
    m_cable_pull_mag += cable_pull_mag_change;

    m_cablePullSub->publish_cablepull_measured_js(m_cable_pull_mag, m_cable_pull_velocity);
    auto last_seg_ptr = m_segmentBodyList.back();
    last_seg_ptr->applyTorque((1.0/dt)*0.001*m_cable_pull_mag*last_seg_ptr->getLocalRot().getCol2());

}

cTransform afVolmetricDrillingPlugin::btTransformTocTransform(const btTransform& in){    
    const auto& in_pos = in.getOrigin();
    const auto& in_rot = in.getRotation();

    cVector3d pos( in_pos.x(), in_pos.y(), in_pos.z() );
    const auto& axis = in_rot.getAxis();
    cVector3d c_axis( axis.getX(), axis.getY(), axis.getZ() );
    cMatrix3d rot(c_axis, in_rot.getAngle()); 
    cTransform out(pos, rot);
    return out;
}

void afVolmetricDrillingPlugin::UpdateCablePullText(){
    m_cablePullMagText->setText("Cable Pull Actual(Goal): " + cStr(m_cable_pull_mag,5) + "(" + cStr(m_cable_pull_mag_goal, 5) + ")");
}