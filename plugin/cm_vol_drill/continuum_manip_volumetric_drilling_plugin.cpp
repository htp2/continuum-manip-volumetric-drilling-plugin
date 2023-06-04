//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2023, AMBF
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

    \author    <henry.phalen@jhu.edu>
    \author    Henry Phalen

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar

    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam

*/
//==============================================================================

#include "continuum_manip_volumetric_drilling_plugin.h"
#include "cmvd_settings_rossub.h"
#include "sequential_impulse_solver.h"
#include "afConversions.h"

#include <boost/program_options.hpp>
#include <fstream>

//==============================================================================

/// @brief called at each graphics iteration of simulator loop
void afVolmetricDrillingPlugin::graphicsUpdate()
{
    UpdateCablePullText();
    // update region of voxels to be updated
    if (m_flagMarkVolumeForUpdate)
    {
        m_mutexVoxel.acquire();
        cVector3d min = m_volumeUpdate.m_min;
        cVector3d max = m_volumeUpdate.m_max;
        m_volumeUpdate.setEmpty();
        m_mutexVoxel.release();
        ((cTexture3d *)m_voxelObj->m_texture.get())->markForPartialUpdate(min, max);
        m_flagMarkVolumeForUpdate = false;
    }
}

/// @brief called at each physics iteration of simulator loop
/// @param dt time step (in seconds) since last call
void afVolmetricDrillingPlugin::physicsUpdate(double dt)
{
    checkForSettingsUpdate();

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);

    if (m_CM_moved_by_other) // i.e. if the CM is not being moved by keyboard / device, but is attached to another AMBF body
    {
        T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform(); // let's find out where it is now
    }
    else
    {                                                                                          // i.e. if the CM is being moved 'manually' using this plugin e.g. by keyboard / device
        if (!cTransformAlmostEqual(m_contManipBaseRigidBody->getLocalTransform(), T_contmanip_base)) // update CM for commanded movements
        {
            cTransform T_newContManipBase;
            T_newContManipBase.setLocalPos(T_contmanip_base.getLocalPos());
            T_newContManipBase.setLocalRot(T_contmanip_base.getLocalRot());
            T_newContManipBase = T_newContManipBase * to_cTransform(m_contManipBaseRigidBody->getInertialOffsetTransform()); // handle offset due to fact that origin is not at center of body
            m_contManipBaseRigidBody->setLocalTransform(T_newContManipBase);
        }
    }

    toolCursorsPosUpdate(T_contmanip_base);

    if (m_volume_collisions_enabled)
    {
        cToolCursor *burr_cursor = m_burrToolCursorList.back();

        // Drilling Behavior
        if (m_burrOn && burr_cursor->isInContact(m_voxelObj))
        {
            // find the color of the voxel that is in contact with the burr
            cCollisionEvent *contact = burr_cursor->m_hapticPoint->getCollisionEvent(0);
            cVector3d voxel_idx(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);
            m_voxelObj->m_texture->m_image->getVoxelColor(uint(voxel_idx.x()), uint(voxel_idx.y()), uint(voxel_idx.z()), m_storedColor);

            if (m_storedColor != m_zeroColor) // i.e. if the voxel is not empty
            {
                // removalCount specifies the max number of contacted voxels that can be removed in a single iteration
                // [TODO]: make this parameter more easily accessible
                int removalCount = cMin(m_removalCount, (int)contact->m_events.size());
                m_mutexVoxel.acquire();
                for (int cIdx = 0; cIdx < removalCount; cIdx++)
                {
                    cVector3d ct(contact->m_events[cIdx].m_voxelIndexX, contact->m_events[cIdx].m_voxelIndexY, contact->m_events[cIdx].m_voxelIndexZ);
                    if (m_hardness_behavior)
                    {
                        m_voxel_hardnesses[uint(ct.x())][uint(ct.y())][uint(ct.z())] -= m_hardness_removal_rate;
                        if (m_voxel_hardnesses[uint(ct.x())][uint(ct.y())][uint(ct.z())] > 0.0)
                        {
                            continue; // skips removal
                        }
                    }
                    removeVoxel(ct);
                }
                m_mutexVoxel.release();
                m_flagMarkVolumeForUpdate = true;
            }
        }

        // Compute and Apply CM-to-volume interactions
        for (auto &cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList})
        {
            for (auto &cursor : cursor_list)
            {
                cursor->computeInteractionForces();
            }
        }
        // Burr
        auto burr_impulse = calculate_impulse_from_tool_cursor_collision(m_burrToolCursorList[0], m_burrBody, dt);
        // m_burrBody->m_bulletRigidBody->applyCentralImpulse(burr_impulse);
        m_burrBody->m_bulletRigidBody->applyCentralImpulse(m_debug_scalar*burr_impulse);
    
        // Segments
        for (int i = 0; i < m_segmentToolCursorList.size(); i++)
        {
            auto seg_impulse = calculate_impulse_from_tool_cursor_collision(m_segmentToolCursorList[i], m_segmentBodyList[i], dt);
            m_segmentBodyList[i]->m_bulletRigidBody->applyCentralImpulse(m_debug_scalar*seg_impulse);
            // m_segmentBodyList[i]->m_bulletRigidBody->applyImpulse(seg_impulse, m_segmentBodyList[i]->getInertialOffsetTransform().getOrigin());
            // m_segmentBodyList[i]->m_bulletRigidBody->applyImpulse(seg_impulse, to_btVector(m_segmentToolCursorList[i]->getDeviceLocalPos()));
            // auto temp = m_segmentBodyList[i]->getInertialOffsetTransform().getOrigin();
            // auto temp2 =  to_btVector(m_segmentBodyList[i]->getLocalPos()) - to_btVector(m_segmentToolCursorList[i]->getDeviceLocalPos());
        }
        // Shaft
        for (int i = 0; i < m_shaftToolCursorList.size(); i++)
        {
            auto shaft_impulse = calculate_impulse_from_tool_cursor_collision(m_shaftToolCursorList[i], m_contManipBaseRigidBody, dt);
            m_contManipBaseRigidBody->m_bulletRigidBody->applyImpulse(shaft_impulse, to_btVector(m_shaftToolCursorList[i]->getDeviceLocalPos()));
        }
    }

    // Compute and Apply CM cable forces
    applyCablePull(dt);
}

/// @brief Remove a voxel from the volume
/// @param pos The position of the voxel to remove (in voxel coordinates)
/// @note This function is to be called from within a mutex lock
void afVolmetricDrillingPlugin::removeVoxel(cVector3d &pos)
{
    cColorb colorb;
    m_voxelObj->m_texture->m_image->getVoxelColor(uint(pos.x()), uint(pos.y()), uint(pos.z()), colorb);
    cColorf colorf = colorb.getColorf();

    double sim_time = m_worldPtr->getSimulationTime();
    double voxel_array[3] = {pos.get(0), pos.get(1), pos.get(2)};

    float color_array[4];
    color_array[0] = colorf.getR();
    color_array[1] = colorf.getG();
    color_array[2] = colorf.getB();
    color_array[3] = colorf.getA();
    m_voxelObj->m_texture->m_image->setVoxelColor(uint(pos.x()), uint(pos.y()), uint(pos.z()), m_zeroColor);

    m_volumeUpdate.enclose(cVector3d(uint(pos.x()), uint(pos.y()), uint(pos.z())));
    m_drillingPub->voxelsRemoved(voxel_array, color_array, sim_time);
}

/// @brief initialize the plugin
/// @param argc number of arguments
/// @param argv array of arguments
/// @param a_afWorld pointer to the ambf World
/// @return 0 if successful, -1 otherwise
/// @note This function is called by the afPluginManager
/// @note Commandline arguments are specified and parsed here
int afVolmetricDrillingPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld)
{
    ros::init(argc, argv, "drilling_simulator"); // primarily to strip out ros args

    // Specify command line options
    namespace p_opt = boost::program_options;
    p_opt::options_description cmd_opts("drilling_simulator Command Line Options");
    cmd_opts.add_options()("info", "Show Info");
    cmd_opts.add_options()("anatomy_volume_name", p_opt::value<std::string>()->default_value("cube"), "Name of volume given in yaml. Default spine_test_volume");
    cmd_opts.add_options()("base_body_name", p_opt::value<std::string>()->default_value("snake_stick"), "Name of body given in yaml. Default snake_stick");
    cmd_opts.add_options()("tool_body_name", p_opt::value<std::string>()->default_value("Burr"), "Name of body given in yaml. Default Burr");
    cmd_opts.add_options()("hardness_behavior", p_opt::value<std::string>()->default_value("0"), ". Turn on volume material hardness features. Default false");
    cmd_opts.add_options()("hardness_spec_file", p_opt::value<std::string>()->default_value(""), ". Path to csv file with hardness specifications per voxel. Default empty. If hardness features set, but this not set, all hardness will be set to 1.0");
    cmd_opts.add_options()("predrill_traj_file", p_opt::value<std::vector<std::string>>()->multitoken()->zero_tokens()->composing(), ". Path to csv file(s) with trajectory that will be predrilled. Default empty");
    cmd_opts.add_options()("predrill_ref_overwrite", p_opt::value<std::string>()->default_value(""), "By default drill traj is relative to [anatomy_volume_name]_anatomical origin. Here you can override to another object");

    // Parse command line options
    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);
    if (var_map.count("info"))
    {
        std::cout << cmd_opts << std::endl;
        return -1;
    }
    std::string anatomy_volume_name = var_map["anatomy_volume_name"].as<std::string>();
    std::string base_body_name = var_map["base_body_name"].as<std::string>();
    std::string tool_body_name = var_map["tool_body_name"].as<std::string>();
    std::string hardness_behavior = var_map["hardness_behavior"].as<std::string>();
    m_hardness_behavior = boost::lexical_cast<bool>(hardness_behavior);
    std::string hardness_spec_file = var_map["hardness_spec_file"].as<std::string>();
    m_hardness_spec_file = hardness_spec_file;
    std::vector<std::string> predrill_traj_files;
    if (var_map.count("predrill_traj_file"))
    {
        predrill_traj_files = var_map["predrill_traj_file"].as<std::vector<std::string>>();
    }
    std::string predrill_ref_overwrite = var_map["predrill_ref_overwrite"].as<std::string>();
    std::string predrill_ref_name = anatomy_volume_name + "_anatomical_origin";
    if (predrill_ref_overwrite != "")
    {
        predrill_ref_name = predrill_ref_overwrite;
    }

    // Bring in ambf world, make adjustments as needed to improve simulation accuracy
    m_worldPtr = a_afWorld;
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->setGravity(btVector3(0.0, 0.0, 0.0));

    // Various scalars needed for other calculation [TODO: some items might not need to be hardcoded here]
    m_to_ambf_unit = 1.0;
    mm_to_ambf_unit = m_to_ambf_unit / 1000.0;
    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);
    m_storedColor = cColorb(0x00, 0x00, 0x00, 0x00);

    // Importing continuum manipulator model
    m_contManipBaseRigidBody = m_worldPtr->getRigidBody(base_body_name);
    if (!m_contManipBaseRigidBody)
    {
        cerr << "ERROR! FAILED TO FIND RIGID BODY NAMED " << base_body_name << endl;
        return -1;
    }
    // Import anatomy volume
    m_volumeObject = m_worldPtr->getVolume(anatomy_volume_name);
    if (!m_volumeObject)
    {
        cerr << "ERROR! FAILED TO FIND VOLUME NAMED " << anatomy_volume_name << endl;
        return -1;
    }
    // Initialize the volume
    int res = volumeInit(a_afWorld);
    if (res != 0)
    {
        cerr << "ERROR! FAILED TO INITIALIZE VOLUME" << endl;
        return -1;
    }
    // Initializing tool cursors (for CM to interact with the volume)
    res = toolCursorInit(a_afWorld);
    if (res != 0)
    {
        cerr << "ERROR! FAILED TO INITIALIZE TOOL CURSORS" << endl;
        return -1;
    }
    // Initialize visualizations on camera
    res = visualInit(a_afWorld);
    if (res != 0)
    {
        cerr << "ERROR! FAILED TO INITIALIZE VISUALS" << endl;
        return -1;
    }
    // Get CM base initial pose
    T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform();

    // Set up voxels_removed publisher
    m_drillingPub = new DrillingPublisher("ambf", "volumetric_drilling");

    // Set up settings ros pub
    m_settingsPub = new CMVDSettingsSub("ambf", "volumetric_drilling");
    m_settingsPub->publish_anatomy_pose(m_volumeObject->getLocalTransform(), m_to_ambf_unit);

    // Volume Properties
    float dim[3];
    dim[0] = m_volumeObject->getDimensions().get(0);
    dim[1] = m_volumeObject->getDimensions().get(1);
    dim[2] = m_volumeObject->getDimensions().get(2);
    int voxelCount[3];
    voxelCount[0] = m_volumeObject->getVoxelCount().get(0);
    voxelCount[1] = m_volumeObject->getVoxelCount().get(1);
    voxelCount[2] = m_volumeObject->getVoxelCount().get(2);
    m_drillingPub->volumeProp(dim, voxelCount);

    if (m_hardness_behavior)
    {
        res = hardnessBehaviorInit(m_hardness_spec_file);
        if (res != 0)
        {
            cerr << "ERROR! FAILED TO INITIALIZE HARDNESS BEHAVIOR" << endl;
            return -1;
        }
    }

    if (!predrill_traj_files.empty())
        res = predrillTrajInit(predrill_traj_files, predrill_ref_name);
    if (res != 0)
    {
        cerr << "ERROR! FAILED TO INITIALIZE PREDRILL TRAJECTORY" << endl;
        return -1;
    }

    // Set up cable pull subscriber
    m_cablePullSub = new CablePullSubscriber("ambf", "volumetric_drilling");

    // Rand num gen
    std::random_device rd;
    rand_eng = std::mt19937(rd());
    unif_dist = std::uniform_real_distribution<>(0, 1);

    // Turn on volume collision
    m_volume_collisions_enabled = true;

    return 1;
}

/// @brief Initialize the text overlays on the camera
/// @param a_afWorld ambf world pointer
/// @return 0 if successful, -1 otherwise
int afVolmetricDrillingPlugin::visualInit(const afWorldPtr a_afWorld)
{
    // Get first camera
    m_mainCamera = m_worldPtr->getCameras()[0];

    // Create a label to display the cable pull magnitude
    cFontPtr font = NEW_CFONTCALIBRI40();
    m_cablePullMagText = new cLabel(font);
    m_cablePullMagText->setLocalPos(20, 70);
    m_cablePullMagText->m_fontColor.setBlack();
    m_cablePullMagText->setFontScale(.5);
    UpdateCablePullText();
    m_mainCamera->getFrontLayer()->addChild(m_cablePullMagText);

    // Create a label to display drill and cable control modes
    m_drillControlModeText = new cLabel(font);
    m_drillControlModeText->setLocalPos(20, 30);
    m_drillControlModeText->m_fontColor.setGreen();
    m_drillControlModeText->setFontScale(.5);
    m_drillControlModeText->setText("Drill Control Mode = Haptic Device / Keyboard");
    m_mainCamera->getFrontLayer()->addChild(m_drillControlModeText);

    m_cableControlModeText = new cLabel(font);
    m_cableControlModeText->setLocalPos(20, 10);
    m_cableControlModeText->m_fontColor.setGreen();
    m_cableControlModeText->setFontScale(.5);
    m_cableControlModeText->setText("Cable Control Mode = Keyboard");
    m_mainCamera->getFrontLayer()->addChild(m_cableControlModeText);
    return 0;
}

/// @brief Initialize the volume model and its properties
/// @param a_afWorld ambf world pointer
/// @return 0 if successful, -1 otherwise
int afVolmetricDrillingPlugin::volumeInit(const afWorldPtr a_afWorld)
{
    m_voxelObj = m_volumeObject->getInternalVolume();

    // Get the volume dimensions and scale
    m_maxVolCorner = m_voxelObj->m_maxCorner;
    m_minVolCorner = m_voxelObj->m_minCorner;
    m_maxTexCoord = m_voxelObj->m_maxTextureCoord;
    m_minTexCoord = m_voxelObj->m_minTextureCoord;
    m_textureCoordScale(0) = (m_maxTexCoord.x() - m_minTexCoord.x()) / (m_maxVolCorner.x() - m_minVolCorner.x());
    m_textureCoordScale(1) = (m_maxTexCoord.y() - m_minTexCoord.y()) / (m_maxVolCorner.y() - m_minVolCorner.y());
    m_textureCoordScale(2) = (m_maxTexCoord.z() - m_minTexCoord.z()) / (m_maxVolCorner.z() - m_minVolCorner.z());

    // Set voxels surface contact properties
    double maxStiffness = 0.005; // This appeared to be the default so since I'm not using a haptic device at the moment I'm hardcoding it here
    m_voxelObj->m_material->setStiffness(2.0 * maxStiffness);
    m_voxelObj->m_material->setDamping(0.0);
    m_voxelObj->m_material->setDynamicFriction(0.0);
    m_voxelObj->setUseMaterial(true);
    return 0;
}

///
/// \brief This method creates a tool cursor for each segment of the CM, for the burr, and along the shaft. The tool cursor is used to produce iteraction forces with volume
/// \param a_afWorld    A world that contains all objects of the virtual environment
/// \return 0 if successful, -1 otherwise
///
int afVolmetricDrillingPlugin::toolCursorInit(const afWorldPtr a_afWorld)
{
    cWorld *chai_world = a_afWorld->getChaiWorld();
    // [TODO]: Someday, this could be generalized to more CM models
    int num_segs = 27;
    int num_shaft_cursor = 6;
    int num_burr_cursor = 1;
    double burr_r = mm_to_ambf_unit * 6.5 / 2;
    double cm_r = mm_to_ambf_unit * 6.0 / 2;

    for (int i = 1; i <= num_segs; i++)
    {
        m_segmentBodyList.push_back(m_worldPtr->getRigidBody("/ambf/env/BODY seg" + to_string(i)));
        auto seg_cursor = new cToolCursor(chai_world);
        m_segmentToolCursorList.push_back(seg_cursor);
    }
    for (int i = 0; i < num_shaft_cursor; i++)
    {
        auto shaft_cursor = new cToolCursor(chai_world);
        m_shaftToolCursorList.push_back(shaft_cursor);
    }
    for (int i = 0; i < num_burr_cursor; i++)
    {
        auto burr_cursor = new cToolCursor(chai_world);
        m_burrToolCursorList.push_back(burr_cursor);
        m_burrBody = m_worldPtr->getRigidBody("/ambf/env/BODY Burr");
    }

    for (auto &shaft_cursor : m_shaftToolCursorList)
    {
        shaft_cursor->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
        shaft_cursor->m_hapticPoint->m_sphereProxy->m_material->setRedCrimson();
        shaft_cursor->m_hapticPoint->m_sphereGoal->m_material->setBlueAquamarine();
        shaft_cursor->setRadius(cm_r);
    }

    for (auto &burr_cursor : m_burrToolCursorList)
    {
        burr_cursor->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
        burr_cursor->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
        burr_cursor->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
        burr_cursor->setRadius(burr_r);
    }

    for (auto &seg_cursor : m_segmentToolCursorList)
    {
        seg_cursor->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
        seg_cursor->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
        seg_cursor->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
        seg_cursor->setRadius(cm_r);
    }
    // Initialize the start pose of the tool cursors
    toolCursorsPosUpdate(m_contManipBaseRigidBody->getLocalTransform());
    for (auto &cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList})
    {
        for (auto &cursor : cursor_list)
        {
            cursor->initialize();
            m_worldPtr->addSceneObjectToWorld(cursor);
        }
    }
    return 0;
}

///
/// \brief Applies small change to the position of the CM base (prmiarily for use with the keyboard commands)
/// \param a_delta Change in position
///
void afVolmetricDrillingPlugin::incrementDevicePos(cVector3d a_delta)
{
    T_contmanip_base.setLocalPos(T_contmanip_base.getLocalPos() + a_delta);
}

///
/// \brief Applies small change to the orientation of the CM base (prmiarily for use with the keyboard commands)
/// \param a_rot Change in orientation (Euler angles XYZ)
///
void afVolmetricDrillingPlugin::incrementDeviceRot(cVector3d a_rot)
{
    cMatrix3d R_cmd;
    R_cmd.setExtrinsicEulerRotationDeg(a_rot(0), a_rot(1), a_rot(2), C_EULER_ORDER_XYZ);
    R_cmd = T_contmanip_base.getLocalRot() * R_cmd;
    T_contmanip_base.setLocalRot(R_cmd);
}

///
/// \brief Update the position of the tool cursors 'goal' spheres, to match the location of bodies in the simulation after last physics step
/// \param a_targetPose The pose of the CM base
///
void afVolmetricDrillingPlugin::toolCursorsPosUpdate(cTransform a_basePose)
{
    double shaft_length = mm_to_ambf_unit * 35.0; // [TODO]: This could be made parameter to work with more general CM
    for (int i = 0; i < m_shaftToolCursorList.size(); i++)
    {
        double offset = static_cast<double>(i) / (m_shaftToolCursorList.size() - 1) * shaft_length;
        auto T_offset = cTransform(cVector3d(0, offset, 0), cMatrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
        m_shaftToolCursorList[i]->setDeviceLocalTransform(a_basePose * T_offset);
    }

    for (auto &burr_cursor : m_burrToolCursorList)
    {
        burr_cursor->setDeviceLocalTransform(m_burrBody->getLocalTransform());
    }

    for (int i = 0; i < m_segmentToolCursorList.size(); i++)
    {
        m_segmentToolCursorList[i]->setDeviceLocalTransform(m_segmentBodyList[i]->getLocalTransform());
    }
}

///
/// \brief Use keyboard commands to perform a variety of tasks
/// \note This function is called by the plugin manager
///
void afVolmetricDrillingPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    if (a_mods == GLFW_MOD_CONTROL) // (CTRL + key)
    {

        // These commands are used to move the CM base position
        if (a_key == GLFW_KEY_W)
        {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol2() * m_drillRate; // z-direction
            incrementDevicePos(dir);
        }
        else if (a_key == GLFW_KEY_D)
        {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol0() * m_drillRate; // x-direction
            incrementDevicePos(dir);
        }
        else if (a_key == GLFW_KEY_S)
        {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol2() * m_drillRate; // z-direction
            incrementDevicePos(-dir);
        }
        else if (a_key == GLFW_KEY_A)
        {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol0() * m_drillRate; // x-direction
            incrementDevicePos(-dir);
        }
        else if (a_key == GLFW_KEY_K)
        {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol1() * m_drillRate; // y-direction
            incrementDevicePos(-dir);
        }
        else if (a_key == GLFW_KEY_I)
        {
            cVector3d dir = m_contManipBaseRigidBody->getLocalRot().getCol1() * m_drillRate; // y-direction
            incrementDevicePos(dir);
        }

        // Cable controls
        else if (a_key == GLFW_KEY_SEMICOLON)
        {
            if (m_cableKeyboardControl)
            {
                m_cable_pull_mag_goal += 0.001;
            }
        }
        else if (a_key == GLFW_KEY_APOSTROPHE)
        {
            if (m_cableKeyboardControl)
            {
                m_cable_pull_mag_goal -= 0.001;
            }
        }

        // Toggle control modes
        else if (a_key == GLFW_KEY_O)
        {
            setDrillControlMode(!m_CM_moved_by_other);
        }
        else if (a_key == GLFW_KEY_SLASH)
        {
            setCableControlMode(!m_cableKeyboardControl);
        }
        else if (a_key == GLFW_KEY_LEFT_BRACKET)
        {
            setVolumeCollisionsEnabled(!m_volume_collisions_enabled);
        }
        else if (a_key == GLFW_KEY_RIGHT_BRACKET)
        {
            // toolCursorInit(m_worldPtr); //[TODO]: No longer needed, but keeping comment for now in case I want to add back in
        }
        else if (a_key == GLFW_KEY_EQUAL)
        {
            m_burrOn = !m_burrOn;
            std::cout << "Burr On State: " << m_burrOn << std::endl;
        }
        else if (a_key == GLFW_KEY_E)
        {
            setPhysicsPaused(!m_worldPtr->isPhysicsPaused());
        }

        // Visual controls
        else if (a_key == GLFW_KEY_C)
        {
            setShowToolCursors(!m_showGoalProxySpheres);
        }

        else if (a_key == GLFW_KEY_N)
        {
            cout << "INFO! RESETTING THE VOLUME" << endl;
            m_volumeObject->reset();
            if(m_hardness_behavior)
            {
                hardnessBehaviorInit(m_hardness_spec_file);
            }
            
        }
    }

    else if (a_mods == GLFW_MOD_ALT) // (ALT + key)
    {
        if (a_key == GLFW_KEY_UP)
        {
            m_removalCount++;
            cout << "INFO! REMOVAL COUNT: " << m_removalCount << endl;
        }
        if (a_key == GLFW_KEY_DOWN)
        {
            m_removalCount--;
            cout << "INFO! REMOVAL COUNT: " << m_removalCount << endl;
        }
    }
    else
    {

        // option - reduce size along X axis
        if (a_key == GLFW_KEY_4)
        {
            sliceVolume(0, -0.005);
        }

        // option - increase size along X axis
        else if (a_key == GLFW_KEY_5)
        {
            sliceVolume(0, 0.005);
        }

        // option - reduce size along Y axis
        else if (a_key == GLFW_KEY_6)
        {
            sliceVolume(1, -0.005);
        }

        // option - increase size along Y axis
        else if (a_key == GLFW_KEY_7)
        {
            sliceVolume(1, 0.005);
        }

        // option - reduce size along Z axis
        else if (a_key == GLFW_KEY_8)
        {
            sliceVolume(2, -0.005);
        }

        // option - increase size along Z axis
        else if (a_key == GLFW_KEY_9)
        {
            sliceVolume(2, 0.005);
        }
        // option - decrease quality of graphic rendering
        else if (a_key == GLFW_KEY_L)
        {
            double value = m_voxelObj->getQuality();
            m_voxelObj->setQuality(value - 0.01);
            cout << "> Quality set to " << cStr(m_voxelObj->getQuality(), 1) << "                            \r";
        }

        // option - increase quality of graphic rendering
        else if (a_key == GLFW_KEY_U)
        {
            double value = m_voxelObj->getQuality();
            m_voxelObj->setQuality(value + 0.01);
            cout << "> Quality set to " << cStr(m_voxelObj->getQuality(), 1) << "                            \r";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_UP)
        {
            double value = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(value + 0.01);
            cout << "> Opacity Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_DOWN)
        {
            double value = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(value - 0.01);
            cout << "> Opacity Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_RIGHT)
        {
            double value = m_voxelObj->getIsosurfaceValue();
            m_voxelObj->setIsosurfaceValue(value + 0.01);
            cout << "> Isosurface Threshold set to " << cStr(m_voxelObj->getIsosurfaceValue(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_LEFT)
        {
            double value = m_voxelObj->getIsosurfaceValue();
            m_voxelObj->setIsosurfaceValue(value - 0.01);
            cout << "> Isosurface Threshold set to " << cStr(m_voxelObj->getIsosurfaceValue(), 1)
                 << "                            \n";
        }

        // option - toggle vertical mirroring
        else if (a_key == GLFW_KEY_ENTER)
        {
            m_renderingMode++;
            if (m_renderingMode > 7)
            {
                m_renderingMode = 0;
            }
            switch (m_renderingMode)
            {
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
        }
        else if (a_key == GLFW_KEY_PAGE_UP)
        {
            m_opticalDensity += 0.1;
            m_voxelObj->setOpticalDensity(m_opticalDensity);
            cout << "> Optical Density set to " << cStr(m_opticalDensity, 1) << "                            \n";
        }
        else if (a_key == GLFW_KEY_PAGE_DOWN)
        {
            m_opticalDensity -= 0.1;
            m_voxelObj->setOpticalDensity(m_opticalDensity);
            cout << "> Optical Density set to " << cStr(m_opticalDensity, 1) << "                            \n";
        }
        else if (a_key == GLFW_KEY_HOME)
        {
            float val = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(val + 0.1);
            cout << "> Optical Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }
        else if (a_key == GLFW_KEY_END)
        {
            float val = m_voxelObj->getOpacityThreshold();
            m_voxelObj->setOpacityThreshold(val - 0.1);
            cout << "> Optical Threshold set to " << cStr(m_voxelObj->getOpacityThreshold(), 1)
                 << "                            \n";
        }

        // controls rotational motion of CM base
        else if (a_key == GLFW_KEY_KP_5)
        {
            cVector3d rotDir(0, 1, 0);
            incrementDeviceRot(rotDir);
        }

        else if (a_key == GLFW_KEY_KP_8)
        {

            cVector3d rotDir(0, -1, 0);
            incrementDeviceRot(rotDir);
        }

        else if (a_key == GLFW_KEY_KP_4)
        {

            cVector3d rotDir(0, 0, -1);
            incrementDeviceRot(rotDir);
        }

        else if (a_key == GLFW_KEY_KP_6)
        {

            cVector3d rotDir(0, 0, 1);
            incrementDeviceRot(rotDir);
        }
        else if (a_key == GLFW_KEY_KP_DECIMAL)
        {
            m_debug_print = !m_debug_print;
            std::cout << "m_debug_print: " << m_debug_print << std::endl;
        }
        else if (a_key == GLFW_KEY_KP_ADD)
        {
            m_debug_scalar += 0.1;
            std::cout << "m_debug_scalar: " << m_debug_scalar << std::endl;
        }
        else if (a_key == GLFW_KEY_KP_SUBTRACT)
        {
            m_debug_scalar -= 0.1;
            std::cout << "m_debug_scalar: " << m_debug_scalar << std::endl;
        }        

        // toggles the visibility of CM mesh (segments, shaft, and burr)
        else if (a_key == GLFW_KEY_B)
        {
            m_showCM = !m_showCM;
            m_contManipBaseRigidBody->m_visualMesh->setShowEnabled(m_showCM);
            for (auto &seg : m_segmentBodyList)
            {
                seg->m_visualMesh->setShowEnabled(m_showCM);
            }
            m_burrBody->m_visualMesh->setShowEnabled(m_showCM);
        }
    }
}

void afVolmetricDrillingPlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes)
{
}

void afVolmetricDrillingPlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos)
{
}

void afVolmetricDrillingPlugin::reset()
{
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
    T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform();
}

bool afVolmetricDrillingPlugin::close()
{
    for (auto &cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList})
    {
        for (auto &cursor : cursor_list)
        {
            cursor->stop();
        }
    }
    return true;
}

/// @brief  Apply cable pull to the CM distal segment
/// @param dt time step (seconds)
void afVolmetricDrillingPlugin::applyCablePull(double dt)
{
    // Set position goal if applicable
    double cable_pull_mag_change;
    double max_mag_change = 0.001;

    if (m_cableKeyboardControl || m_cablePullSub->command_type == cable_pull_command_type::POSITION)
    {
        if (m_cableKeyboardControl)
        {
        } // m_cable_pull_mag_goal already set by keyboard commands
        else
        {
            if (m_cablePullSub->command_type != cable_pull_command_type::POSITION) // sanity check
            {
                std::cerr << "conflicting cable pull commands" << std::endl;
            }
            m_cable_pull_mag_goal = m_cablePullSub->cable_pull_position_target;
        }
        double cable_pull_mag_err = (m_cable_pull_mag_goal - m_cable_pull_mag);
        cable_pull_mag_change = cable_pull_mag_err;
        if (abs(cable_pull_mag_change) > max_mag_change)
        {
            cable_pull_mag_change = cable_pull_mag_change / abs(cable_pull_mag_change) * max_mag_change;
        }
    }
    else
    {
        if (m_cablePullSub->command_type != cable_pull_command_type::VELOCITY) // sanity check
        {
            std::cerr << "conflicting cable pull commands" << std::endl;
        }
        cable_pull_mag_change = m_cablePullSub->cable_pull_velocity_target * dt; // currently assuming instantaneous 'accel', TODO?
    }

    m_cable_pull_velocity = cable_pull_mag_change / dt;
    m_cable_pull_mag += cable_pull_mag_change;
    double cable_pull_force_val = m_cable_pull_mag;

    m_cablePullSub->publish_cablepull_measured_js(m_cable_pull_mag, m_cable_pull_velocity);
    auto last_seg_ptr = m_segmentBodyList.back();
    auto torque_imp = 10.0 * cable_pull_force_val * last_seg_ptr->getLocalRot().getCol2() * dt;
    last_seg_ptr->m_bulletRigidBody->applyTorqueImpulse(btVector3(torque_imp.x(), torque_imp.y(), torque_imp.z()));
}

void afVolmetricDrillingPlugin::UpdateCablePullText()
{
    m_cablePullMagText->setText("Cable Pull Actual(Goal): " + cStr(m_cable_pull_mag, 5) + "(" + cStr(m_cable_pull_mag_goal, 5) + ")");
}

void afVolmetricDrillingPlugin::sliceVolume(int axisIdx, double delta)
{
    string axis_str = "";
    if (axisIdx == 0)
    {
        axis_str = "X";
    }
    else if (axisIdx == 1)
    {
        axis_str = "Y";
    }
    else if (axisIdx == 2)
    {
        axis_str = "Z";
    }
    else
    {
        cerr << "ERROR! Volume axis index should be either 0, 1 or 2" << endl;
        return;
    }

    string delta_dir_str = "";
    if (delta > 0)
    {
        delta_dir_str = "Increase";
    }
    else
    {
        delta_dir_str = "Decrease";
    }

    double value = cClamp((m_voxelObj->m_maxCorner(axisIdx) + delta), 0.01, m_maxVolCorner(axisIdx));
    m_voxelObj->m_maxCorner(axisIdx) = value;
    m_voxelObj->m_minCorner(axisIdx) = -value;
    m_voxelObj->m_maxTextureCoord(axisIdx) = 0.5 + value * m_textureCoordScale(axisIdx);
    m_voxelObj->m_minTextureCoord(axisIdx) = 0.5 - value * m_textureCoordScale(axisIdx);

    cerr << "> " << delta_dir_str << " Volume size along " << axis_str << " axis.                            \r";
}

void afVolmetricDrillingPlugin::setShowToolCursors(bool bool_show)
{
    m_showGoalProxySpheres = bool_show;
    for (auto &cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList})
    {
        for (auto &cursor : cursor_list)
        {
            cursor->m_hapticPoint->setShow(m_showGoalProxySpheres, m_showGoalProxySpheres);
        }
    }
}

void afVolmetricDrillingPlugin::setDrillControlMode(bool bool_set)
{
    m_CM_moved_by_other = bool_set;
    if (m_CM_moved_by_other)
    {
        m_drillControlModeText->m_fontColor.setRed();
        m_drillControlModeText->setText("Drill Control Mode = External afComm");
    }
    else
    {
        m_drillControlModeText->m_fontColor.setGreen();
        m_drillControlModeText->setText("Drill Control Mode = Haptic Device / Keyboard");
    }
}

void afVolmetricDrillingPlugin::setCableControlMode(bool bool_set)
{
    m_cableKeyboardControl = bool_set;
    std::string cable_control_mode = m_cableKeyboardControl ? "Keyboard" : "Subscriber";
    m_cableKeyboardControl ? m_cableControlModeText->m_fontColor.setGreen() : m_cableControlModeText->m_fontColor.setRed();
    m_cableControlModeText->setText("Cable Control Mode = " + cable_control_mode);
}

void afVolmetricDrillingPlugin::setPhysicsPaused(bool bool_set)
{
    bool paused = m_worldPtr->isPhysicsPaused();
    if (paused == bool_set)
    {
        std::cout << "Attempted to set physics to paused: " << bool_set << " but already at that setting" << std::endl;
        return;
    }

    if (bool_set)
    {
        m_worldPtr->pausePhysics(false);
    }
    else
    {
        auto last_seg_ptr = m_segmentBodyList.back();
        last_seg_ptr->applyTorque(cVector3d(0.0, 0.0, 0.0));
        m_worldPtr->pausePhysics(true);
    }
    std::cout << "Toggled plugin physics paused to: " << m_worldPtr->isPhysicsPaused() << std::endl;
}

void afVolmetricDrillingPlugin::setVolumeCollisionsEnabled(bool bool_set)
{
    m_volume_collisions_enabled = bool_set;
}

/// @brief  Parse a CSV file containing a trajectory for the predrill tool and burr size
/// @param filename  CSV file containing: first line: burr size: double;; subsequent lines: trajectory (x,y,z) points: double,double,double (no header
/// @param traj_points outparam for trajectory points x,y,z
/// @param burr_size outparam for burr size (m)
/// @return 
bool parsePredrillTrajFromCSV(const std::string &filename, std::vector<cVector3d> &traj_points, double &burr_size)
{
    std::ifstream file(filename);
    if (!file)
    {
        std::cout << "[parsePredrillTrajFromCSV] No such file: " << filename << std::endl;
        return false;
    }
    
    // Read the burr size (first line)
    std::string s;
    if (!std::getline(file, s))
    {
        std::cout << "[parsePredrillTrajFromCSV] Empty file: " << filename << std::endl;
        return false;
    }
    std::istringstream ss(s);
    std::vector<std::string> record;
    while (ss)
    {
        std::string s;
        if (!getline(ss, s))
            break;
        record.push_back(s);
    }
    burr_size = std::stod(record[0]);
    
    // Read the trajectory points (subsequent lines)
    while (file)
    {
        std::string s;
        if (!std::getline(file, s))
            break;
        std::istringstream ss(s);
        std::vector<std::string> record;
        while (ss)
        {
            std::string s;
            if (!getline(ss, s, ','))
                break;
            record.push_back(s);
        }
        traj_points.push_back(cVector3d(std::stod(record[0]), std::stod(record[1]), std::stod(record[2])));
    }
    return true;
}

/// @brief  Checks for changes to the settings exposed to rostopics
/// @TODO:  This implementation could be improved, it is not very scalable
void afVolmetricDrillingPlugin::checkForSettingsUpdate(void)
{
    if (m_settingsPub->setCableControlMode_changed)
    {
        setCableControlMode(m_settingsPub->setCableControlMode_last_val);
        m_settingsPub->setCableControlMode_changed = false;
    }
    if (m_settingsPub->setDrillControlMode_changed)
    {
        setDrillControlMode(m_settingsPub->setDrillControlMode_last_val);
        m_settingsPub->setDrillControlMode_changed = false;
    }
    if (m_settingsPub->setPhysicsPaused_changed)
    {
        setPhysicsPaused(m_settingsPub->setPhysicsPaused_last_val);
        m_settingsPub->setPhysicsPaused_changed = false;
    }
    if (m_settingsPub->setShowToolCursors_changed)
    {
        setShowToolCursors(m_settingsPub->setShowToolCursors_last_val);
        m_settingsPub->setShowToolCursors_changed = false;
    }
    if (m_settingsPub->setVolumeCollisionsEnabled_changed)
    {
        setVolumeCollisionsEnabled(m_settingsPub->setVolumeCollisionsEnabled_last_val);
        m_settingsPub->setVolumeCollisionsEnabled_changed = false;
    }
    if (m_settingsPub->initToolCursors_changed)
    {
        // toolCursorInit(m_worldPtr);
        m_settingsPub->initToolCursors_changed = false;
    }
    if (m_settingsPub->resetVoxels_changed)
    {
        m_volumeObject->reset();
        m_settingsPub->resetVoxels_changed = false;
    }
    if (m_settingsPub->setBurrOn_changed)
    {
        m_burrOn = m_settingsPub->setBurrOn_last_val;
        m_settingsPub->setBurrOn_changed = false;
    }
}

/// @brief  Initialize behavior for voxel hardnesses
/// @param hardness_spec_file  CSV file containing hardness values for each voxel
/// @return 0 if successful, -1 if not
int afVolmetricDrillingPlugin::hardnessBehaviorInit(const std::string &hardness_spec_file)
{
    if (m_hardness_behavior)
    {
        std::vector<std::vector<std::vector<double>>> forces(m_volumeObject->getVoxelCount().get(0), std::vector<std::vector<double>>(m_volumeObject->getVoxelCount().get(1), std::vector<double>(m_volumeObject->getVoxelCount().get(2), 1.0)));

        if (hardness_spec_file == "")
        {
            std::cout << "[hardnessBehaviorInit]: No hardness spec file given, using default of 1.0" << std::endl;
            m_voxel_hardnesses = forces;
            return 0;
        }

        std::ifstream file(hardness_spec_file);
        if (!file)
        {
            std::cout << "[hardnessBehaviorInit]: No such file: " << hardness_spec_file << std::endl;
            return -1;
        }

        // read first line
        std::string s;
        if (!std::getline(file, s))
        {
            std::cout << "[hardnessBehaviorInit]: File appears to be empty: " << hardness_spec_file << std::endl;
            return -1;
        }
        std::istringstream ss(s);
        std::vector<std::string> record;
        while (ss)
        {
            std::string s;
            if (!getline(ss, s, ','))
                break;
            record.push_back(s);
        }
        // see if first line is m_volumeObject->getVoxelCount().get(0), m_volumeObject->getVoxelCount().get(1), m_volumeObject->getVoxelCount().get(2)
        if (record.size() != 3)
        {
            std::cout << "[hardnessBehaviorInit]: expected hardness spec dimensionality of 3 in " << hardness_spec_file << std::endl;
            return -1;
        }
        if (std::stoi(record[0]) != m_volumeObject->getVoxelCount().get(0) || std::stoi(record[1]) != m_volumeObject->getVoxelCount().get(1) || std::stoi(record[2]) != m_volumeObject->getVoxelCount().get(2))
        {
            std::cout << "[hardnessBehaviorInit]: expected hardness spec size of " << m_volumeObject->getVoxelCount().get(0) << ", " << m_volumeObject->getVoxelCount().get(1) << ", " << m_volumeObject->getVoxelCount().get(2) << " in " << hardness_spec_file << std::endl;
            std::cout << "[hardnessBehaviorInit]: got " << record[0] << ", " << record[1] << ", " << record[2] << std::endl;
            return -1;
        }

        // rest of the file is one number per line which is the hardness, loop through each dimension of harndesses
        for (size_t i = 0; i < m_volumeObject->getVoxelCount().get(0); i++)
        {
            for (size_t j = 0; j < m_volumeObject->getVoxelCount().get(1); j++)
            {
                for (size_t k = 0; k < m_volumeObject->getVoxelCount().get(2); k++)
                {
                    if (!std::getline(file, s))
                    {
                        std::cout << "[hardnessBehaviorInit]: No data at i,j,k = " << i << "," << j << "," << k << ": " << hardness_spec_file << std::endl;
                        return -1;
                    }
                    forces[i][j][k] = std::stod(s) + 0.5;
                }
            }
        }
        file.close();
        m_voxel_hardnesses = forces;
    }
    return 0;
}

/// @brief Remove voxels in the cylinder around specified trajectory(ies) as if they were pre-drilled
int afVolmetricDrillingPlugin::predrillTrajInit(const std::vector<std::string> &predrill_traj_files, const std::string &predrill_reference_object_name)
{
    m_predrill_reference_object = m_worldPtr->getBaseObject(predrill_reference_object_name, false);
    if (!m_predrill_reference_object)
    {
        cerr << "ERROR! FAILED TO FIND PREDRILL REFERENCE OBJECT " << predrill_reference_object_name << endl;
        return -1;
    }
    auto predrill_offset = m_predrill_reference_object->getLocalTransform();
    auto volume_origin_offset = m_volumeObject->getLocalTransform();
    volume_origin_offset.invert();
    // TODO there is likely a much more direct way of doing this - remove all voxels in the cylinder / "pill" around the trajectory
    int resol = 20;
    m_mutexVoxel.acquire();
    for (auto &predrill_traj_file : predrill_traj_files)
    {
        double burr_size;
        std::vector<cVector3d> traj_points;
        if (!parsePredrillTrajFromCSV(predrill_traj_file, traj_points, burr_size))
        {
            std::cout << "[predrillTrajInit]: Error reading: " << predrill_traj_file << std::endl;
            return -1;
        }
        burr_size *= mm_to_ambf_unit;

        for (auto &pt : traj_points)
        {
            pt = volume_origin_offset * predrill_offset * pt;
            for (size_t i = 0; i <= resol; i++)
            {
                for (size_t j = 0; j <= resol; j++)
                {
                    for (size_t k = 0; k <= resol; k++)
                    {
                        auto denom = static_cast<double>(resol);
                        double r = i / denom * burr_size / 2.0;
                        double th = j / denom * 2 * PI;
                        double phi = k / denom * PI;
                        cVector3d offset(r * sin(phi) * cos(th), r * sin(phi) * sin(th), r * cos(phi));

                        cVector3d idx;
                        auto new_pt = pt + offset;
                        // std::cout << "actual point " << new_pt << std::endl;
                        m_volumeObject->localPosToVoxelIndex(new_pt, idx);
                        removeVoxel(idx);
                    }
                }
            }
        }
    }
    m_mutexVoxel.release();
    graphicsUpdate();
    return 0;
}

/// @brief Determine if two transforms are approx equal
bool afVolmetricDrillingPlugin::cTransformAlmostEqual(const cTransform &a, const cTransform &b)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (std::abs(a.m[i][j] - b.m[i][j]) > 0.0001)
            {
                return false;
            }
        }
    }
    return true;
}

/// @brief Uses seqential impulse contact method to calculate the impulse from a tool cursor collision with the volume object
btVector3 afVolmetricDrillingPlugin::calculate_impulse_from_tool_cursor_collision(cToolCursor *tool_cursor, afRigidBodyPtr &body, double dt)
{
    bool use_legacy_method = false;
    
    if (use_legacy_method)
    {
        // ouput impulse from collision (default zero)
        btVector3 imp_out(0.0, 0.0, 0.0);

        // voxel collision properties
        double m2 = 0.;
        auto dim = m_volumeObject->getDimensions();
        auto num_voxels = m_volumeObject->getVoxelCount();
        // get max of dim[i]/num_voxels[i] for i = 0,1,2
        double r2 = 0.0;
        for (size_t i = 0; i < 3; i++)
        {
            r2 = cMax(r2, dim(i) / num_voxels(i));
        }

        cCollisionEvent *contact = tool_cursor->m_hapticPoint->getCollisionEvent(0);
        cVector3d voxel_idx(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);
        m_voxelObj->m_texture->m_image->getVoxelColor(uint(voxel_idx.x()), uint(voxel_idx.y()), uint(voxel_idx.z()), m_storedColor);
        bool in_collision = m_storedColor != m_zeroColor;
        if (in_collision)
        {
            double m1 = body->getMass();
            auto r1 = tool_cursor->m_hapticPoint->getRadiusContact();
            auto cx1 = tool_cursor->m_hapticPoint->getGlobalPosGoal();
            Eigen::Vector3d x1;
            x1 << cx1.x(), cx1.y(), cx1.z();

            auto cx2 = tool_cursor->m_hapticPoint->getGlobalPosProxy();
            Eigen::Vector3d x2;
            x2 << cx2.x(), cx2.y(), cx2.z();

            Eigen::Vector3d n_plane;
            n_plane << contact->m_globalNormal.x(), contact->m_globalNormal.y(), contact->m_globalNormal.z();
            // n_plane *= -1.0; // switch normal direction to point from contact point towards proxy point (i.e. outward)
            
            x2 = x2+(x1-x2).normalized() * r1;

            // cVector3d cx2;
            // m_volumeObject->voxelIndexToLocalPos(voxel_idx, cx2);
            // cx2 = m_volumeObject->getLocalTransform() * cx2;
            // x2 << cx2.x(), cx2.y(), cx2.z();

            Eigen::Matrix<double, 12, 1> V;
            V.setZero();
            V(0) = body->m_bulletRigidBody->getLinearVelocity().x();
            V(1) = body->m_bulletRigidBody->getLinearVelocity().y();
            V(2) = body->m_bulletRigidBody->getLinearVelocity().z();
            V(3) = body->m_bulletRigidBody->getAngularVelocity().x();
            V(4) = body->m_bulletRigidBody->getAngularVelocity().y();
            V(5) = body->m_bulletRigidBody->getAngularVelocity().z();        

            Eigen::Matrix<double, 12, 1> F_ext;
            F_ext.setZero();
            F_ext(0) = body->m_bulletRigidBody->getTotalForce().x();
            F_ext(1) = body->m_bulletRigidBody->getTotalForce().y();
            F_ext(2) = body->m_bulletRigidBody->getTotalForce().z();
            F_ext(3) = body->m_bulletRigidBody->getTotalTorque().x();
            F_ext(4) = body->m_bulletRigidBody->getTotalTorque().y();
            F_ext(5) = body->m_bulletRigidBody->getTotalTorque().z();

            Eigen::Matrix<double, 12, 1> P;
            P.setZero();

            // Eigen::Vector3d n_plane;
            // n_plane << contact->m_globalNormal.x(), contact->m_globalNormal.y(), contact->m_globalNormal.z();
            // bool in_contact = compute_impulse_two_sphere_collision(P, x1, x2, r1, r2, m1, m2, dt, V, F_ext, 0.4);
            
            // x2 = x2 + r2*n_plane;


            bool in_contact = compute_impulse_plane_sphere_collision(P, x1, x2, r1, n_plane, m1, m2, dt, V, F_ext, 0.4);

            if (in_contact)
            {
                imp_out = btVector3(P(0), P(1), P(2));
                
                // imp_out *= m_debug_scalar;
                // std::cout << "impulse: " << imp_out.x() << imp_out.y() << imp_out.z() << std::endl;
            }
            else
            {
                // std::cout << "SI says no contact, but tool cursor says contact" << std::endl;
            }
        }

        return imp_out;
    }

    else
    {
        btVector3 imp_out(0.0, 0.0, 0.0);

        double m2 = 0.; // fixed body
        double b = 0.4;
        double a = 1.0;

        cCollisionEvent *contact = tool_cursor->m_hapticPoint->getCollisionEvent(0);
        cVector3d voxel_idx(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);
        m_voxelObj->m_texture->m_image->getVoxelColor(uint(voxel_idx.x()), uint(voxel_idx.y()), uint(voxel_idx.z()), m_storedColor);
        bool in_collision = m_storedColor != m_zeroColor;
        if (in_collision)
        {
            double m1 = body->getMass();
            auto r1 = tool_cursor->m_hapticPoint->getRadiusContact();
            auto c_surface_point = tool_cursor->m_hapticPoint->getGlobalPosGoal();
            Eigen::Vector3d surface_point;
            surface_point << c_surface_point.x(), c_surface_point.y(), c_surface_point.z();



            auto c_inner_point = tool_cursor->m_hapticPoint->getGlobalPosProxy();
            Eigen::Vector3d inner_point;
            inner_point << c_inner_point.x(), c_inner_point.y(), c_inner_point.z();

            Eigen::Vector3d error_vector = surface_point - inner_point;

            Eigen::Vector3d n_plane;
            n_plane << contact->m_globalNormal.x(), contact->m_globalNormal.y(), contact->m_globalNormal.z();
            Eigen::Vector3d n = -n_plane;
            
            double C = error_vector.dot(n);

            // only consider the translational component
            Eigen::Matrix<double, 1, 3> J_trans;
            J_trans << -n.transpose();

            Eigen::Matrix<double, 3, 1> V;
            V.setZero();
            V(0) = body->m_bulletRigidBody->getLinearVelocity().x();
            V(1) = body->m_bulletRigidBody->getLinearVelocity().y();
            V(2) = body->m_bulletRigidBody->getLinearVelocity().z();

            Eigen::Matrix<double, 3, 1> F_ext;
            F_ext.setZero();
            F_ext(0) = body->m_bulletRigidBody->getTotalForce().x();
            F_ext(1) = body->m_bulletRigidBody->getTotalForce().y();
            F_ext(2) = body->m_bulletRigidBody->getTotalForce().z();
            // F_ext(0) = body->m_estimatedForce.x();
            // F_ext(1) = body->m_estimatedForce.y();
            // F_ext(2) = body->m_estimatedForce.z();

            // std::cout << body->m_estimatedForce.x() << " " << body->m_estimatedForce.y() << " " << body->m_estimatedForce.z() << std::endl;

            // double bias = compute_bias(C, V, n_plane, dt, b, a);
            double b_slop = 0.00001;
            double a_slop = 0.1;
            double pen_bias = (-b / dt) * std::max(std::abs(C) - b_slop, 0.);
            // pen_bias = (-b / dt) * C
            double Vc = - V.dot(n);
            double restitution_bias = a * Vc;
            double bias = pen_bias + restitution_bias;
            double inv_m1 = m1 > 0. ? 1. / m1 : 0.;
            Eigen::Matrix<double, 3, 3> M_inv = inv_m1 * Eigen::Matrix<double, 3, 3>::Identity();

            Eigen::Matrix<double, 1, 1> K = J_trans * M_inv * J_trans.transpose();
            double inv_K = 1. / K(0, 0);
            Eigen::Matrix<double, 3, 1> Vi = V + M_inv * F_ext * dt;
            double lam = -inv_K * (J_trans * Vi + bias);
            auto P = J_trans.transpose() * lam * dt;

            imp_out = btVector3(P(0), P(1), P(2));
            // imp_out *= m_debug_scalar;
            std::cout << "C: " << C << std::endl;
            std::cout << "impulse: " << imp_out.x() << ", " << imp_out.y() << ", " << imp_out.z() << std::endl;
        }
        return imp_out;
    }


}