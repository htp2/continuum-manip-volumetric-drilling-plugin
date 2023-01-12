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

#include <boost/program_options.hpp>
#include <fstream>

using namespace std;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
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

void afVolmetricDrillingPlugin::physicsUpdate(double dt)
{
    checkForSettingsUpdate();

    m_worldPtr->getChaiWorld()->computeGlobalPositions(true);

    if (m_CM_moved_by_other) // i.e. if the CM is not being moved by keyboard / device, but driven by a ROS node
    {
        T_contmanip_base = m_contManipBaseRigidBody->getLocalTransform(); // let's find out where it is now
    }
    else
    {                                                                                          // i.e. if the CM is being moved 'manually' using this plugin e.g. by keyboard / device
        if (!cTransformEqual(m_contManipBaseRigidBody->getLocalTransform(), T_contmanip_base)) // update CM for commanded movements
        {
            cTransform T_newContManipBase;
            T_newContManipBase.setLocalPos(T_contmanip_base.getLocalPos());
            T_newContManipBase.setLocalRot(T_contmanip_base.getLocalRot());
            T_newContManipBase = T_newContManipBase * btTransformTocTransform(m_contManipBaseRigidBody->getInertialOffsetTransform()); // handle offset due to fact that origin is not at center of body
            m_contManipBaseRigidBody->setLocalTransform(T_newContManipBase);
        }
    }

    toolCursorsPosUpdate(T_contmanip_base);

    f_i = 0;

    if (m_volume_collisions_enabled)
    {
        cToolCursor *burr_cursor = m_burrToolCursorList.back();

        if (m_burrOn && burr_cursor->isInContact(m_voxelObj)) //&& m_targetToolCursorIdx == 0 /*&& (userSwitches == 2)*/)
        {
            cCollisionEvent *contact = burr_cursor->m_hapticPoint->getCollisionEvent(0);
            cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);

            m_voxelObj->m_texture->m_image->getVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_storedColor);

            bool remove_voxel = m_storedColor != m_zeroColor;

            if (remove_voxel)
            {
                int removalCount = cMin(m_removalCount, (int)contact->m_events.size());
                // std::cout << "Contact: " << (int)contact->m_events.size() << " voxels" << std::endl;
                m_mutexVoxel.acquire();
                for (int cIdx = 0; cIdx < removalCount; cIdx++)
                {

                    cVector3d ct(contact->m_events[cIdx].m_voxelIndexX, contact->m_events[cIdx].m_voxelIndexY, contact->m_events[cIdx].m_voxelIndexZ);

                    if (m_experimental_behavior)
                    {
                        m_force_to_drill_voxel[uint(ct.x())][uint(ct.y())][uint(ct.z())] -= 1.0 / float(m_removalCount);
                        // std::cout << "Voxel at " << ct << " m_force_to_drill: = " <<m_force_to_drill_voxel[uint(ct.x())][uint(ct.y())][uint(ct.z())] <<std::endl;
                        if (m_force_to_drill_voxel[uint(ct.x())][uint(ct.y())][uint(ct.z())] > 0.0)
                        {
                            continue;
                        }
                    }

                    cColorb colorb;
                    m_voxelObj->m_texture->m_image->getVoxelColor(uint(ct.x()), uint(ct.y()), uint(ct.z()), colorb);
                    cColorf colorf = colorb.getColorf();

                    double sim_time = m_contManipBaseRigidBody->getCurrentTimeStamp();
                    double voxel_array[3] = {ct.get(0), ct.get(1), ct.get(2)};

                    float color_array[4];
                    color_array[0] = colorf.getR();
                    color_array[1] = colorf.getG();
                    color_array[2] = colorf.getB();
                    color_array[3] = colorf.getA();
                    m_voxelObj->m_texture->m_image->setVoxelColor(uint(ct.x()), uint(ct.y()), uint(ct.z()), m_zeroColor);

                    m_volumeUpdate.enclose(cVector3d(uint(ct.x()), uint(ct.y()), uint(ct.z())));
                    m_drillingPub->voxelsRemoved(voxel_array, color_array, sim_time);
                }
                m_mutexVoxel.release();
                m_flagMarkVolumeForUpdate = true;
            }
        }
        // compute interaction forces
        for (auto &cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList})
        {
            for (auto &cursor : cursor_list)
            {
                cursor->computeInteractionForces();
            }
        }
        for (int i = 0; i < m_segmentToolCursorList.size(); i++)
        {
            auto seg_force = calculate_force_from_tool_cursor_collision(m_segmentToolCursorList[i], m_segmentBodyList[i], dt);
            m_segmentBodyList[i]->applyForce(seg_force);
            // m_segmentBodyList[i]->applyForce(m_debug_value * cVector3d(F(0), F(1), F(2)));
        }
        // apply force from burr
        auto burr_force = calculate_force_from_tool_cursor_collision(m_burrToolCursorList[0], m_burrBody, dt);
        m_burrBody->applyForce(burr_force);
    }

    for (int i = 0; i < m_shaftToolCursorList.size(); i++)
    {
        
        auto shaft_force = calculate_force_from_tool_cursor_collision(m_shaftToolCursorList[0], m_contManipBaseRigidBody, dt);
        m_contManipBaseRigidBody->applyForceAtPointOnBody(shaft_force, m_shaftToolCursorList[i]->getDeviceLocalPos());
        // m_contManipBaseRigidBody->applyForceAtPointOnBody(100000.0 * m_shaftToolCursorList[i]->getDeviceLocalForce(), m_shaftToolCursorList[i]->getDeviceLocalPos());
    }

    applyCablePull(dt);
}

int afVolmetricDrillingPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld)
{
    // Parse command line options
    namespace p_opt = boost::program_options;
    p_opt::options_description cmd_opts("drilling_simulator Command Line Options");
    cmd_opts.add_options()("info", "Show Info");
    cmd_opts.add_options()("anatomy_volume_name", p_opt::value<std::string>()->default_value("cube"), "Name of volume given in yaml. Default spine_test_volume");
    cmd_opts.add_options()("base_body_name", p_opt::value<std::string>()->default_value("snake_stick"), "Name of body given in yaml. Default snake_stick");
    cmd_opts.add_options()("tool_body_name", p_opt::value<std::string>()->default_value("Burr"), "Name of body given in yaml. Default Burr");
    cmd_opts.add_options()("experimental_behavior", p_opt::value<std::string>()->default_value("0"), ". Turn on [experimental] features Default false");
    cmd_opts.add_options()("debug_traj_file", p_opt::value<std::string>()->default_value(""), ". Input needed for [experimental] features Default empty");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if (var_map.count("info"))
    {
        std::cout << cmd_opts << std::endl;
        return -1;
    }

    string file_path = __FILE__;
    string cur_path = file_path.substr(0, file_path.rfind("/"));

    std::string anatomy_volume_name = var_map["anatomy_volume_name"].as<std::string>();
    std::string base_body_name = var_map["base_body_name"].as<std::string>();
    std::string tool_body_name = var_map["tool_body_name"].as<std::string>();
    std::string experimental_behavior = var_map["experimental_behavior"].as<std::string>();
    m_experimental_behavior = boost::lexical_cast<bool>(experimental_behavior);
    std::string debug_traj_file = var_map["debug_traj_file"].as<std::string>();

    // Bring in ambf world, make adjustments as needed to improve simulation accuracy
    m_worldPtr = a_afWorld;
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->setGravity(btVector3(0.0, 0.0, 0.0));

    // Various scalars needed for other calculation
    m_to_ambf_unit = 10.0;
    mm_to_ambf_unit = m_to_ambf_unit / 1000.0;
    double burr_r = mm_to_ambf_unit * 6.5 / 2;
    m_zeroColor = cColorb(0x00, 0x00, 0x00, 0x00);
    m_boneColor = cColorb(255, 249, 219, 255);
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
    volumeInit(a_afWorld);
    // Initializing tool cursors (for CM to interact with the volume)
    toolCursorInit(a_afWorld);
    // Initialize visualizations on camera
    visualInit(a_afWorld);

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

    if (m_experimental_behavior)
        experimentalBehaviorInit(debug_traj_file);

    // Set up cable pull subscriber
    m_cablePullSub = new CablePullSubscriber("ambf", "volumetric_drilling");

    // Rand num gen
    std::random_device rd;
    rand_eng = std::mt19937(rd());
    unif_dist = std::uniform_real_distribution<>(0, 1);

    return 1;
}

void afVolmetricDrillingPlugin::visualInit(const afWorldPtr a_afWorld)
{
    // Get first camera
    m_mainCamera = m_worldPtr->getCameras()[0];
    // create a font
    cFontPtr font = NEW_CFONTCALIBRI40();
    m_cablePullMagText = new cLabel(font);
    m_cablePullMagText->setLocalPos(20, 70);
    m_cablePullMagText->m_fontColor.setBlack();
    m_cablePullMagText->setFontScale(.5);
    UpdateCablePullText();
    m_mainCamera->getFrontLayer()->addChild(m_cablePullMagText);

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
}

void afVolmetricDrillingPlugin::volumeInit(const afWorldPtr a_afWorld)
{
    m_voxelObj = m_volumeObject->getInternalVolume();

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
}

///
/// \brief This method initializes the tool cursors.
/// \param a_afWorld    A world that contains all objects of the virtual environment
/// \return
///
void afVolmetricDrillingPlugin::toolCursorInit(const afWorldPtr a_afWorld)
{
    cWorld *chai_world = a_afWorld->getChaiWorld();
    int num_segs = 27;
    int num_shaft_cursor = 6;
    int num_burr_cursor = 1;

    for (int i = 1; i <= num_segs; i++)
    {
        m_segmentBodyList.push_back(m_worldPtr->getRigidBody("/ambf/env/BODY seg" + to_string(i)));
        m_segmentJointList.push_back(m_worldPtr->getJoint("/ambf/env/JOINT joint" + to_string(i)));
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
        shaft_cursor->setRadius(mm_to_ambf_unit * 6 / 2);
    }

    for (auto &burr_cursor : m_burrToolCursorList)
    {
        burr_cursor->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
        burr_cursor->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
        burr_cursor->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
        burr_cursor->setRadius(mm_to_ambf_unit * 6.5 / 2);
    }

    for (auto &seg_cursor : m_segmentToolCursorList)
    {
        seg_cursor->setShowContactPoints(m_showGoalProxySpheres, m_showGoalProxySpheres);
        seg_cursor->m_hapticPoint->m_sphereProxy->m_material->setGreenChartreuse();
        seg_cursor->m_hapticPoint->m_sphereGoal->m_material->setOrangeCoral();
        seg_cursor->setRadius(mm_to_ambf_unit * 6 / 2);
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

    for (auto &cursor_list : {m_shaftToolCursorList, m_segmentToolCursorList, m_burrToolCursorList})
    {
        for (auto &cursor : cursor_list)
        {
            e_int.push_back(cVector3d(0.0, 0.0, 0.0));
            e_prev.push_back(cVector3d(0.0, 0.0, 0.0));
        }
    }
}

///
/// \brief incrementDevicePos
/// \param a_vel
///
void afVolmetricDrillingPlugin::incrementDevicePos(cVector3d a_vel)
{
    T_contmanip_base.setLocalPos(T_contmanip_base.getLocalPos() + a_vel);
}

///
/// \brief incrementDeviceRot
/// \param a_rot
///
void afVolmetricDrillingPlugin::incrementDeviceRot(cVector3d a_rot)
{
    cMatrix3d R_cmd;
    R_cmd.setExtrinsicEulerRotationDeg(a_rot(0), a_rot(1), a_rot(2), C_EULER_ORDER_XYZ);
    R_cmd = T_contmanip_base.getLocalRot() * R_cmd;
    T_contmanip_base.setLocalRot(R_cmd);
}

///
/// \brief This method updates the position of the shaft tool cursors
/// which eventually updates the position of the whole tool.
///
void afVolmetricDrillingPlugin::toolCursorsPosUpdate(cTransform a_targetPose)
{
    for (int i = 0; i < m_shaftToolCursorList.size(); i++)
    {
        // static cast i to double
        double offset = static_cast<double>(i) / (m_shaftToolCursorList.size() - 1) * 35.0 * mm_to_ambf_unit;
        auto T_offset = cTransform(cVector3d(0, offset, 0), cMatrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
        m_shaftToolCursorList[i]->setDeviceLocalTransform(a_targetPose * T_offset);
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
/// \brief This method updates the position of the drill mesh.
/// After obtaining g_targetToolCursor, the drill mesh adjust it's position and rotation
/// such that it follows the proxy position of the g_targetToolCursor.
///
void afVolmetricDrillingPlugin::drillPoseUpdateFromCursors()
{
    cMatrix3d newContManipBaseRot;
    newContManipBaseRot = m_shaftToolCursorList[0]->getDeviceLocalRot();
    cTransform T_newContManipBase;
    T_newContManipBase.setLocalPos(m_shaftToolCursorList[0]->m_hapticPoint->getLocalPosProxy());
    T_newContManipBase.setLocalRot(newContManipBaseRot);
    T_newContManipBase = T_newContManipBase * btTransformTocTransform(m_contManipBaseRigidBody->getInertialOffsetTransform()); // handle offset due to fact that origin is not at center of body
    m_contManipBaseRigidBody->setLocalTransform(T_newContManipBase);
}

void afVolmetricDrillingPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    if (a_mods == GLFW_MOD_CONTROL)
    {

        // controls linear motion of tool
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

        else if (a_key == GLFW_KEY_O)
        {
            setDrillControlMode(!m_CM_moved_by_other);
        }

        else if (a_key == GLFW_KEY_C)
        {
            setShowToolCursors(!m_showGoalProxySpheres);
        }

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
        else if (a_key == GLFW_KEY_LEFT_BRACKET)
        {
            setVolumeCollisionsEnabled(!m_volume_collisions_enabled);
        }
        else if (a_key == GLFW_KEY_RIGHT_BRACKET)
        {
            // toolCursorInit(m_worldPtr);
        }
        else if (a_key == GLFW_KEY_EQUAL)
        {
            m_burrOn = !m_burrOn;
            std::cout << "Burr On State: " << m_burrOn << std::endl;
        }

        else if (a_key == GLFW_KEY_SLASH)
        {
            setCableControlMode(!m_cableKeyboardControl);
        }

        else if (a_key == GLFW_KEY_E)
        {
            setPhysicsPaused(!m_worldPtr->isPhysicsPaused());
        }

        // option - polygonize model and save to file
        else if (a_key == GLFW_KEY_F9)
        {
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

        else if (a_key == GLFW_KEY_N)
        {
            cout << "INFO! RESETTING THE VOLUME" << endl;
            // m_volumeObject->reset(); //TODO REENABLE
        }
    }
    else if (a_mods == GLFW_MOD_ALT)
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

        // controls rotational motion of tool
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
        else if (a_key == GLFW_KEY_KP_7)
        {
            if (m_pid == 0)
            {
                m_P -= 0.1;
                std::cout << "m_P: " << m_P << std::endl;
            }
            else if (m_pid == 1)
            {
                m_I -= 0.1;
                std::cout << "m_I: " << m_I << std::endl;
            }
            else if (m_pid == 2)
            {
                m_D -= 0.1;
                std::cout << "m_D: " << m_D << std::endl;
            }
        }
        else if (a_key == GLFW_KEY_KP_9)
        {
            if (m_pid == 0)
            {
                m_P += 0.1;
                std::cout << "m_P: " << m_P << std::endl;
            }
            else if (m_pid == 1)
            {
                m_I += 0.001;
                std::cout << "m_I: " << m_I << std::endl;
            }
            else if (m_pid == 2)
            {
                m_D += 0.1;
                std::cout << "m_D: " << m_D << std::endl;
            }
        }
        else if (a_key == GLFW_KEY_KP_0)
        {
            m_pid = m_pid + 1;
            if (m_pid > 2)
            {
                m_pid = 0;
            }
        }
        else if (a_key == GLFW_KEY_KP_DECIMAL)
        {
            m_debug_print = !m_debug_print;
            std::cout << "m_debug_print: " << m_debug_print << std::endl;
        }
        else if (a_key == GLFW_KEY_X)
        {

            if (m_suddenJump)
            {
                m_suddenJump = false;
            }

            else
            {
                m_suddenJump = true;
            }
        }

        // toggles the visibility of drill mesh in the scene
        else if (a_key == GLFW_KEY_B)
        {
            m_showDrill = !m_showDrill;
            m_contManipBaseRigidBody->m_visualMesh->setShowEnabled(m_showDrill);
            for (auto &seg : m_segmentBodyList)
            {
                seg->m_visualMesh->setShowEnabled(m_showDrill);
            }
            m_burrBody->m_visualMesh->setShowEnabled(m_showDrill);
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
    for (auto tool : m_toolCursorList)
    {
        tool->stop();
    }

    delete m_deviceHandler;

    return true;
}

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

    // Add 'noise' to cable pull commands (simulates cable friction, etc.)
    bool m_cable_pull_noise = false;
    if (m_cable_pull_noise)
    {
        double cable_err_mult_min = 0.0;
        double cable_err_mult_max = 1000.0;

        double cable_err_denom_min = 0.0;
        double cable_err_denom_max = 1000.0;

        cable_err_denom += ((cable_err_mult_min + unif_dist(rand_eng) * (cable_err_mult_max - cable_err_mult_min)) * cable_pull_mag_change);

        cable_err_denom = std::min(cable_err_denom, cable_err_denom_max);
        cable_err_denom = std::max(cable_err_denom, cable_err_denom_min);

        cable_pull_force_val = m_cable_pull_mag / (1.0 + cable_err_denom * m_cable_pull_mag * m_cable_pull_mag);
    }

    m_cablePullSub->publish_cablepull_measured_js(m_cable_pull_mag, m_cable_pull_velocity);
    auto last_seg_ptr = m_segmentBodyList.back();
    last_seg_ptr->applyTorque(10.0 * cable_pull_force_val * last_seg_ptr->getLocalRot().getCol2());
    // last_seg_ptr->applyForce(cVector3d(-10.0*m_cable_pull_mag*last_seg_ptr->getLocalRot().getCol1()), last_seg_ptr->getLocalRot()*cVector3d( 2 *mm_to_ambf_unit,0.0,0.0) );
}

cTransform afVolmetricDrillingPlugin::btTransformTocTransform(const btTransform &in)
{
    const auto &in_pos = in.getOrigin();
    const auto &in_rot = in.getRotation();

    cVector3d pos(in_pos.x(), in_pos.y(), in_pos.z());
    const auto &axis = in_rot.getAxis();
    cVector3d c_axis(axis.getX(), axis.getY(), axis.getZ());
    cMatrix3d rot(c_axis, in_rot.getAngle());
    cTransform out(pos, rot);
    return out;
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

bool afVolmetricDrillingPlugin::fillGoalPointsFromCSV(const std::string &filename, std::vector<cVector3d> &trace_points)
{
    std::ifstream file(filename);
    if (!file)
    {
        std::cout << "Error reading: " << filename << std::endl;
        return false;
    }
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
        trace_points.push_back(cVector3d(std::stod(record[0]), std::stod(record[1]), std::stod(record[2])));
    }
    return true;
}

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
        // m_volumeObject->reset();// TODO: reenable this
        m_settingsPub->resetVoxels_changed = false;
    }
    if (m_settingsPub->setBurrOn_changed)
    {
        m_burrOn = m_settingsPub->setBurrOn_last_val;
        m_settingsPub->setBurrOn_changed = false;
    }
}

void afVolmetricDrillingPlugin::experimentalBehaviorInit(const std::string &debug_traj_file)
{
    // Start experimental behavior
    // std::vector<std::vector<std::vector<double>>> forces(voxelCount[0], std::vector<std::vector<double>>(voxelCount[1], std::vector<double>(voxelCount[2], 1.0)));
    // m_force_to_drill_voxel = forces;
    // m_experimental_behavior = true;
    if (m_experimental_behavior)
    {
        std::cout << "Starting experimental behavior" << std::endl;
        std::vector<std::vector<std::vector<double>>> forces(m_volumeObject->getVoxelCount().get(0), std::vector<std::vector<double>>(m_volumeObject->getVoxelCount().get(1), std::vector<double>(m_volumeObject->getVoxelCount().get(2), 1.0)));
        // load forces from file /home/henry/snake_registration/simulation/anatomy/CT_HU_masked.csv

        std::string debug_vol_file = "/home/henry/snake_registration/simulation/anatomy/CT_HU_masked.csv";
        std::ifstream file(debug_vol_file);
        if (!file)
        {
            std::cout << "Error reading: " << debug_vol_file << std::endl;
            return;
        }

        // read first line
        std::string s;
        if (!std::getline(file, s))
            return;
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
            std::cout << "Error reading: " << debug_vol_file << std::endl;
            return;
        }
        if (std::stoi(record[0]) != m_volumeObject->getVoxelCount().get(0) || std::stoi(record[1]) != m_volumeObject->getVoxelCount().get(1) || std::stoi(record[2]) != m_volumeObject->getVoxelCount().get(2))
        {
            std::cout << "Error reading: " << debug_vol_file << std::endl;
            return;
        }

        // rest of the file is one number per line which is the force, loop through each dimension of forces
        for (size_t i = 0; i < m_volumeObject->getVoxelCount().get(0); i++)
        {
            for (size_t j = 0; j < m_volumeObject->getVoxelCount().get(1); j++)
            {
                for (size_t k = 0; k < m_volumeObject->getVoxelCount().get(2); k++)
                {
                    if (!std::getline(file, s))
                        return;
                    forces[i][j][k] = std::stod(s) + 0.5;
                }
            }
        }

        file.close();

        m_force_to_drill_voxel = forces;

        // remove from ball around entry point
        double entry_burr_size = 9.0 * mm_to_ambf_unit;
        double entry_depth = 5.0 * mm_to_ambf_unit;
        size_t entry_cut_points_size = 60;

        std::vector<cVector3d> trace_points;
        fillGoalPointsFromCSV(debug_traj_file, trace_points);
        std::vector<cVector3d> entry_cut_points;
        // assume we start with straight path which is parallel to trace_points[1] - trace_points[0]
        auto entry_direction = cNormalize(trace_points[1] - trace_points[0]);
        // make entry_cut_points go from -entry_depth mm to entry_depth mm in entry_direction relative to trace_points[0] and it will have length of 60 points
        for (size_t i = 0; i < entry_cut_points_size; i++)
        {
            entry_cut_points.push_back(trace_points[0] + entry_direction * entry_depth * (2 * (static_cast<double>(i) / static_cast<double>(entry_cut_points_size)) - 1));
        }

        auto T_inv = m_volumeObject->getLocalTransform();
        T_inv.identity();
        int resol = 20;
        m_mutexVoxel.acquire();
        for (auto &pt : entry_cut_points)
        {
            for (size_t i = 0; i <= resol; i++)
            {
                for (size_t j = 0; j <= resol; j++)
                {
                    for (size_t k = 0; k <= resol; k++)
                    {
                        auto denom = static_cast<double>(resol);
                        double r = i / denom * entry_burr_size / 2.0;
                        double th = j / denom * 2 * PI;
                        double phi = k / denom * PI;
                        cVector3d offset(r * sin(phi) * cos(th), r * sin(phi) * sin(th), r * cos(phi));

                        cVector3d idx;
                        auto new_pt = T_inv * pt + offset;
                        // std::cout << "actual point " << new_pt << std::endl;
                        m_volumeObject->localPosToVoxelIndex(new_pt, idx);
                        m_voxelObj->m_texture->m_image->setVoxelColor(uint(idx.x()), uint(idx.y()), uint(idx.z()), m_zeroColor);
                        m_volumeUpdate.enclose(cVector3d(uint(idx.x()), uint(idx.y()), uint(idx.z())));
                    }
                }
            }
        }
        m_mutexVoxel.release();
        graphicsUpdate();
    }
}
bool afVolmetricDrillingPlugin::cTransformEqual(const cTransform &a, const cTransform &b)
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

cVector3d afVolmetricDrillingPlugin::calculate_force_from_tool_cursor_collision(cToolCursor *tool_cursor, afRigidBodyPtr &body, double dt)
{
    // ouput force from collision (default zero)
    cVector3d force_out(0.0, 0.0, 0.0);
    
    // voxel collision properties
    double m2 = 0.;
    auto dim = m_volumeObject->getDimensions();
    auto num_voxels = m_volumeObject->getVoxelCount();
    // get max of dim[i]/num_voxels[i] for i = 0,1,2
    double r2 = 0.0;
    for (size_t i = 0; i < 3; i++)
    {
        r2 = cMax(r2, dim(i) / num_voxels(i))*4.0;
    }

    cCollisionEvent *contact = tool_cursor->m_hapticPoint->getCollisionEvent(0);
    cVector3d orig(contact->m_voxelIndexX, contact->m_voxelIndexY, contact->m_voxelIndexZ);
    m_voxelObj->m_texture->m_image->getVoxelColor(uint(orig.x()), uint(orig.y()), uint(orig.z()), m_storedColor);
    bool in_collision = m_storedColor != m_zeroColor;
    if (in_collision)
    {
        // auto collision_point = contact->m_globalPos;

        double m1 = body->getMass();
        auto r1 = tool_cursor->m_hapticPoint->getRadiusContact();
        auto cx1 = tool_cursor->m_hapticPoint->getGlobalPosGoal();
        Eigen::Vector3d x1;
        x1 << cx1.x(), cx1.y(), cx1.z();

        Eigen::Vector3d x2;
        // x2 << collision_point.x(), collision_point.y(), collision_point.z();
        cVector3d cx2;
        m_volumeObject->voxelIndexToLocalPos(orig, cx2);
        cx2 = m_volumeObject->getLocalTransform() * cx2;
        x2 << cx2.x(), cx2.y(), cx2.z();

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
        
        // std::cout << body->getMass() << std::endl;

        Eigen::Matrix<double, 12, 1> P;
        P.setZero();

        bool in_contact = compute_impulse_two_sphere_collision(P, x1, x2, r1, r2, m1, m2, dt, V, F_ext, 0.4);
        if (in_contact)
        {
            auto F = P / dt;
            force_out = cVector3d(F(0), F(1), F(2));
            // std::cout << "force_out: " << force_out << std::endl;
        }
        else
        {
            // std::cout << "SI says no contact, but tool cursor says contact" << std::endl;
        }


        // if(false)
        // {auto e = tool_cursor->m_hapticPoint->getGlobalPosProxy() - tool_cursor->m_hapticPoint->getGlobalPosGoal();
        // e_int[f_i] = e_int[f_i] + e;
        // auto de = e_prev[f_i] - e;

        // force_out = m_P * e + m_I * e_int[f_i] + m_D * de;
        // e_prev[f_i] = e;
        // f_i++;
        // }
    }
    else{
        // e_int[f_i].zero();
    }

    force_out = force_out * m_P;

    return force_out;
}