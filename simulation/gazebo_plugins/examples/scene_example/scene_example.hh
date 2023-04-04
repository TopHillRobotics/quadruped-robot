/*
 *  Copyright (C) 2018 João Borrego
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *      
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*!
    \file examples/scene_example/scene_example.hh
    \brief Random scene generation example

    Generates a scene with up to 10 objects in a 4x4 grid.

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Includes

// Gazebo
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

// Custom messages
#include "camera_utils_request.pb.h"
#include "camera_utils_response.pb.h"
#include "visual_utils_request.pb.h"
#include "visual_utils_response.pb.h"
#include "world_utils_request.pb.h"
#include "world_utils_response.pb.h"

// Utilities
#include "utils.hh"
// Object class
#include "ObjectGrid.hh"

// I/O streams
#include <iostream>
// File streams
#include <fstream>
// Iterating over the contents of a dir
#include <boost/filesystem.hpp>
// Protecting variables
#include <mutex>
// Set container
#include <set>
// Sleep
#include <chrono>
#include <thread>
// Regular expressions
#include <regex>
// Linear algebra
#include <Eigen/Dense>
// INT MAX
#include <climits>

//////////////////////////////////////////////////

// Macros

// Fixed viewpoint
#define FIXED_VIEW 0
// Moving viewpoint, change in position
#define MOVING_VIEW_POS 1
// Moving viewpoint, change in position and rotation
#define MOVING_VIEW_POS_ROT 2

// Do not raise documentation error 
//! @cond DoNotRaiseWarning

/// Matches name field in <model name=""> XML tag
#define REGEX_XML_MODEL "<model name=(\"([^\"]|\"\")*\")>"
/// Matches string enclosed in <uid> XML tags
#define REGEX_XML_UID   "<uid>[\\s\\S]*?<\\/uid>"

//! @endcond

//////////////////////////////////////////////////

// Macros for custom messages

// Camera utils

/// Request to move camera to given pose
#define MOVE_REQUEST            gap::msgs::CameraUtilsRequest::MOVE
/// Response acknowledging move camera request
#define MOVE_RESPONSE           gap::msgs::CameraUtilsResponse::MOVE
/// Request to capture a frame and save it to disk
#define CAPTURE_REQUEST         gap::msgs::CameraUtilsRequest::CAPTURE
/// Response acknowledging captured frame
#define CAPTURE_RESPONSE        gap::msgs::CameraUtilsResponse::CAPTURE
/// Request 3D to 2D point projection
#define PROJECTION_REQUEST      gap::msgs::CameraUtilsRequest::PROJECTION
/// Response 3D to 2D point projection
#define PROJECTION_RESPONSE     gap::msgs::CameraUtilsResponse::PROJECTION
/// Request to change camera plugin settings
#define OPTIONS                 gap::msgs::CameraUtilsRequest::OPTIONS

// Visual utils

/// Request update
#define UPDATE      gap::msgs::VisualUtilsRequest::UPDATE
/// Visual updated response
#define UPDATED     gap::msgs::VisualUtilsResponse::UPDATED

// World utils

/// Spawn entity
#define SPAWN           gap::msgs::WorldUtilsRequest::SPAWN
/// Move entity
#define WORLD_MOVE      gap::msgs::WorldUtilsRequest::MOVE
/// Start or stop physcis simulation
#define PHYSICS         gap::msgs::WorldUtilsRequest::PHYSICS

/// Spawn custom object
#define CUSTOM          gap::msgs::Object::CUSTOM
/// Spawn custom light object
#define CUSTOM_LIGHT    gap::msgs::Object::CUSTOM_LIGHT

//////////////////////////////////////////////////

// API Topics

/// Topic monitored by CameraUtils plugin for incoming requests
#define CAMERA_UTILS_TOPIC          "~/gap/camera_utils"
/// Topic for receiving replies from CameraUtils plugin
#define CAMERA_UTILS_RESPONSE_TOPIC "~/gap/camera_utils/response"
/// Topic monitored by VisualUtils plugin for incoming requests
#define VISUAL_UTILS_TOPIC          "~/gap/visual_utils"
/// Topic for receiving replies from VisualUtils plugin
#define VISUAL_UTILS_RESPONSE_TOPIC "~/gap/visual_utils/response"
/// Topic monitored by WorldUtils plugin for incoming requests
#define WORLD_UTILS_TOPIC           "~/gap/world_utils"
/// Topic for receiving replies from WorldUtils plugin
#define WORLD_UTILS_RESPONSE_TOPIC  "~/gap/world_utils/response"

// Message pointer typedefs

/// Pointer to CameraUtils response message
typedef const boost::shared_ptr<const gap::msgs::CameraUtilsResponse>
    CameraUtilsResponsePtr;
/// Pointer to VisualUtils request message
typedef const boost::shared_ptr<const gap::msgs::VisualUtilsResponse>
    VisualUtilsResponsePtr;
/// Pointer to WorldUtils request message
typedef const boost::shared_ptr<const gap::msgs::WorldUtilsResponse>
    WorldUtilsResponsePtr;

//////////////////////////////////////////////////

/// Function prototypes

/// \brief Adds an SDF model to a WorldUtils request
/// \param msg  WorldUtils request message
/// \param file SDF file with model
void addModelFromFile(
    gap::msgs::WorldUtilsRequest & msg,
    const std::string & file);

/// \brief Add objects in global grid to WorldUtils spawn request
/// \param msg  WorldUtils request message
void addDynamicModels(gap::msgs::WorldUtilsRequest & msg);

/// \brief Add objects in global grid to VisualUtils update request
void updateObjects(gap::msgs::VisualUtilsRequest & msg);

/// \brief Add move object command to WorldUtils request
/// \param msg      WordlUtils request
/// \param name     Object name
/// \param is_light Whether object is a light
/// \param pose     New object pose
void addMoveObject(
    gap::msgs::WorldUtilsRequest & msg,
    const std::string & name,
    const bool is_light,
    const ignition::math::Pose3d & pose);

/// \brief Obtain random camera pose in dome
/// \return New random camera pose
ignition::math::Pose3d getRandomCameraPose();

/// \brief Obtain random light pose in dome
/// \return New random light pose
ignition::math::Pose3d getRandomLightPose();

/// \brief Send CameraUtils request to capture current scene
/// \param pub          Publisher for CameraUtils request topic
/// \param iteration    Current iteration
void captureScene(gazebo::transport::PublisherPtr pub, int iteration);

/// \brief Wait for camera to move to new pose
/// \return True if process should wait
bool waitForMove();

/// \brief Wait for visuals to update
/// \return True if process should wait
bool waitForVisuals();

/// \brief Wait for camera to save frame to disk
/// \return True if process should wait
bool waitForCamera();

/// \brief Wait for projected points
bool waitForProjections();

/// \brief Create set with names of existing objects
void createNameSet();

/// \brief Add 3D points to projection request
void addProjections(gap::msgs::CameraUtilsRequest & msg);

/// \brief Move camera to global camera pose
void moveCamera(gazebo::transport::PublisherPtr pub);

/// \brief Callback function for CameraUtils response
/// \param _msg Incoming message
void onCameraUtilsResponse(CameraUtilsResponsePtr & _msg);

/// \brief Callback function for VisualUtils response
/// \param _msg Incoming message
void onVisualUtilsResponse(VisualUtilsResponsePtr & _msg);

/// \brief Callback function for WorldUtils response
/// \param _msg Incoming message
void onWorldUtilsResponse(WorldUtilsResponsePtr & _msg);

/// \brief Enables/disables physics engine
/// \param pub WorldUtils publisher ptr
/// \param enable Desired physics engine status
void setPhysics(gazebo::transport::PublisherPtr pub, bool enable);

/// \brief Debug function to visualise scene
/// Debug function to visualise acquired frame and object bounding boxes
void visualizeData(const std::string & image_dir, int iteration);

/// \brief Store current scene annotations
/// \param path         Path to dataset folder
/// \param iteration    Current iteration
void storeAnnotations(
    const std::string & path,
    const int iteration);
