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
    \file world_utils/WorldUtils.hh
    \brief World Utils plugin

    A custom gazebo plugin that provides an interface to programatically
    interact with the World object.

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include <gazebo/common/Events.hh>
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include "gazebo/physics/physics.hh"
#include <gazebo/transport/transport.hh>

// Mutex
#include <mutex>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <list>
#include <string>
#include <regex>
// Boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

// Custom messages
#include "world_utils_request.pb.h"
#include "world_utils_response.pb.h"

// Õbjects with pending move operations
#include "MoveObject.hh"

namespace WorldUtils {

/// Topic monitored for incoming commands
#define REQUEST_TOPIC "~/gap/world_utils"
/// Topic for publishing replies
#define RESPONSE_TOPIC "~/gap/world_utils/response"

// Ease of use macros

// Request

/// Spawn entity
#define SPAWN           gap::msgs::WorldUtilsRequest::SPAWN
/// Move entity
#define MOVE            gap::msgs::WorldUtilsRequest::MOVE
/// Remove entity from the world
#define REMOVE          gap::msgs::WorldUtilsRequest::REMOVE
/// Start or stop physcis simulation
#define PHYSICS         gap::msgs::WorldUtilsRequest::PHYSICS
/// Pause or resume simulation
#define PAUSE           gap::msgs::WorldUtilsRequest::PAUSE
/// Get entity or world information
#define STATUS          gap::msgs::WorldUtilsRequest::STATUS

/// Spawn sphere object
#define SPHERE          gap::msgs::Object::SPHERE
/// Spawn cylinder object
#define CYLINDER        gap::msgs::Object::CYLINDER
/// Spawn box object
#define BOX             gap::msgs::Object::BOX
/// Spawn custom object
#define CUSTOM          gap::msgs::Object::CUSTOM
/// Spawn custom light object
#define CUSTOM_LIGHT    gap::msgs::Object::CUSTOM_LIGHT
/// Spawn a model included in gazebo model path
#define MODEL           gap::msgs::Object::MODEL

// Response

/// \brief Provide world state information
#define INFO            gap::msgs::WorldUtilsResponse::INFO
/// \brief TODO
#define SUCCESS         gap::msgs::WorldUtilsResponse::SUCCESS

// Regex patterns

// Do not raise documentation warning
//! @cond DoNotRaiseWarning

/// Matches string enclosed in <script> XML tags
#define REGEX_XML_SCRIPT "<script>[\\s\\S]*?<\\/script>"
/// Matches string enclosed in <pose> XML tags
#define REGEX_XML_POSE   "<pose>[\\s\\S]*?<\\/pose>"

//! @endcond

}

namespace gazebo {

    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const gap::msgs::WorldUtilsRequest>
        WorldUtilsRequestPtr;
    /// Shared pointer declaration for response message type
    typedef const boost::shared_ptr<const gap::msgs::WorldUtilsResponse>
        WorldUtilsResponsePtr;

    /// \brief A custom gazebo plugin that provides an interface to
    /// programatically interact with the World object.
    ///
    /// See the example usage below:
    /// \code{.xml}
    ///    <plugin name="world" filename="libWorldUtils.so"/>
    /// \endcode
    ///
    /// See worlds/spawner.world for a complete example.
    ///
    /// \warning This plugin is likely to undergo major changes, as some
    /// of its features can easily be done client-side.
    class WorldUtils : public WorldPlugin {

        /// Mutex for safe data access
        public: std::mutex mutex;

        /// A pointer to the world
        private: physics::WorldPtr world;
        /// Connection to World Update events
        private: event::ConnectionPtr updateConnection;

        /// A node used for transport
        private: transport::NodePtr node;
        /// A subscriber to the request topic
        private: transport::SubscriberPtr sub;
        /// A publisher to the reply topic
        private: transport::PublisherPtr pub;

        /// A publisher to the gazebo request topic
        private: transport::PublisherPtr request_pub;
        /// A subscriber to the gazebo response topic
        private: transport::SubscriberPtr response_sub;
        /// A publisher to light modify topic
        private: transport::PublisherPtr light_pub;

        // Regex patterns

        /// Regex for applying custom material
        private: std::regex script_reg;
        /// Regex for applying custom pose
        private: std::regex pose_reg;

        // Counters for automatic naming

        /// Number of generated spheres
        private: int sphere_counter      {0};
        /// Number of generated cylinders
        private: int cylinder_counter    {0};
        /// Number of generated boxes
        private: int box_counter         {0};
        /// Number of generated lights
        private: int light_counter       {0};

        /// Queue of objects with pending move actions
        private: std::queue<MoveObject> move_queue;

        // Public methods

        /// \brief Constructs the object
        public: WorldUtils();

        /// \brief Loads the object
        /// \param _world   The World object to which the plugin is attached
        /// \param _sdf     The SDF element with plugin parameters
        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

        /// \brief Callback function for handling world updates
        public: void onUpdate();

        // Private methods

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        private: void onRequest(WorldUtilsRequestPtr &_msg);

        /// \brief Removes everything from the world
        private: void clearWorld();

        /// \brief Removes entities matching a given string
        /// \param match    The string to be matched
        /// \param is_light Whether to target light objects or not
        private: void clearMatching(const std::string & match, const bool is_light);

        /// \brief Returns the SDF of a sphere
        /// \param model_name   Model name
        /// \param mass         Model mass
        /// \param radius       Sphere radius
        /// \param position     Sphere position
        /// \param orientation  Sphere orientation
        /// \return Sphere SDF string
        /// \deprecated Unused feature, easily replaced by client command
        private: const std::string genSphere(
            const std::string &model_name,
            const double mass,
            const double radius,
            const ignition::math::Vector3d position,
            const ignition::math::Quaterniond orientation);

        /// \brief Returns the SDF of a cylinder
        /// \param model_name   Model name
        /// \param mass         Model mass
        /// \param radius       Cylinder radius
        /// \param length       Cylinder length
        /// \param position     Cylinder position
        /// \param orientation  Cylinder orientation
        /// \return Cylinder SDF string
        /// \deprecated Unused feature, easily replaced by client command
        private: const std::string genCylinder(
            const std::string &model_name,
            const double mass,
            const double radius,
            const double length,
            const ignition::math::Vector3d position,
            const ignition::math::Quaterniond orientation);

        /// \brief Returns the SDF of a box
        /// \param model_name   Model name
        /// \param mass         Model mass
        /// \param size         Box size 3D vector
        /// \param position     Box position
        /// \param orientation  Box orientation
        /// \return Box SDF
        /// \deprecated Unused feature, easily replaced by client command
        private: const std::string genBox(
            const std::string &model_name,
            const double mass,
            const ignition::math::Vector3d size,
            const ignition::math::Vector3d position,
            const ignition::math::Quaterniond orientation);
    };
}
