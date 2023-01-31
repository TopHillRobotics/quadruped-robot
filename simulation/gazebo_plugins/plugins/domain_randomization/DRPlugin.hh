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
    \file plugins/domain_randomization/DRPlugin.hh
    \brief Domain randomization Gazebo plugin headers

    Plugin for handling Domain Randomization requests

    \author João Borrego : jsbruglie
*/

#ifndef _DOMAIN_RANDOMIZATION_PLUGIN_HH_
#define _DOMAIN_RANDOMIZATION_PLUGIN_HH_

// Gazebo
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>

// Custom messages
#include "dr_request.pb.h"
#include "model_cmd.pb.h"
#include "dr_response.pb.h"
// Custom gazebo debug utilities
#include "gz_debug.hh"

// Required fields workaround
#include <limits>

namespace gazebo {

    // Message types

    /// Declaration for request message type
    typedef gap::msgs::DRRequest DRRequest;
    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const gap::msgs::DRRequest>
        DRRequestPtr;
    /// Declaration for response message type
    typedef gap::msgs::DRResponse DRResponse;
    /// Shared pointer declaration for response message type
    typedef const boost::shared_ptr<const gap::msgs::DRResponse>
        DRResponsePtr;
    /// Declaration for model command message type
    typedef gap::msgs::ModelCmd ModelCmdMsg;
    /// Shared pointer declaration for model command message type
    typedef const boost::shared_ptr<const gap::msgs::ModelCmd>
        ModelCmdPtr;
    
    // Forward declaration of private data class
    class DRPluginPrivate;

    /// \brief Multi-purpose domain randomization plugin
    ///
    /// See the example usage below:
    ///
    /// \code{.xml}
    ///    <plugin name="domain_randomization_plugin" filename="libDRPlugin.so">
    ///       <request_topic>~/gap/dr</request_topic>
    ///       <response_topic>~/gap/dr/response</response_topic>
    ///    </plugin>
    /// \endcode
    ///
    /// See worlds/domain_randomization.world for a complete example.
    class DRPlugin : public WorldPlugin
    {
        /// Default topic for DRPlugin requests
        public: static const char REQUEST_TOPIC[];
        /// Default topic for DRPlugin responses
        public: static const char RESPONSE_TOPIC[];
        /// Position controller type
        public: static const int POSITION;
        /// Velocity controller type
        public: static const int VELOCITY;

        /// SDF tag for topic for DRPlugin requests
        public: static const char PARAM_REQ_TOPIC[];
        /// SDF tag for topic for DRPlugin responses
        public: static const char PARAM_RES_TOPIC[];

        // Private attributes

        /// Topic for DRPlugin requests
        private: std::string req_topic {REQUEST_TOPIC};
        /// Topic for DRPlugin responses
        private: std::string res_topic {RESPONSE_TOPIC};

        /// Class with private attributes
        private: std::unique_ptr<DRPluginPrivate> data_ptr;
        /// Connection to world update event
        private: event::ConnectionPtr update_connection;

        /// World to which the plugin is attached
        private: physics::WorldPtr world;
        /// Physics engine pointer
        private: physics::PhysicsEnginePtr physics_engine;

        /// Pending request
        private: boost::shared_ptr<DRRequest const> msg;
        /// Pending feedback
        private: bool feedback_pending;


        // Public methods

        /// \brief Constructs the object
        public: DRPlugin();

        /// \brief Destroys the object
        public: virtual ~DRPlugin();

        /// \brief Loads the plugin
        /// \param _world The world pointer
        /// \param _sdf   The sdf element pointer
        public: virtual void Load(
            physics::WorldPtr _world, sdf::ElementPtr _sdf);

        /// \brief Loads the topic names from SDF
        /// \param _sdf   The sdf element pointer
        private: void loadTopicNames(sdf::ElementPtr _sdf);

        /// \brief Called on World Update event
        public: void onUpdate();

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(DRRequestPtr & _msg);

        // Physics 

        /// \brief Processes physics message
        /// \param msg Physics message
        private: void processPhysics(const msgs::Physics & msg);

        /// \brief Processes model message
        /// \param msg Model message
        private: void processModel(const msgs::Model & msg);

        /// \brief Updates joint
        /// \param model Parent model pointer
        /// \param msg Joint message
        private: void processJoint(
            physics::ModelPtr model,
            const msgs::Joint & msg);

        /// \brief Updates link
        /// \param model Parent model pointer
        /// \param msg Link message
        private: void processLink(
            physics::ModelPtr model,
            const msgs::Link & msg);

        /// \brief Updates inertial
        /// \param link Parent link pointer
        /// \param msg Inertial message
        private: void processInertial(
            physics::LinkPtr link,
            const msgs::Inertial & msg);

        /// \brief Updates surface
        /// \param collision Parent collision pointer
        /// \param msg Surface message
        private: void processSurface(
            physics::CollisionPtr collision,
            const msgs::Surface & msg);

        /// \brief Processes model command message
        /// \param msg Joint command message pointer
        private: void processModelCmd(
            const ModelCmdMsg & msg);

        /// \brief Processes joint command message
        /// \param model Parent model pointer
        /// \param msg Joint command message pointer
        private: void processJointCmd(
            physics::ModelPtr model,
            const msgs::JointCmd & msg);
        
        /// \brief Updates PID controller
        /// \param type PID controller type (POSITION or VELOCITY)
        /// \param controller Joint controller pointer
        /// \param joint Target joint scoped name, e.g. <model>::<joint>
        /// \param msg PID message pointer
        private: void processPID(
            int type,
            physics::JointControllerPtr controller,
            const std::string & joint,
            const msgs::PID & msg);

    };
}

#endif
