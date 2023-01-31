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
    \file visual_utils/VisualUtils.hh
    \brief Visual Utils plugin

    A custom gazebo plugin that provides an interface to programatically
    change the visual properties of an object.

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include <gazebo/common/Events.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/SystemPaths.hh"
#include <gazebo/msgs/msgs.hh>
#include "gazebo/rendering/RenderEngine.hh"
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>

// Boost - for convenient string split
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
// Mutex
#include <mutex>
// Shuffle vector
#include <algorithm>
#include <random>
#include <chrono>

// Custom messages
#include "visual_utils_request.pb.h"
#include "visual_utils_response.pb.h"

namespace VisualUtils {

/// Topic monitored for incoming commands
#define REQUEST_TOPIC   "~/gap/visual_utils"
/// Topic for publishing replies
#define RESPONSE_TOPIC  "~/gap/visual_utils/response"

/// Request update
#define UPDATE          gap::msgs::VisualUtilsRequest::UPDATE
/// Set default pose
#define DEFAULT_POSE    gap::msgs::VisualUtilsRequest::DEFAULT_POSE
/// TODO
#define MATERIAL        gap::msgs::VisualUtilsRequest::MATERIAL_PREFIX

/// Visual updated response
#define UPDATED         gap::msgs::VisualUtilsResponse::UPDATED

// Default parameters

/// Default unique name
#define DEFAULT_NAME    "default"

}

namespace gazebo{

    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const gap::msgs::VisualUtilsRequest>
        VisualUtilsRequestPtr;
    /// Shared pointer declaration for response message type
    typedef const boost::shared_ptr<const gap::msgs::VisualUtilsResponse>
        VisualUtilsResponsePtr;

    // Forward declaration of private data class
    class VisualUtilsPrivate;

    /// \brief A custom gazebo plugin that provides an interface to programatically
    /// alter visuals during simulation.
    ///
    /// Materials are assumed to be loaded and name [pattern][index]
    /// See the example usage below:
    ///
    /// \code{.xml}
    ///    <plugin name="visual_utils" filename="libVisualUtils.so">
    ///     <!-- Unique name identifier -->
    ///     <uid>box_1</uid>
    ///     <!-- Prefix patterns for material names, separated by whitespace -->
    ///     <patterns>Plugin/flat_ Plugin/gradient_ ... </patterns>
    ///     <!-- Number of variants per prefix pattern -->
    ///     <variants>100</variants>
    ///    </plugin>
    /// \endcode
    ///
    /// See worlds/visual.world for a complete example.
    class VisualUtils : public VisualPlugin {

        /// \brief Constructs the object
        public: VisualUtils();

        /// \brief Destroys the object
        public: virtual ~VisualUtils();

        /// \brief Loads the plugin
        /// \param _visual  The visual to which the plugin is attached
        /// \param _sdf     The SDF element with plugin parameters
        public: virtual void Load(
            rendering::VisualPtr _visual,
            sdf::ElementPtr _sdf);

        /// \brief Update once per simulation iteration.
        public: void Update();

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        public: void onRequest(VisualUtilsRequestPtr & _msg);

        /// \brief Private data pointer
        private: std::unique_ptr<VisualUtilsPrivate> dataPtr;

        /// \brief Loads names of available materials.
        private: void loadResources();

        /// \brief Randomly generates a new material name.
        /// \param name Output random material name
        private: void randomMaterialName(std::string & name);
    };
}
