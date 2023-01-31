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
    \file camera_utils/CameraUtils.hh
    \brief Camera Utils plugin

    A custom gazebo plugin that provides an interface to programatically collect
    data from cameras at specific times.

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include "gazebo/common/Plugin.hh"
#include <gazebo/msgs/msgs.hh>
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
// Custom messages
#include "camera_utils_request.pb.h"
#include "camera_utils_response.pb.h"
// Strings
#include <string>
// To create directories
#include <boost/filesystem.hpp>
// Malloc and memcpy
#include <stdio.h>
#include <string.h>

namespace CameraUtils {

/// Topic monitored for incoming commands
#define REQUEST_TOPIC   "~/gap/camera_utils"
/// Topic for replying to commands
#define RESPONSE_TOPIC  "~/gap/camera_utils/response"

/// Request to move camera to given pose
#define MOVE_REQUEST        gap::msgs::CameraUtilsRequest::MOVE
/// Response acknowledging move camera request
#define MOVE_RESPONSE       gap::msgs::CameraUtilsResponse::MOVE
/// Request camera capture
#define CAPTURE_REQUEST     gap::msgs::CameraUtilsRequest::CAPTURE
/// Camera capture response
#define CAPTURE_RESPONSE    gap::msgs::CameraUtilsResponse::CAPTURE
/// Request change image storage directory
#define OPTIONS_REQUEST     gap::msgs::CameraUtilsRequest::OPTIONS
/// Request 3D to 2D point projection
#define PROJECTION_REQUEST  gap::msgs::CameraUtilsRequest::PROJECTION
/// Point projection response
#define PROJECTION_RESPONSE gap::msgs::CameraUtilsResponse::PROJECTION

// Default parameters

/// Default output directory
#define DEFAULT_OUTPUT_DIR  (const std::string) "/tmp/camera_utils_output/"
/// Default captured images extension
#define DEFAULT_EXTENSION   (const std::string) ".png"

}

namespace gazebo{

    /// Shared pointer declaration for request message type
    typedef const boost::shared_ptr<const gap::msgs::CameraUtilsRequest>
        CameraUtilsRequestPtr;
    /// Shared pointer declaration for response message type
    typedef const boost::shared_ptr<const gap::msgs::CameraUtilsResponse>
        CameraUtilsReplyPtr;

    // Forward declaration of private data class
    class CameraUtilsPrivate;

    /// \brief A custom gazebo plugin that provides an interface to programatically
    /// collect data from cameras at specific times.
    ///
    /// See the example usage below:
    /// \code{.xml}
    ///    <plugin name="camera_utils" filename="libCameraUtils.so">
    ///
    ///      <!-- Output image directory -->
    ///      <output_dir>/tmp/camera_world</output_dir>
    ///
    ///      <!-- Output image extension -->
    ///      <extension>.png</extension>
    ///
    ///    </plugin>
    /// \endcode
    ///
    /// See worlds/camera.world for a complete example.
    class CameraUtils : public SensorPlugin {

        // Private attributes

        /// Class with private attributes
        private: std::unique_ptr<CameraUtilsPrivate> dataPtr;
        /// Directory for saving output
        private: std::string output_dir;
        /// Saved frames counter
        private: int saved_counter {0};
        /// File name for next capture
        private: std::string next_file_name;
        /// Internal flag for saving on next update
        private: bool save_on_update {false};
        /// Connects to new frame rendered event
        private: event::ConnectionPtr newFrameConnection;

        // Protected attributes

        /// Pointer to camera sensor
        protected: sensors::CameraSensorPtr parentSensor;
        /// Pointer to camera object
        protected: rendering::CameraPtr camera;
        /// Image width
        protected: unsigned int width;
        /// Image height
        protected: unsigned int height;
        /// Image depth
        protected: unsigned int depth;
        /// Image format
        protected: std::string format;
        /// Exported image extension
        protected: std::string extension;

        // Public methods

        /// \brief Constructs the object
        public: CameraUtils();

        /// \brief Destroys the object
        public: virtual ~CameraUtils();

        /// \brief Loads the object
        /// \param _sensor  The camera sensor to which the plugin is attached
        /// \param _sdf     The SDF element with plugin parameters
        public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

        /// \brief Callback function for handling frame updates
        /// \param _image   Image data
        /// \param _width   Image width
        /// \param _height  Image height
        /// \param _depth   Image depth
        /// \param _format  Image format
        public: void OnNewFrame(const unsigned char *_image,
            unsigned int _width, unsigned int _height,
            unsigned int _depth, const std::string &_format);

        // Private methods

        /// \brief Callback function for handling incoming requests
        /// \param _msg  The message
        private: void onRequest(CameraUtilsRequestPtr &_msg);
    };
}
