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
    \file examples/camera_example/camera_example.cc
    \brief Camera tools example client implementation

    An example client to interact with CameraUtils plugin

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "camera_example.hh"

using namespace gap::msgs;

int main(int _argc, char **_argv)
{

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the object spawner topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<CameraUtilsRequest>(CAMERA_UTILS_TOPIC);

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Create a custom message to change camera options
    CameraUtilsRequest msg;
    msg.set_type(OPTIONS);
    msg.set_output_dir("/tmp/");
    msg.set_extension(".jpg");
    // Send the message
    pub->Publish(msg);

    // Create a custom message to aqcuire a frame
    msg.set_type(CAPTURE);
    msg.set_file_name("test");
    // Send the message
    pub->Publish(msg);

    // Shut down
    gazebo::client::shutdown();
    return 0;
}
