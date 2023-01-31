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
    \file examples/visual_example/visual_example.cc
    \brief Visual tools example client implementation

    An example client to interact with VisualUtils plugin

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "visual_example.hh"

using namespace gap::msgs;

int main(int argc, char **argv)
{
    // Setup communication

    // Setup Gazebo client
    gazebo::client::setup(argc, argv);
    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    // Publish to the visual plugin topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<VisualUtilsRequest>(VISUAL_UTILS_TOPIC);
    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    std::vector<std::string> targets = {"box","sphere","cylinder","ground"};

    // Main loop
    for (int i = 0; i < 10; i++)
    {
        // Create and send a custom message
        VisualUtilsRequest msg;
        msg.set_type(UPDATE);
        // Define targets of request
        for (const auto & target : targets) {
            msg.add_targets(target);
        }
        pub->Publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Clean up
    gazebo::client::shutdown();
    return 0;
}
