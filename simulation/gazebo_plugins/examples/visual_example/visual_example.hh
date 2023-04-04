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
    \file examples/visual_example/visual_example.hh
    \brief Visual tools example client

    An example client to interact with VisualUtils plugin
    
    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

// I/O streams
#include <iostream>
// For sleeps
#include <chrono>
#include <thread>

// Custom messages
#include "visual_utils_request.pb.h"

/// Topic monitored by the server for incoming commands
#define VISUAL_UTILS_TOPIC "~/gap/visual_utils"

/// Request update
#define UPDATE      gap::msgs::VisualUtilsRequest::UPDATE
