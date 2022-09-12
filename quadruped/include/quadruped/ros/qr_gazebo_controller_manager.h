// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_GAZEBO_CONTROLLER_MANAGER_H
#define QR_GAZEBO_CONTROLLER_MANAGER_H

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>
#include <chrono>


/**
 * @brief init simulation controllers
 * @param nodeHandle
 * @param serviceName
 */
bool startControllers(ros::NodeHandle &nodeHandle, std::string serviceName);

/**
 * @brief stop simulation controllers
 * @param nodeHandle
 * @param serviceName
 */
bool stopControllers(ros::NodeHandle &nodeHandle, std::string serviceName);

/**
 * @brief restart simultion by system command
 * 
 */
bool ResetRobotBySystem(ros::NodeHandle &nodeHandle, std::string robotName);

/**
 * @brief restart simulation by gazebo-ros service
 * @param modelStateClient
 * @param jointStateClient
 */
bool ResetRobotByService(ros::NodeHandle &nodeHandle,
                         ros::ServiceClient &modelStateClient, 
                         ros::ServiceClient &jointStateClient, 
                         std::string robotName);


#endif //QR_GAZEBO_CONTROLLER_MANAGER_H