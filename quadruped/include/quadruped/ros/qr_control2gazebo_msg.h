// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

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

#ifndef QR_CONTROL2GAZEBO_MSG_H_
#define QR_CONTROL2GAZEBO_MSG_H_

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <tf/transform_broadcaster.h>

#include "xpp_msgs/topic_names.h"
#include "xpp_msgs/TerrainInfo.h"
#include "xpp_msgs/RobotParameters.h"
#include "xpp_msgs/RobotStateCartesian.h"
#include "xpp_states/convert.h"
#include "xpp_states/robot_state_cartesian.h"
#include "controllers/qr_locomotion_controller.h"


namespace Quadruped {

/**
 * @brief Collect messages from gazebo and transpond to visualization module.
 */
class qrController2GazeboMsg {

public:

    qrController2GazeboMsg(qrRobot *robotIn, qrLocomotionController *locomotionController, ros::NodeHandle &nhIn);

    void PublishGazeboStateCallback();

private:

    qrRobot *robot;

    qrLocomotionController *locomotionController;

    qrPosePlanner *posePlanner;

    ros::NodeHandle &nh;

    ros::Publisher gazeboStatePublish;

    ros::Publisher gazeboParamPublish;

    ros::Publisher posePlannerPublish;

    ros::ServiceClient baseStateClient;

    tf::TransformBroadcaster desiredPoseFramebr;

    tf::Transform transform;

    Vec3<float> rIB;

    Quat<float> quat;

    ros::Time currentTime;

    ros::Time lastTime;

};

} // Namespace Quadruped

#endif // QR_CONTROL2GAZEBO_MSG_H_
