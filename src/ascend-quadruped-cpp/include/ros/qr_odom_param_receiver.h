// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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


#ifndef QR_ODOM_PARAM_RECEIVER_H_
#define QR_ODOM_PARAM_RECEIVER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "mpc_controller/locomotion_controller.h"

namespace Quadruped {
    /**
     * @brief send odometry of body in world frame.
     */    
    class qrOdometryParamReceiver {
    public:
        qrOdometryParamReceiver(Robot *robotIn, LocomotionController *locomotionController, ros::NodeHandle &nhIn);

        void PublishOdometry();
        
    private:
        Robot *robot;
        RobotEstimator *robotEstimator;
        LocomotionController *locomotionController;

        ros::NodeHandle &nh;
        ros::Publisher pubOdometry;
        // broadcast odom to base_link
        tf::TransformBroadcaster odomBroadcaster;
        ros::Time currentTime;
        ros::Time lastTime;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_ROS_ROBOT_ODOM_ESTIMATOR_H_
