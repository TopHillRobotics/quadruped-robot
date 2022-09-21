#include "state_estimator/qr_odom_estimator.h"

qrRobotOdometryEstimator::qrRobotOdometryEstimator(qrRobot *robotIn,
                                               ros::NodeHandle &nhIn)
    : robot(robotIn), nh(nhIn)
{
    pubOdometry = nh.advertise<nav_msgs::Odometry>("legOdom", 1);
    lastTime = ros::Time::now();
    odomEstimateX = robot->GetBasePosition()[0];
    odomEstimateY = robot->GetBasePosition()[1];
    ROS_INFO("robot_odom_estimator init success...");
}

void qrRobotOdometryEstimator::PublishOdometry()
{
    ros::spinOnce();
    currentTime = ros::Time::now();
    // ControlDataFlow* controlDataFlow = robot->controlDataFlow;
    const Vec3<float> estimatedVelocity = robot->GetBaseVelocity();
    const Vec3<float> &baserpy = robot->GetBaseRollPitchYaw();
    const Vec3<float> &baseRollPitchYawRate = robot->GetBaseRollPitchYawRate();
    float vX = estimatedVelocity[0];
    float vY = estimatedVelocity[1];

    double dt = (currentTime - lastTime).toSec();
    double dx, dy;
    dx = (vX * cos(baserpy[2]) - vY * sin(baserpy[2])) * dt;
    dy = (vX * sin(baserpy[2]) + vY * cos(baserpy[2])) * dt;
    odomEstimateX += dx;
    odomEstimateY += dy;
    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(baserpy[2]);

    // publish the transform over tf
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = currentTime;
    odomTrans.header.frame_id = "odom";
    odomTrans.child_frame_id = "base";
    odomTrans.transform.translation.x = odomEstimateX;
    odomTrans.transform.translation.y = odomEstimateY;
    odomTrans.transform.translation.z = 0.0;
    odomTrans.transform.rotation = odomQuat;
    
    // send the transform
    odomBroadcaster.sendTransform(odomTrans);

    // publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = odomEstimateX;
    odom.pose.pose.position.y = odomEstimateY;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuat;
    //set the velocity
    odom.child_frame_id = "base";
    odom.twist.twist.linear.x = vX;
    odom.twist.twist.linear.y = vY;
    odom.twist.twist.angular.z = baseRollPitchYawRate[2];

    //publish the message
    pubOdometry.publish(odom);
    lastTime = currentTime;
}
