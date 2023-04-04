/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the state of quadruped via INEKF.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/
#ifndef INEKF_CPP_INTERFACE_H
#define INEKF_CPP_INTERFACE_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <array>
#include <math.h>
#include <Eigen/Dense>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "InEKF.h"
#include "utils/qr_se3.h"

namespace inekf {


struct KinematicsMeasurement {
    std::vector<Quat<float>> q;
    std::vector<Eigen::Matrix<float, 3, 1>> p;
    std::vector<Eigen::Matrix<float, 3, 3>> J;
};
struct IMUData {
    std::array<double, 3> gyroscope;          // angular velocity （unit: rad/s)
    std::array<double, 3> accelerometer;      // m/(s2)
    
};

class INEKFInterface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using IMU = UNITREE_LEGGED_SDK::IMU;

    INEKFInterface(): t(0.0), t_prev(0.0)
    {
        std::cout<<"INEKFInterface" <<std::endl;
        // InitEnvironment();
        // Initialize();
    }
    
    void Initialize(Eigen::Matrix3d R0, Eigen::Vector3d v0, Eigen::Vector3d bg0, Eigen::Vector3d ba0);
    
    void ReceiveMotorAngleData(double T, std::array<float, 12>& motor_angles);
    
    void ReceiveKinematicsMeasurement(const KinematicsMeasurement &kinematicsMeasurement);
    
    void ReceiveImuDataInSim(double T, const IMU& imu);
    
    void ReceiveImuData(double T, const IMU& imu);
    
    void ReceiveLandmarkData(double T, mapIntVector3d& prior_landmarks){ ;}
    
    RobotState Update_1(Vec3<double> v);
    
    RobotState Update_2();
    
    void UpdateContact(Eigen::Matrix<bool, 4, 1> isContacts);

    const Eigen::MatrixXd getRobotStateX();
    
    const Eigen::VectorXd getRobotStateTheta();
    
    const Eigen::MatrixXd getRobotStateP();
    
    void printState();

    Vec3<double> getPosition();
    
    Vec3<double> getRotation();

private:
    long long cycle=0;
    double t;
    double dt=0.0;
    double t_prev;
    IMU imu;
    std::vector<std::string> measurement;
    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    RobotState initial_state;
    NoiseParams noise_params;
    InEKF filter;
    Eigen::Matrix<double,6,1> imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double, 4, 1> IMUquat;
    // --- Optionally initialize prior landmarks --- //
    // mapIntVector3d prior_landmarks;
    // Eigen::Vector3d p_wl;
    // int id;
    // cout << "Received KINEMATIC observation, correcting state\n";  
    std::vector<Eigen::Quaternion<double>> q; // R_VectorNav_to_FRToeBottom, R_VectorNav_to_FLToeBottom, R_VectorNav_to_BRToeBottom, R_VectorNav_to_BLToeBottom
    std::vector<Eigen::Vector3d> p; // p_VectorNav_to_FRToeBottom, ...
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance;
    vectorKinematics measured_kinematics;

};
} // inekf

#endif // INEKF_CPP_INTERFACE