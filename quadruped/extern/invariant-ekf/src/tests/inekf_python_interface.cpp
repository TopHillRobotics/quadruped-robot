/************************************************************************
Copyright (c) 2021, Huawei.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Written by Yijie zhu @10.20
************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
// #include <boost/algorithm/string.hpp>
#include <array>
#include <math.h>
#include <Eigen/Dense>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
// #include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "InEKF.h"
#include "se3.h"
#define DT_MIN 1e-6
#define DT_MAX 1

double stod98(const std::string &s) {
    return atof(s.c_str());
}

int stoi98(const std::string &s) {
    return atoi(s.c_str());
}

using namespace std;
using namespace UNITREE_LEGGED_SDK;
using namespace inekf;


class INEKFInterface
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    INEKFInterface(): t(0.0d), t_prev(0.0d){
        // InitEnvironment();
        // Initialize();
    }
    void Initialize(Eigen::Matrix3d R0, Eigen::Vector3d v0, Eigen::Vector3d bg0, Eigen::Vector3d ba0);
    void ReceiveMotorAngleData(double T, std::array<float, 12>& motor_angles);
    void ReceiveKinematicsMeasurement(const pybind11::dict& dictionary);
    void ReceiveImuDataInSim(double T, const pybind11::dict& dictionary);
    void ReceiveImuData(double T, const IMU& imu);
    void ReceiveLandmarkData(double T, mapIntVector3d& prior_landmarks){ ;};
    RobotState Update_1();
    RobotState Update_2();
    void UpdateContact(std::array<bool, 4> is_contacts);

    const Eigen::MatrixXd getRobotStateX();
    const Eigen::VectorXd getRobotStateTheta();
    const Eigen::MatrixXd getRobotStateP();
    void printState();

private:
    long long cycle=0;
    double t;
    double t_prev;
    IMU imu;
    vector<string> measurement;
    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    RobotState initial_state;
    NoiseParams noise_params;
    InEKF filter;
    Eigen::Matrix<double,6,1> imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double, 4, 1> IMUquat; //  = Eigen::Matrix<double,3,1>::Zero();
    // --- Optionally initialize prior landmarks --- //
    // mapIntVector3d prior_landmarks;
    // Eigen::Vector3d p_wl;
    // int id;
    // cout << "Received KINEMATIC observation, correcting state\n";  
    std::vector<Eigen::Quaternion<double>> q;  // R_VectorNav_to_FRToeBottom, R_VectorNav_to_FLToeBottom, R_VectorNav_to_BRToeBottom, R_VectorNav_to_BLToeBottom
    std::vector<Eigen::Vector3d> p;   // p_VectorNav_to_FRToeBottom, ...
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance;
    vectorKinematics measured_kinematics;

};
void INEKFInterface::printState()
{
    cout << "Robot's state is: \n";
    cout << filter.getState() << endl;
}
const Eigen::MatrixXd INEKFInterface::getRobotStateX()
{
    return filter.getState().getX();
}

const Eigen::VectorXd INEKFInterface::getRobotStateTheta(){
    return filter.getState().getTheta();
}
const Eigen::MatrixXd INEKFInterface::getRobotStateP(){
    return filter.getState().getP();
}

void INEKFInterface::Initialize(Eigen::Matrix3d _R0, Eigen::Vector3d _v0, 
                                Eigen::Vector3d _bg0, Eigen::Vector3d _ba0){
    // R0 << 1, 0, 0, // initial orientation
    //       0, -1, 0, // IMU frame is rotated 90deg about the x-axis
    //       0, 0, -1;
    // R0 << 1, 0, 0, // initial orientation
    //       0, 1, 0, 
    //       0, 0, 1;
    // v0 << 0,0,0; // initial velocity
    p0 << 0,0,2.4; // initial position
    // bg0 << 0,0,0; // initial gyroscope bias
    // ba0 << 0,0,0; // initial accelerometer bias
    R0 = _R0;
    v0 = _v0;
    bg0 = _bg0;
    ba0 = _ba0;
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);
    
    cout << "bg0 = "<<bg0<<endl;
    cout << "ba0 = "<<ba0<<endl; 
    // Initialize state covariance
    noise_params.setGyroscopeNoise(0.0001);
    noise_params.setAccelerometerNoise(0.01);
    noise_params.setGyroscopeBiasNoise(0.000001);
    noise_params.setAccelerometerBiasNoise(0.00001);
    noise_params.setContactNoise(0.001);

    // Initialize invariant extended Kalman filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);
    cout << "Noise parameters are initialized to: \n";
    cout << filter.getNoiseParams() << endl;
    cout << "Robot's state is initialized to: \n";
    cout << filter.getState() << endl;
 
    // // Landmark 1
    // id = 1;
    // p_wl << 0,-1,0;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // // // Landmark 2
    // // id = 2;
    // // p_wl << 1,1,-0.5;
    // // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // // Landmark 3
    // id = 3;
    // p_wl << 2,-1,0.5;
    // prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    // // Store landmarks for localization
    // filter.setPriorLandmarks(prior_landmarks); 

}


void INEKFInterface::ReceiveMotorAngleData(double T, std::array<float, 12>& motor_angles) {
    t = T;
}

void INEKFInterface::ReceiveImuDataInSim(double T, const pybind11::dict& dictionary) {
    // # std::array<float, 4> quaternion;               // quaternion, normalized, (w,x,y,z)
    // # std::array<float, 3> gyroscope;                // angular velocity （unit: rad/s)
    // # std::array<float, 3> accelerometer;            // m/(s2)
    // # std::array<float, 3> rpy;                      // euler angle（unit: rad)
    // # int8_t temperature;
    t = T;
    std::array<double, 3> gyroscope;     // angular velocity （unit: rad/s)
    std::array<double, 3> accelerometer;
    for (std::pair<pybind11::handle, pybind11::handle> item : dictionary) {
        string key = item.first.cast<std::string>();
        if (key=="gyroscope") {
            gyroscope = item.second.cast<std::array<double, 3>>();
        }
        else if(key == "accelerometer") {
            accelerometer = item.second.cast<std::array<double, 3>>();
        }
        
    }
    imu_measurement << gyroscope[0], 
                       gyroscope[1], 
                       gyroscope[2],
                       accelerometer[0],
                       accelerometer[1],
                       accelerometer[2];
    // cout<<"t="<<t<<endl;

}

void INEKFInterface::ReceiveKinematicsMeasurement(const pybind11::dict& dictionary) {
    measured_kinematics.clear();
    q.clear();
    p.clear();
    std::vector<Eigen::Matrix3d> Jacobians;
    // cout<<"Kinematics:"<<endl;
    for (std::pair<pybind11::handle, pybind11::handle> item : dictionary) {
        string key = item.first.cast<std::string>();
        // cout<< key <<endl;
        if (key=="q") {
            // q = item.second.cast<std::vector<Eigen::Quaterniond>>(); // error
            auto q_ = item.second.cast<std::array<Eigen::Matrix<double, 4,1>, 4>>();
            
            for (auto& q_i : q_){
                q.push_back(Eigen::Quaterniond(q_i));
                // cout<<q_i<<endl;
            }
        }
        else if(key == "p") {
            auto p_ = item.second.cast<std::array<Eigen::Vector3d, 4>>();
            for (auto& pp : p_){
                p.push_back(pp);
                // cout<< pp <<endl;
            }
        }
        else if (key == "J") { //  analysis jacobian for each leg
            auto Jacobians_ = item.second.cast<std::array<Eigen::Matrix3d, 4>>();
            for (auto& jj :Jacobians_){
                // cout<<jj<<endl;
                Jacobians.push_back(jj);
            }
        }
        else{
            printf("no valid key !!!!!!!!!!!!!!!!!") ;
        }
    }
    Eigen::Matrix3d Cov_e = Eigen::Matrix3d::Identity()*0.0001;
    // cout<<"cov 0 ========="<<endl;
    for (int i=0; i<4; ++i) {
        // q[i].normalize();  // error
        pose.block<3,3>(0,0) = q[i].toRotationMatrix();
        pose.block<3,1>(0,3) = p[i];
        Eigen::Matrix<double, 6,3> J = Eigen::Matrix<double,6,3>::Zero();
        J.block<3,3>(3,0) = Jacobians[i];
        covariance =  J*Cov_e*J.transpose(); // --> 6*6 matrix
        measured_kinematics.push_back(Kinematics(i, pose, covariance));
    }
}

void INEKFInterface::ReceiveImuData(double T, const IMU& imu) {
    // cout << "Received IMU Data, propagating state\n";
    // assert((measurement.size()-2) == 6);
    t = T;
    // Angular Velocity (w), Linear Acceleration (a)
    imu_measurement << (double)imu.gyroscope[0], 
                       (double)imu.gyroscope[1], 
                       (double)imu.gyroscope[2],
                       (double)imu.accelerometer[0],
                       (double)imu.accelerometer[1],
                       (double)imu.accelerometer[2];
    // std::cout << imu_measurement <<std::endl;
    IMUquat << (double)imu.quaternion[0], (double)imu.quaternion[1], (double)imu.quaternion[2],(double)imu.quaternion[3];
    // std::cout <<IMUrpy <<std::endl;
}

void INEKFInterface::UpdateContact(std::array<bool, 4> is_contacts){
    // cout << "Received CONTACT Data, setting filter's contact state\n";
    vector<pair<int,bool> > contacts;
    int id;
    bool indicator;
    // Read in contact data
    for (int i=0; i<4; ++i) {
        id = i;
        indicator = is_contacts[i];
        contacts.push_back(pair<int,bool> (id, indicator));
    }       
    // Set filter's contact state
    filter.setContacts(contacts);
}


RobotState INEKFInterface::Update_1(){
    // Propagate
    // Propagate using IMU data
    double dt = t - t_prev;
    if (dt > DT_MIN && dt < DT_MAX) {
        filter.Propagate(imu_measurement_prev, dt);
    }
    Eigen::Matrix3d oritationMatrix = robotics::math::quaternionToRotationMatrix(IMUquat);
    if (cycle%5==0)
        filter.setOrientation(oritationMatrix);
    cycle++;
    // Store previous timestamp
    t_prev = t;
    imu_measurement_prev = imu_measurement;
    // Print Propagated state
    const RobotState& state = filter.getState();
    // cout << state << endl;
    return state;
}

RobotState INEKFInterface::Update_2(){
    // Correct
    filter.CorrectKinematics(measured_kinematics);
    // Print final state
    const RobotState& state = filter.getState();
    // cout << state << endl;
    return state;
}

namespace py = pybind11;
// PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Quaterniond>)
// PYBIND11_MAKE_OPAQUE(std::vector<Eigen::MatrixXd>)
// TODO: Expose all of comm.h and the inekf_interface Class.
PYBIND11_MODULE(inekf_interface, m) {
    m.doc() = R"pbdoc(
          A1 Robot Interface Python Bindings
          -----------------------
          .. currentmodule:: a1_robot_interface
          .. autosummary::
             :toctree: _generate
      )pbdoc";
    // py::bind_vector<std::vector<Eigen::Quaterniond>>(m, "quatVector");
    // py::bind_vector<std::vector<Eigen::MatrixXd>>(m, "matVector");
    py::class_<RobotState>(m, "RobotState")
        .def(py::init<>())
        .def("getP", &RobotState::getP)
        .def("getTheta", &RobotState::getTheta)
        .def("getX", &RobotState::getX)
        .def("getRotation", &RobotState::getRotation)
        .def("getVelocity", &RobotState::getVelocity)
        .def("getPosition", &RobotState::getPosition)
        .def("getGyroscopeBias", &RobotState::getGyroscopeBias)
        .def("getAccelerometerBias", &RobotState::getAccelerometerBias);        
    
    py::class_<INEKFInterface>(m, "INEKFInterface")
        .def(py::init<>())
        .def("initialize", &INEKFInterface::Initialize)
        .def("receive_motor_angle_data", &INEKFInterface::ReceiveMotorAngleData)
        .def("receive_kinematics_measurement", &INEKFInterface::ReceiveKinematicsMeasurement)
        .def("receive_imu_data_in_sim", &INEKFInterface::ReceiveImuDataInSim)
        .def("receive_imu_data", &INEKFInterface::ReceiveImuData)
        // .def("receive_imu_data_in_sim", py::overload_cast<double, const py::dict&>(&INEKFInterface::ReceiveImuDataInSim))
        // .def("receive_imu_data", py::overload_cast<double, const IMU&>(&INEKFInterface::ReceiveImuData))
        .def("update_1", &INEKFInterface::Update_1)
        .def("update_2", &INEKFInterface::Update_2)
        .def("update_contact", &INEKFInterface::UpdateContact)
        .def("get_robotstate_X", &INEKFInterface::getRobotStateX)
        .def("get_robotstate_Theta", &INEKFInterface::getRobotStateTheta)
        .def("get_robotstate_P", &INEKFInterface::getRobotStateP)
        .def("print_state", &INEKFInterface::printState);

        

    

    #ifdef VERSION_INFO
      m.attr("__version__") = VERSION_INFO;
    #else
      m.attr("__version__") = "dev";
    #endif
      m.attr("TEST") = py::int_(int(42));

}
