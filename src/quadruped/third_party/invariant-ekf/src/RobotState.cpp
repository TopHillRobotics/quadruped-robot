/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   qrRobotState.h
 *  @author Ross Hartley
 *  @brief  Source file for qrRobotState (thread-safe)
 *  @date   September 25, 2018
 **/

#include "qrRobotState.h"
#include "LieGroup.h"

namespace inekf {

using namespace std;

// Default constructor
qrRobotState::qrRobotState() :
    X_(Eigen::MatrixXd::Identity(5,5)), Theta_(Eigen::MatrixXd::Zero(6,1)), P_(Eigen::MatrixXd::Identity(15,15)) 
{
    cout << "creating a robot state 1 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
    // P_.block<3,3>(0,0) = 0.03*Eigen::Matrix3d::Identity();
    // P_.block<3,3>(3,3) = 0.0001*Eigen::Matrix3d::Identity();
    // P_.block<3,3>(6,6) = 0.00001*Eigen::Matrix3d::Identity();
    // P_.block<3,3>(9,9) = 0.0001*Eigen::Matrix3d::Identity();
    // P_.block<3,3>(12,12) = 0.0025*Eigen::Matrix3d::Identity();
    
    // for(int i=6; i<=8;++i){
    //     for(int j=i;j<=i+6; j=j+3){
    //         if(i!=j)
    //             P_(i, j) = 0.1;
    //     }
    // }
    // for(int i=9; i<=11; ++i){
    //     for(int j=i-3;j<=i-3+6; j=j+3){
    //         if(i!=j)
    //             P_(i, j) = 0.1;
    //     }
    // }
    // for(int i=12; i<=14;++i){
    //     for(int j=i-6;j<=i-6+6; j=j+3){
    //         if(i!=j)
    //             P_(i, j) = 0.1;
    //     }
    // }
}
// Initialize with Xs
qrRobotState::qrRobotState(const Eigen::MatrixXd& X) : 
    X_(X), Theta_(Eigen::MatrixXd::Zero(6,1)) 
{
    // cout << "creating a robot state" << endl;
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
    cout << "creating a robot state 2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
    P_ = Eigen::MatrixXd::Zero(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
    //Zero(15,15);
    P_(3, 3) = 1.f;
    for(int i=7; i<=9;++i){
        for(int j=i;j<=i+6; j=j+3){
            P_(i, j) = 1.f;
        }
    }
    for(int i=10; i<=12; ++i){
        for(int j=i-3;j<=i-3+6; j=j+3){
            P_(i, j) = 1.f;
        }
    }
    if (this->dimX()>=5) {
        for(int i=13; i<=15;++i){
            for(int j=i-6;j<=i-6+6; j=j+3){
                P_(i, j) = 1.f;
            }
        }
    }
}
// Initialize with X and Theta
qrRobotState::qrRobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta) : 
    X_(X), Theta_(Theta) {
    P_ = Eigen::MatrixXd::Identity(3*this->dimX()+this->dimTheta()-6, 3*this->dimX()+this->dimTheta()-6);
}
// Initialize with X, Theta and P
qrRobotState::qrRobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta, const Eigen::MatrixXd& P) : 
    X_(X), Theta_(Theta), P_(P) {}
// TODO: error checking to make sure dimensions are correct and supported


// // Move initialization
// qrRobotState::qrRobotState(qrRobotState&& other) {
//     lock_guard<mutex> lock(other.mutex_);
//     X_ = std::move(other.X_);
//     other.X_ = Eigen::MatrixXd;
// }

#if INEKF_USE_MUTEX
// Copy initialization
qrRobotState::qrRobotState(const qrRobotState& other) {
    lock_guard<mutex> other_lock(other.mutex_);
    X_ = other.X_;
    Theta_ = other.Theta_;
    P_ = other.P_;
}

// // Move assignment
// qrRobotState::qrRobotState& operator = (qrRobotState&& other) {
//     std::lock(mtx, other.mtx);
//     std::lock_guard<std::mutex> self_lock(mtx, std::adopt_lock);
//     std::lock_guard<std::mutex> other_lock(other.mtx, std::adopt_lock);
//     value = std::move(other.value);
//     other.value = 0;
//     return *this;
// }

// Copy assignment
qrRobotState& qrRobotState::operator = (const qrRobotState& other) {
    lock(mutex_, other.mutex_);
    lock_guard<mutex> self_lock(mutex_, adopt_lock);
    lock_guard<mutex> other_lock(other.mutex_, adopt_lock);
    X_ = other.X_;
    Theta_ = other.Theta_;
    P_ = other.P_;
    return *this;
}
#endif


const Eigen::MatrixXd qrRobotState::getX() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_; 
}
const Eigen::VectorXd qrRobotState::getTheta() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_; 
}
const Eigen::MatrixXd qrRobotState::getP() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return P_; 
}
const Eigen::Matrix3d qrRobotState::getRotation() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,3>(0,0); 
}
const Eigen::Vector3d qrRobotState::getVelocity() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,1>(0,3); 
}
const Eigen::Vector3d qrRobotState::getPosition() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.block<3,1>(0,4); 
}
const Eigen::Vector3d qrRobotState::getGyroscopeBias() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.head(3); 
}
const Eigen::Vector3d qrRobotState::getAccelerometerBias() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.tail(3); 
}
const int qrRobotState::dimX() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return X_.cols(); 
}
const int qrRobotState::dimTheta() {
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return Theta_.rows();
}
const int qrRobotState::dimP() { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    return P_.cols(); 
}

void qrRobotState::setX(const Eigen::MatrixXd& X) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_ = X; 
}
void qrRobotState::setTheta(const Eigen::VectorXd& Theta) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_ = Theta; 
}
void qrRobotState::setP(const Eigen::MatrixXd& P) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    P_ = P; 
}
void qrRobotState::setRotation(const Eigen::Matrix3d& R) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,3>(0,0) = R; 
}
void qrRobotState::setVelocity(const Eigen::Vector3d& v) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,1>(0,3) = v; 
}
void qrRobotState::setPosition(const Eigen::Vector3d& p) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    X_.block<3,1>(0,4) = p; 
}
void qrRobotState::setGyroscopeBias(const Eigen::Vector3d& bg) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_.head(3) = bg; 
}
void qrRobotState::setAccelerometerBias(const Eigen::Vector3d& ba) { 
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(mutex_);
#endif
    Theta_.tail(3) = ba; 
}


void qrRobotState::copyDiagX(int n, Eigen::MatrixXd& BigX) {
    int dimX = this->dimX();
    for(int i=0; i<n; ++i) {
        int startIndex = BigX.rows();
        BigX.conservativeResize(startIndex + dimX, startIndex + dimX);
        BigX.block(startIndex,0,dimX,startIndex) = Eigen::MatrixXd::Zero(dimX,startIndex);
        BigX.block(0,startIndex,startIndex,dimX) = Eigen::MatrixXd::Zero(startIndex,dimX);
#if INEKF_USE_MUTEX
        unique_lock<mutex> mlock(mutex_);
#endif
        BigX.block(startIndex,startIndex,dimX,dimX) = X_;
    }
    return;
}

ostream& operator<<(ostream& os, const qrRobotState& s) {  
#if INEKF_USE_MUTEX
    unique_lock<mutex> mlock(s.mutex_);
#endif
    os << "--------- qrRobot State -------------" << endl;
    os << "X:\n" << s.X_ << endl << endl;
    os << "Theta:\n" << s.Theta_ << endl << endl;
    os << "P:\n" << s.P_ << endl;
    os << "-----------------------------------";
    return os;  
}

} // end inekf namespace
