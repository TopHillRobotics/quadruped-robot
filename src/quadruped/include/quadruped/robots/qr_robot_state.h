#ifndef QR_ROBOT_STATE_H
#define QR_ROBOT_STATE_H

#include <Eigen/Dense>
#include "utils/se3.h"
#include "qr_robot_config.h"

struct IMU{
    std::array<float, 4> quaternion;               // quaternion, normalized, (w,x,y,z)
    std::array<float, 3> gyroscope;                // angular velocity （unit: rad/s)
    std::array<float, 3> accelerometer;            // m/(s2)
    std::array<float, 3> rpy;                      // euler angle（unit: rad)
};

struct MotorState
{
    float q;                           // current angle (unit: radian)
    float dq;                          // current velocity (unit: radian/second)
    float tauEst;                      // current estimated output torque (unit: N.m)
    void operator=(const MotorState &state){
        this->q = state.q;
        this->dq = state.dq;
        this->tauEst = state.tauEst;
    }
};

class RobotState{
public:

    RobotState(RobotConfig*& config);

    Eigen::Matrix<float, 3, 1> basePosition; //robot base position in world frame
    Eigen::Matrix<float, 4, 1> baseOrientation; //robot base orientation in world frame
    Eigen::Matrix<float, 3, 1> baseRollPitchYaw; //yaw calibrated robot rpy in world frame
    Eigen::Matrix<float, 3, 1> baseRollPitchYawRate; //robot rpy rate in base frame
    Eigen::Matrix<float, 12, 1> motorAngles;
    Eigen::Matrix<float, 12, 1> motorVelocities;
    //Note: force direction?
    Eigen::Matrix<float, 4, 1> footForce;
    Eigen::Matrix<bool, 4, 1> footContact;

    float yawOffset = 0.f;
    RobotConfig* &config;
    IMU imu;
    std::array<MotorState, 20> motorState;

    void Update();

    Eigen::Matrix<float, 3, 4> GetFootPositionsInBaseFrame();

    Eigen::Matrix<float, 3, 4> GetFootPositionsInWorldFrame(
            bool useInput=false, Vec3<float> basePositionIn={0.f,0.f,0.f}, Quat<float> baseOrientationIn={1.f,0.f,0.f,0.f});

    Eigen::Matrix<float, 3, 3> ComputeJacobian(int legId);

    std::map<int, float> MapContactForceToJointTorques(int legId, Eigen::Matrix<float, 3, 1> contractForce);
private:

    float CalibrateYaw();
};

#endif // QR_ROBOT_STATE_H
