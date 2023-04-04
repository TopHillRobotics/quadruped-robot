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

#include "robots/qr_robot.h"


namespace Quadruped {

qrRobot::qrRobot(): timer(useRosTime)
{
    accFilter = qrMovingWindowFilter<float, 3>(5);
    gyroFilter = qrMovingWindowFilter<float, 3>(5);
    rpyFilter = qrMovingWindowFilter<float, 3>(5);
    quatFilter = qrMovingWindowFilter<float, 4>(5);
    motorVFilter = qrMovingWindowFilter<float, 12>(20);
}


void qrRobot::Reset()
{
    std::vector<float> comOffsetList = robotConfig["robot_params"][GetControlMode()]["com_offset"].as<std::vector<float >>();
    comOffset = Eigen::MatrixXf::Map(&comOffsetList[0], 3, 1);

    std::vector<std::vector<float >> hipOffsetList =
        robotConfig["robot_params"]["hip_offset"].as<std::vector<std::vector<float>>>();
    Vec3<float> hipOffsetFR = Eigen::MatrixXf::Map(&hipOffsetList[0][0], 3, 1);
    Vec3<float> hipOffsetFL = Eigen::MatrixXf::Map(&hipOffsetList[1][0], 3, 1);
    Vec3<float> hipOffsetRL = Eigen::MatrixXf::Map(&hipOffsetList[2][0], 3, 1);
    Vec3<float> hipOffsetRR = Eigen::MatrixXf::Map(&hipOffsetList[3][0], 3, 1);
    hipOffset << hipOffsetFR, hipOffsetFL, hipOffsetRL, hipOffsetRR;

    basePosition = {0.f, 0.f, A1_BODY_HIGHT};

    accFilter.Reset();
    gyroFilter.Reset();
    rpyFilter.Reset();
    quatFilter.Reset();
}


void qrRobot::UpdateDataFlow()
{
    for (int legId=0; legId<NumLeg; ++legId) {
        stateDataFlow.footJvs[legId] = ComputeJacobian(legId);
    }
    stateDataFlow.footVelocitiesInBaseFrame = ComputeFootVelocitiesInBaseFrame();
    stateDataFlow.footPositionsInBaseFrame = FootPositionsInBaseFrame(motorAngles);

    stateDataFlow.baseRMat = robotics::math::quaternionToRotationMatrix(baseOrientation).transpose();
    stateDataFlow.baseRInControlFrame = stateDataFlow.groundRMat.transpose() * stateDataFlow.baseRMat;
    ComputeMoment();
}


void qrRobot::ComputeMoment()
{
    stateDataFlow.estimatedMoment.setZero();
    for (int legId=0; legId < NumLeg; ++legId) {
        const Mat3<float>& Ji = stateDataFlow.footJvs[legId];
        Vec3<float> Fi = Ji.transpose().inverse() * motortorque.segment(3*legId,3);
        stateDataFlow.estimatedFootForce.col(legId) = Fi;
        stateDataFlow.estimatedMoment += (stateDataFlow.footPositionsInBaseFrame.col(legId).cross(Fi));
    }
    stateDataFlow.estimatedMoment = stateDataFlow.estimatedMoment.cwiseQuotient(stateDataFlow.estimatedFootForce.rowwise().sum());
}


Vec3<float> qrRobot::WithLegSigns(const Vec3<float>& v, int leg_id)
{
    switch (leg_id) {
        case 0:
            return Vec3<float>(v[0], -v[1], v[2]);
        case 1:
            return Vec3<float>(v[0], v[1], v[2]);
        case 2:
            return Vec3<float>(-v[0], -v[1], v[2]);
        case 3:
            return Vec3<float>(-v[0], v[1], v[2]);
        default:
            throw std::runtime_error("Invalid leg id!");
    }
}


Vec3<float> qrRobot::FootPositionInHipFrameToJointAngle(Vec3<float> &foot_position, int hip_sign)
{
    /* hip sign means the left or right hip frame is different */
    float signedHipLength = hipLength * hip_sign;
    Vec3<float> xyz(foot_position[0], foot_position[1], foot_position[2]);
    Vec3<float> legLength(signedHipLength, upperLegLength, lowerLegLength);

    /* assume that upper leg has the same lenghth as lower leg*/
    float thetaAB, thetaHip, thetaKnee;
    thetaKnee = -acos((xyz.squaredNorm() - legLength.squaredNorm()) / (2 * lowerLegLength * upperLegLength));
    float l = sqrt(upperLegLength * upperLegLength + lowerLegLength * lowerLegLength +
        2 * upperLegLength * lowerLegLength * cos(thetaKnee));
    thetaHip = asin(-xyz.x() / l) - thetaKnee / 2;
    float c1 = signedHipLength * xyz.y() - l * cos(thetaHip + thetaKnee / 2) * xyz.z();
    float s1 = l * cos(thetaHip + thetaKnee / 2) * xyz.y() + signedHipLength * xyz.z();
    thetaAB = atan2(s1, c1);

    return Vec3<float>(thetaAB, thetaHip, thetaKnee);
}


Vec3<float> qrRobot::FootPositionInHipFrame(Vec3<float> &angles, int hip_sign)
{
    /* Simple geometric caculation */
    float thetaAB = angles[0], thetaHip = angles[1], thetaKnee = angles[2];
    float signedHipLength = hipLength * hip_sign;
    float legDistance = sqrt(upperLegLength * upperLegLength + lowerLegLength * lowerLegLength +
        2 * upperLegLength * lowerLegLength * cos(thetaKnee));
    float effSwing = thetaHip + thetaKnee / 2;
    float offXHip, offZHip, offYHip, offX, offY, offZ;
    offXHip = -legDistance * sin(effSwing);
    offZHip = -legDistance * cos(effSwing);
    offYHip = signedHipLength;

    offX = offXHip;
    offY = cos(thetaAB) * offYHip - sin(thetaAB) * offZHip;
    offZ = sin(thetaAB) * offYHip + cos(thetaAB) * offZHip;

    return Vec3<float>(offX, offY, offZ);
}


Eigen::Matrix<float, 3, 3> qrRobot::AnalyticalLegJacobian(Vec3<float> &leg_angles, int leg_id)
{
    float signedHipLength = hipLength * pow(-1, leg_id + 1);
    Vec3<float> t = leg_angles;

    float lEff = sqrt(upperLegLength * upperLegLength + lowerLegLength * lowerLegLength +
        2 * upperLegLength * lowerLegLength * cos(t[2]));
    float tEff = t[1] + t[2] / 2;

    Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
    J(0, 0) = 0;
    J(0, 1) = -lEff * cos(tEff);
    J(0, 2) = lowerLegLength * upperLegLength * sin(t[2]) * sin(tEff) / lEff - lEff * cos(
        tEff) / 2;
    J(1, 0) = -signedHipLength * sin(t[0]) + lEff * cos(t(0)) * cos(tEff);
    J(1, 1) = -lEff * sin(t(0)) * sin(tEff);
    J(1, 2) = -lowerLegLength * upperLegLength * sin(t(0)) * sin(t(2)) * cos(
        tEff) / lEff - lEff * sin(t(0)) * sin(tEff) / 2;
    J(2, 0) = signedHipLength * cos(t(0)) + lEff * sin(t(0)) * cos(tEff);
    J(2, 1) = lEff * sin(tEff) * cos(t(0));
    J(2, 2) = lowerLegLength * upperLegLength * sin(t(2)) * cos(t(0)) * cos(
        tEff) / lEff + lEff * sin(tEff) * cos(t(0)) / 2;

    return J;
}


Mat34<float> qrRobot::FootPositionsInBaseFrame(Eigen::Matrix<float, 12, 1> foot_angles)
{
    Mat34<float> footPositions;
    Vec3<float> singleFootAngles;
    for (int legId = 0; legId < NumLeg; legId++) {
        singleFootAngles = foot_angles.segment(3*legId, 3);
        footPositions.col(legId) = FootPositionInHipFrame(singleFootAngles, pow(-1, legId + 1));
    }
    return footPositions + hipOffset;
}


Mat34<float> qrRobot::ComputeFootVelocitiesInBaseFrame()
{
    Eigen::Matrix<float,3,4> footVelocitiesInBaseFrame;
    for (int legId=0; legId < NumLeg; ++legId) {
        const Mat3<float>& jacobian = stateDataFlow.footJvs[legId];// ComputeJacobian(legId);
        /* Only pick the jacobian related to joint motors */
        Vec3<float> jointVelocities = motorVelocities.segment(legId * 3, 3);
        footVelocitiesInBaseFrame.col(legId) = jacobian * jointVelocities;   
    }
    return footVelocitiesInBaseFrame;
}
            

void qrRobot::ComputeMotorAnglesFromFootLocalPosition(
    int leg_id,
    Vec3<float> foot_local_position,
    Eigen::Matrix<int, 3, 1> &joint_idx,
    Vec3<float> &joint_angles)
{
    joint_idx << NumMotorOfOneLeg * leg_id, NumMotorOfOneLeg * leg_id + 1, NumMotorOfOneLeg * leg_id + 2;
    Vec3<float> singleFootLocalPosition = foot_local_position - hipOffset.col(leg_id);
    joint_angles = this->FootPositionInHipFrameToJointAngle(singleFootLocalPosition, pow(-1, (leg_id + 1)));
}


Vec3<float> qrRobot::ComputeMotorVelocityFromFootLocalVelocity(
    int leg_id,
    Vec3<float> leg_angles,
    Vec3<float> foot_local_velocity)
{
    Vec3<float> dq = AnalyticalLegJacobian(leg_angles,leg_id).inverse() * foot_local_velocity;
    return dq;
}


Mat34<float> qrRobot::GetFootPositionsInWorldFrame(bool use_input, Vec3<float> base_position, Quat<float> base_orientation)
{
    Mat34<float> footPositionsInBaseFrame = GetFootPositionsInBaseFrame();
    if (!use_input) {
        return robotics::math::invertRigidTransform(basePosition, baseOrientation, footPositionsInBaseFrame);
    } else {
        return robotics::math::invertRigidTransform(base_position, base_orientation, footPositionsInBaseFrame);
    }
}


Eigen::Matrix<float, 3, 3> qrRobot::ComputeJacobian(int leg_id)
{
    Vec3<float> legMotorAngles;
    legMotorAngles << this->GetMotorAngles().block(leg_id * 3, 0, 3, 1);
    return this->AnalyticalLegJacobian(legMotorAngles, leg_id);
}


std::map<int, float> qrRobot::MapContactForceToJointTorques(int leg_id, Vec3<float> contact_force)
{
    const Eigen::Matrix<float, 3, 3>& jv = stateDataFlow.footJvs[leg_id];
    Vec3<float> motorTorquesPerLeg = jv.transpose() * contact_force;
    std::map<int, float> motorTorquesDict;
    for (int torqueIndex = 0; torqueIndex < motorTorquesPerLeg.size(); torqueIndex++) {
        int jointIndex = torqueIndex + leg_id * NumMotorOfOneLeg;
        motorTorquesDict[jointIndex] = motorTorquesPerLeg[torqueIndex];
    }
    return motorTorquesDict;
}

} // namespace Quadruped
