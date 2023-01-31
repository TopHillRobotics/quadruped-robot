/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Base class of quadruped robots.
* Author: Zhao Yao
* Create: 2021-11-3
* Notes: xx
* Modify: init the file. @ Zhao Yao 2021.11.19
*/
#include "robots/robot.h"
namespace Quadruped {
    Robot::Robot() : timer(useRosTime)
    {
        accFilter = MovingWindowFilter<float, 3>(5); // 5 
        gyroFilter = MovingWindowFilter<float, 3>(5);
        rpyFilter = MovingWindowFilter<float, 3>(5);
        quatFilter = MovingWindowFilter<float, 4>(5);
        motorVFilter = MovingWindowFilter<float, 12>(20); // 20
    }


    void Robot::Reset()
    {
        std::vector<float> comOffsetList = robotConfig["robot_params"][GetControlMode()]["com_offset"].as<std::vector<float >>();
        comOffset = Eigen::MatrixXf::Map(&comOffsetList[0], 3, 1);

        std::vector<std::vector<float >> hipOffsetList =
            robotConfig["robot_params"]["hip_offset"].as<std::vector<std::vector<float>>>();
        Eigen::Matrix<float, 3, 1> hipOffsetFR = Eigen::MatrixXf::Map(&hipOffsetList[0][0], 3, 1);
        Eigen::Matrix<float, 3, 1> hipOffsetFL = Eigen::MatrixXf::Map(&hipOffsetList[1][0], 3, 1);
        Eigen::Matrix<float, 3, 1> hipOffsetRL = Eigen::MatrixXf::Map(&hipOffsetList[2][0], 3, 1);
        Eigen::Matrix<float, 3, 1> hipOffsetRR = Eigen::MatrixXf::Map(&hipOffsetList[3][0], 3, 1);
        hipOffset << hipOffsetFR, hipOffsetFL, hipOffsetRL, hipOffsetRR;
        hipOffset2Com = hipOffset.colwise() - comOffset;

        // yawOffset = baseRollPitchYaw[2];
        basePosition = {0.f, 0.f, A1_BODY_HIGHT};
        // baseRollPitchYaw.setZero();
        // baseOrientation << 1.f, 0.f, 0.f, 0.f;
        // clear filer
        accFilter.Reset();
        gyroFilter.Reset();
        rpyFilter.Reset();
        quatFilter.Reset();
    }

    Vec3<float> Robot::withLegSigns(const Vec3<float>& v, int legID)
    {
        switch (legID) {
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

    Eigen::Matrix<float, 3, 1>
    Robot::FootPositionInHipFrameToJointAngle(Eigen::Matrix<float, 3, 1> &footPosition, int hipSign)
    {
        float signedHipLength = hipLength * hipSign;
        Eigen::Matrix<float, 3, 1> xyz(footPosition[0], footPosition[1], footPosition[2]);
        Eigen::Matrix<float, 3, 1> legLength(signedHipLength, upperLegLength, lowerLegLength);

        float thetaAB, thetaHip, thetaKnee;
        thetaKnee = -acos((xyz.squaredNorm() - legLength.squaredNorm()) / (2 * lowerLegLength * upperLegLength));
        float l = sqrt(upperLegLength * upperLegLength + lowerLegLength * lowerLegLength +
            2 * upperLegLength * lowerLegLength * cos(thetaKnee));
        thetaHip = asin(-xyz.x() / l) - thetaKnee / 2;
        float c1 = signedHipLength * xyz.y() - l * cos(thetaHip + thetaKnee / 2) * xyz.z();
        float s1 = l * cos(thetaHip + thetaKnee / 2) * xyz.y() + signedHipLength * xyz.z();
        thetaAB = atan2(s1, c1);

        return Eigen::Matrix<float, 3, 1>(thetaAB, thetaHip, thetaKnee);
    }

    Eigen::Matrix<float, 3, 1> Robot::FootPositionInHipFrame(Eigen::Matrix<float, 3, 1> &angles, int hipSign)
    {
        float thetaAB = angles[0], thetaHip = angles[1], thetaKnee = angles[2];
        float signedHipLength = hipLength * hipSign;
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

        return Eigen::Matrix<float, 3, 1>(offX, offY, offZ);
    }

    Eigen::Matrix<float, 3, 3> Robot::AnalyticalLegJacobian(Eigen::Matrix<float, 3, 1> &legAngles, int legId)
    {

        float signedHipLength = hipLength * pow(-1, legId + 1);
        Eigen::Matrix<float, 3, 1> t = legAngles;

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

    Eigen::Matrix<float, 3, 4> Robot::FootPositionsInBaseFrame(Eigen::Matrix<float, 12, 1> footAngles)
    {
        // Eigen::Map<Eigen::MatrixXf> reshapedFootAngles(footAngles.data(), 3, 4);
        Eigen::Matrix<float, 3, 4> footPositions;
        Eigen::Matrix<float, 3, 1> singleFootAngles;
        for (int legId = 0; legId < NumLeg; legId++) {
            // singleFootAngles << reshapedFootAngles.col(legId);
            singleFootAngles = footAngles.segment(3*legId, 3);
            footPositions.col(legId) = FootPositionInHipFrame(singleFootAngles, pow(-1, legId + 1));
        }
        return footPositions + hipOffset;
    }

    Eigen::Matrix<float, 3, 4> Robot::ComputeFootVelocitiesInBaseFrame()
    {
        Eigen::Matrix<float,3,4> footVelocitiesInBaseFrame;
        for (int legId=0; legId < NumLeg; ++legId) {
            const Mat3<float>& jacobian = stateDataFlow.footJvs[legId];// ComputeJacobian(legId);
            // Only pick the jacobian related to joint motors
            Vec3<float> jointVelocities = motorVelocities.segment(legId * 3, 3);
            footVelocitiesInBaseFrame.col(legId) = jacobian * jointVelocities;   
        }
        return footVelocitiesInBaseFrame;
    }
                

    void Robot::ComputeMotorAnglesFromFootLocalPosition(int legId,
                                                        Eigen::Matrix<float, 3, 1> footLocalPosition,
                                                        Eigen::Matrix<int, 3, 1> &jointIdx,
                                                        Eigen::Matrix<float, 3, 1> &jointAngles)
    {
        jointIdx << NumMotorOfOneLeg * legId, NumMotorOfOneLeg * legId + 1, NumMotorOfOneLeg * legId + 2;
        Eigen::Matrix<float, 3, 1> singleFootLocalPosition = footLocalPosition - hipOffset.col(legId);
        jointAngles = this->FootPositionInHipFrameToJointAngle(singleFootLocalPosition, pow(-1, (legId + 1)));
    }

    Eigen::Matrix<float, 3, 1> Robot::ComputeMotorVelocityFromFootLocalVelocity(int legId,
                                                    Eigen::Matrix<float, 3, 1> legAngles,
                                                    Eigen::Matrix<float, 3, 1> footLocalVelocity)
    {
        Eigen::Matrix<float, 3, 1> dq = AnalyticalLegJacobian(legAngles,legId).inverse()*footLocalVelocity;
        return dq;
        // return {0,0,0};
    }


    Eigen::Matrix<float, 3, 4> Robot::GetFootPositionsInBaseFrame()
    {
        // return FootPositionsInBaseFrame(this->GetMotorAngles());
        return stateDataFlow.footPositionsInBaseFrame;
    }

    Eigen::Matrix<float, 3, 4> Robot::GetFootPositionsInWorldFrame(bool useInput, Vec3<float> basePositionIn, Quat<float> baseOrientationIn)
    {
        Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame = GetFootPositionsInBaseFrame(); // base to  world frame
        if (!useInput) {
            return robotics::math::invertRigidTransform(basePosition, baseOrientation, footPositionsInBaseFrame);
        } else {
            return robotics::math::invertRigidTransform(basePositionIn, baseOrientationIn, footPositionsInBaseFrame);
        }
    }

    Eigen::Matrix<float, 3, 3> Robot::ComputeJacobian(int legId)
    {
        Eigen::Matrix<float, 3, 1> legMotorAngles;
        legMotorAngles << this->GetMotorAngles().block(legId * 3, 0, 3, 1);
        return this->AnalyticalLegJacobian(legMotorAngles, legId);

    }

    Eigen::Matrix<float, 3, 4> Robot::GetHipPositionsInBaseFrame()
    {
        return defaultHipPosition;
    }

    std::map<int, float> Robot::MapContactForceToJointTorques(int legId, Eigen::Matrix<float, 3, 1> contractForce)
    {
        const Eigen::Matrix<float, 3, 3>& jv = stateDataFlow.footJvs[legId]; // ComputeJacobian(legId);
        Eigen::Matrix<float, 3, 1> motorTorquesPerLeg = jv.transpose() * contractForce; // TODO TEST
        std::map<int, float> motorTorquesDict;
        for (int torqueIndex = 0; torqueIndex < motorTorquesPerLeg.size(); torqueIndex++) {
            int jointIndex = torqueIndex + legId * NumMotorOfOneLeg;
            motorTorquesDict[jointIndex] = motorTorquesPerLeg[torqueIndex];
        }
        return motorTorquesDict;
    }

    Eigen::Matrix<float, 12, 1> Robot::GetMotorPositionGains()
    {
        return motorKps;
    }

    Eigen::Matrix<float, 12, 1> Robot::GetMotorVelocityGains()
    {
        return motorKds;
    }

    void Robot::UpdateDataFlow()
    {
        for (int legId=0; legId<NumLeg; ++legId) {
            stateDataFlow.footJvs[legId] = ComputeJacobian(legId);    
        }
        stateDataFlow.footVelocitiesInBaseFrame = ComputeFootVelocitiesInBaseFrame();
        stateDataFlow.footPositionsInBaseFrame = FootPositionsInBaseFrame(motorAngles);
        // baseLinearAcceleration
        stateDataFlow.baseRMat = robotics::math::quaternionToRotationMatrix(baseOrientation).transpose();
        stateDataFlow.baseRInControlFrame = stateDataFlow.groundRMat.transpose() * stateDataFlow.baseRMat;
        ComputeMoment();
    }

    void Robot::ComputeMoment()
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

} // namespace Quadruped
