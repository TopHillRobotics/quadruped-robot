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

#include "estimators/qr_ground_surface_estimator.h"

namespace Quadruped {
  
qrGroundSurfaceEstimator::qrGroundSurfaceEstimator(qrRobot *robotIn, std::string terrainConfigPath):
    robot(robotIn)
{
    footStepperConfig = YAML::LoadFile(terrainConfigPath);
    terrain.footHoldOffset = footStepperConfig["foothold_offset"].as<float>();
    robot->footHoldOffset = terrain.footHoldOffset;
    Reset(0.f);
    std::cout << "init groundEsitmator finish\n" << std::endl;
}


void qrGroundSurfaceEstimator::Update(float currentTime)
{   
    Eigen::Matrix<bool, 4, 1> contactState = robot->GetFootContact();
    bool shouldUpdate = false;
    int N=0;
    int i=0;
    for (i=0; i < 4; ++i) {
        if (contactState[i]) {
            if ((!lastContactState[i])) {
                shouldUpdate = true;
            }
            ++N;
        }
    }
    lastContactState = contactState;
    if (N <= 3 || !shouldUpdate) {
        return;
    }
    Eigen::Matrix<double, 3, 4> footPositionsInBaseFrame = robot->GetFootPositionsInBaseFrame().cast<double>();
    bodyPositionInWorldFrame = robot->GetBasePosition(); 
    bodyPositionInWorldFrame[2] = robot->stateDataFlow.heightInControlFrame;
    pZ = footPositionsInBaseFrame.row(2);
    W.col(1) = footPositionsInBaseFrame.row(0);
    W.col(2) = footPositionsInBaseFrame.row(1);

    /* the plane : z(x, y) = a[0] + a[1]x + a[2]y */
    Mat3<double> ww = W.transpose()* W;
    a = ww.inverse()* W.transpose()*pZ;
    GetNormalVector(true);
    ComputeControlFrame();
}


void qrGroundSurfaceEstimator::Reset(float currentTime)
{
    terrain.terrainType = static_cast<TerrainType>(footStepperConfig["terrain_type"].as<int>());
    switch (robot->controlParams["mode"]) {
        case LocomotionMode::POSITION_LOCOMOTION: {
            terrain.terrainType = TerrainType::PLUM_PILES;
        } break;
        case LocomotionMode::VELOCITY_LOCOMOTION: {
            terrain.terrainType = TerrainType::SLOPE;
        } break;
        case LocomotionMode::ADVANCED_TROT: {
            terrain.terrainType = TerrainType::STAIRS;
        } break;
        default:
            break;
    }

    switch (terrain.terrainType) {
        case TerrainType::PLANE: {
            // terrain.costMap = terrain.block<N, N>(0, 0);
        } break;
        case TerrainType::PLUM_PILES: {
            float gapWidth = footStepperConfig["gap_width"].as<float>();
            std::vector<float> distanceOfGaps = footStepperConfig["gaps"].as<std::vector<float>>();
            for (float distance : distanceOfGaps) {
                qrGap* gap = new qrGap(distance, gapWidth, {0, 0, 0});
                terrain.gaps.push_back(gap);
            }
        } break;
        case TerrainType::STAIRS: {
            // todo
        } break;
        case TerrainType::SLOPE: {
            // todo
        } break;
        case TerrainType::ROUGH: {
            // todo
        } break;
        default : throw std::domain_error("no such terrain!");
    }

    a = Eigen::Matrix<double, 3, 1>::Zero();
    W = Eigen::Matrix<double,4,3>::Ones();
    pZ = Vec4<double>::Zero();
    n << 0.f, 0.f, 1.f;
    bodyPositionInWorldFrame = robot->GetBasePosition(); 
    controlFrameRPY << 0., 0., 0.;
    controlFrameOrientation << 1.0, 0, 0, 0;
    controlFrame << 1, 0, 0 ,0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
    lastContactState << 0, 0, 0, 0;

    robot->stateDataFlow.groundRMat.setIdentity();
    robot->stateDataFlow.groundOrientation << 1.f, 0.f, 0.f, 0.f;
    robot->stateDataFlow.baseRInControlFrame.setIdentity();
}


float qrGroundSurfaceEstimator::GetZInBaseFrame(float x, float y)
{
    float z = a.transpose() * Vec3<double>(1, x, y);
    return z;
}


float qrGroundSurfaceEstimator::GetZInControlFrame(float x, float y)
{
    /* a1*x + a2*y + (-1)*z + a0 = 0  ==>
       [a1, a2, -1, a0]^T * [x, y, z, 1] = 0
       let A_b = [a1, a2, -1, a0], a_b = [a0, a1, a2]
       A_c = Rcb * A_b, where Rcb is a homogenous transformation. */
    Vec4<double> AInControlFrame =  controlFrame * Vec4<double>(a[1],a[2],-1.f, a[0]);
    float z = (AInControlFrame[0]*x + AInControlFrame[1]*y + AInControlFrame[3]*1) / (-AInControlFrame[2]);
    return z;
}


Eigen::Matrix<double, 3, 1> qrGroundSurfaceEstimator::GetNormalVector(bool update)
{
    if (update) {
        double factor = std::sqrt(a[1]*a[1] + a[2]*a[2] + 1);
        n << -a[1], -a[2], 1.0;
        n /= factor;
    }
    return n; // in base frame
}


Eigen::Matrix<double, 4, 4> qrGroundSurfaceEstimator::ComputeControlFrame()
{
    // Vec3<float> comRPY = robot->GetBaseRollPitchYaw();
    Quat<double> quat = robot->GetBaseOrientation().cast<double>();
    Mat3<double> BaseR = robotics::math::quaternionToRotationMatrix(quat).transpose();
    Vec3<double> nInWorldFrame = robotics::math::invertRigidTransform<double>({0,0,0},quat, n);
    nInWorldFrame << 0,0,1; // todo , assume the ground is flat
    Vec3<double> xAxis = BaseR.col(0);
    Vec3<double> yAxis = nInWorldFrame.cross(xAxis);
    yAxis.normalize();
    xAxis = yAxis.cross(nInWorldFrame);
    xAxis.normalize();
    nInWorldFrame.normalize();
    Mat3<double> R;
    R.col(0) = xAxis;
    R.col(1) = yAxis;
    R.col(2) = nInWorldFrame;
    double ratio = 0.8; // todo, 0.7 for walk mode
    Vec3<double> newRPY = robotics::math::rotationMatrixToRPY(R.transpose());

    controlFrameRPY = (1-ratio) * controlFrameRPY + ratio * newRPY;
    controlFrameRPY[0] = 0;
    // controlFrameRPY[1] = 0;
    R = robotics::math::rpyToRotMat(controlFrameRPY).transpose();
    /* filtering the controlframe rpy angles.
    
     * float comYaw = comRPY[2];
     * float cy = cos(comYaw);
     * float sy = sin(comYaw);
     * float pitch = atan(n[0]/(n[1]+1e-7)*cy+n[1]/n[2]*sy);
     * float roll = atan((sy*sin(pitch)-n[1]/n[2]*cos(pitch))/cy);
     *
     * // float pitch = atan(n[0]/n[2]);
     * // float roll = atan(-n[1]/n[2]);
     *
     * controlFrameRPY << roll, pitch, comYaw;
     */
    controlFrameOrientation = robotics::math::rpyToQuat(controlFrameRPY);
    // Mat3<float> R = robotics::math::rpyToRotMat(controlFrameRPY).transpose();
    controlFrame.block<3,3>(0,0) = R;
    controlFrame.block<3,1>(0,3) = robot->GetBasePosition().cast<double>();

    robot->stateDataFlow.groundRMat = R.cast<float>();
    robot->stateDataFlow.groundOrientation = controlFrameOrientation.cast<float>();
    robot->stateDataFlow.baseRInControlFrame = robot->stateDataFlow.groundRMat.transpose() * robot->stateDataFlow.baseRMat;
    /* controlFrame is represent in world frame */
    return controlFrame;
}


Eigen::Matrix<float, 3, 3> qrGroundSurfaceEstimator::GetAlignedDirections()
{
    Mat3<float> R = controlFrame.block<3,3>(0,0).cast<float>();   
    return R;
}

} // Namespace Quadruped
