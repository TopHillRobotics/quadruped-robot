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

#include "estimator/qr_ground_estimator.h"

qrGroundSurfaceEstimator::qrGroundSurfaceEstimator(qrRobot* robot, std::string terrainConfigPath):
    robot(robot)
{
    terrainConfig = YAML::LoadFile(terrainConfigPath);
    this->robotState = robot->GetRobotState();
    this->robotConfig = robot->GetRobotConfig();
    Reset();
}

void qrGroundSurfaceEstimator::Update()
{
    Vec4<bool> contactState = this->robotState->GetFootContact();
    bool shouldUpdate = false;
    int N = 0;
    int i = 0;
    for(i = 0; i < 4; ++i) {
        if (contactState[i]) {
            if (!this->lastContactStates[i]) {
                shouldUpdate = true;
            }
            ++N;
        }
    }
    this->lastContactStates = contactState;
    if (N <= 3 || !shouldUpdate) {
        return ;
    }
    Eigen::Matrix<double, 3, 4> footPositionsInBaseFrame = this->robotState->GetFootPositionInBaseFrame().cast<double>();
    this->pZ = footPositionsInBaseFrame.row(2);
    this->W.col(1) = footPositionsInBaseFrame.row(0);
    this->W.col(2) = footPositionsInBaseFrame.row(1);
    
    Mat3<double> ww = this->W.transpose() * this->W;
    this->a = ww.inverse() * this->W.transpose() * this->pZ;
    GetNormalVector(true);
    ComputeControlFrame();
}

void qrGroundSurfaceEstimator::Reset()
{
    this->terrain.terrainType = static_cast<TerrainType>(this->terrainConfig["terrain_type"].as<int>());
    if (this->robotConfig->controlMode == LocomotionMode::POSITION_LOCOMOTION) {
        this->terrain.terrainType = TerrainType::PLUM_PILES;
    } else if (this->robotConfig->controlMode == LocomotionMode::VELOCITY_LOCOMOTION) {
        this->terrain.terrainType = TerrainType::PLANE;
    }

    this->terrain.footHoldOffset = this->terrainConfig["foothold_offset"].as<float>();
    // TODO:: check this
    // this->robotConfig->footHoldOffset = this->terrain.footHoldOffset;
    switch (terrain.terrainType) {
        case TerrainType::PLANE: {
        } break;
        case TerrainType::PLUM_PILES: {
            float gapWidth = this->terrainConfig["gap_width"].as<float>();
            std::vector<float> distanceOfGaps = this->terrainConfig["gaps"].as<std::vector<float>>();
            for (float distance : distanceOfGaps) {
                Gap* gap = new Gap(distance, gapWidth, {0, 0, 0});
                this->terrain.gaps.push_back(gap);
            }
        } break;
        case TerrainType::STAIRS: {
            // todo
        } break;
        default : throw std::domain_error("no such terrain!");
    }

    this->a = Vec3<double>::Zero();
    this->W = Eigen::Matrix<double,4,3>::Ones();
    this->pZ = Vec4<double>::Zero();
    this->n << 0.f, 0.f, 1.f;
    this->controlFrameRPY << 0., 0., 0.;
    this->controlFrameOrientation << 1.0, 0, 0, 0;
    this->controlFrame << 1, 0, 0 ,0,
                          0, 1, 0, 0,
                          0, 0, 1, 0,
                          0, 0, 0, 1;
    this->lastContactStates << 0, 0, 0, 0;
}

float qrGroundSurfaceEstimator::GetZ(float x, float y)
{
    float z = a.transpose() * Vec3<double>(1, x, y);
    return z;
}

Vec3<double> qrGroundSurfaceEstimator::GetNormalVector(bool update)
{
    if (update){
        double factor = std::sqrt(a[1] * a[1] + a[2] * a[2] + 1);
        this->n << -(this->a[1]), -(this->a[2]), 1.0;
        this->n /= factor;
    }
    return n;
}

Mat4<double> qrGroundSurfaceEstimator::ComputeControlFrame()
{
    Quat<double> quat = this->robotState->GetBaseOrientation().cast<double>();
    Vec3<double> nInWorldFrame = math::InvertRigidTransform<double>({0,0,0},quat, n);
    Vec3<double> xAxis = math::Quat2RotMat(quat).col(0);
    Vec3<double> yAxis = nInWorldFrame.cross(xAxis);
    yAxis.normalize();
    xAxis = yAxis.cross(nInWorldFrame);
    xAxis.normalize();
    nInWorldFrame.normalize();
    Mat3<double> R;
    R.col(0) = xAxis;
    R.col(1) = yAxis;
    R.col(2) = nInWorldFrame;
    double ratio = 0.7; // todo, 0.7 for walk mode
    this->controlFrameRPY = (1 - ratio) * this->controlFrameRPY + ratio * math::RotMat2Rpy(R);
    this->controlFrameOrientation = math::Rpy2Quat(this->controlFrameRPY);
    this->controlFrame.block<3,3>(0,0) = R;
    this->controlFrame.block<3,1>(0,3) = this->robotState->GetBasePosition().cast<double>();
    return controlFrame;
}

Mat3<float> qrGroundSurfaceEstimator::GetAlignedDirections()
{
    Mat3<float> R = controlFrame.block<3,3>(0,0).cast<float>();
    return R;
}
