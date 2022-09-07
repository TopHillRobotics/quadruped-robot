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
#include "state_estimator/qr_ground_estimator.h"

  
qrGroundSurfaceEstimator::qrGroundSurfaceEstimator(qrRobot *robotIn, std::string terrainConfigPath, unsigned int windowSize)
    : robot(robotIn)
{
    footStepperConfig = YAML::LoadFile(terrainConfigPath);
    Reset(0.f);
}

void qrGroundSurfaceEstimator::Update(float currentTime)
{   
    Eigen::Matrix<bool, 4, 1> contactState = robot->GetFootContacts();

    if(!ShouldUpdate(contactState)){
        return ;
    }

    // equation of the plane is pZ(x,y) = a_0 + a_1 * x + a_2 * y
    // then the least square problem is min||Wa - pZ||_2
    Eigen::Matrix<double, 3, 4> footPositionsInBaseFrame = robot->state.GetFootPositionsInBaseFrame().cast<double>();
    pZ = footPositionsInBaseFrame.row(2);
    W.col(1) = footPositionsInBaseFrame.row(0);
    W.col(2) = footPositionsInBaseFrame.row(1);
    
    // the analytic solution is (W^T * W)^(-1) * W^T * pZ
    Mat3<double> ww = W.transpose()* W;
    a = ww.inverse()* W.transpose()*pZ;
    GetNormalVector(true);
    ComputeControlFrame();
}

void qrGroundSurfaceEstimator::Reset(float currentTime) {
    terrain.terrainType = static_cast<TerrainType>(footStepperConfig["terrain_type"].as<int>());

    terrain.footHoldOffset = footStepperConfig["foothold_offset"].as<float>();
    robot->config->footHoldOffset = terrain.footHoldOffset;
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
        default : throw std::domain_error("no such terrain!");
    }

    a = Eigen::Matrix<double, 3, 1>::Zero();
    W = Eigen::Matrix<double, 4, 3>::Ones();
    pZ = Vec4<double>::Zero();
    n << 0.f, 0.f, 1.f;
    controlFrameRPY << 0., 0., 0.;
    controlFrameOrientation << 1.0, 0, 0, 0;
    controlFrame << 1, 0, 0 ,0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
    lastContactState << 0, 0, 0, 0;
}

float qrGroundSurfaceEstimator::GetZ(float x, float y)
{
    float z = a.transpose() * Vec3<double>(1, x, y);
    return z;
}

Eigen::Matrix<double, 3, 1> qrGroundSurfaceEstimator::GetNormalVector(bool update)
{
    // z axis of normal vector should be positive.It will be easy to calculate the control frame
    if (update){
        double factor = std::sqrt(a[1]*a[1] + a[2]*a[2] + 1);
        n << -a[1], -a[2], 1.0;
        n /= factor;
    }

    return n; // in base frame
}

Eigen::Matrix<double, 4, 4> qrGroundSurfaceEstimator::ComputeControlFrame()
{
    // x axis of the control frame is the orientation of robot base axis projected on the plane
    // z axis is the norm vector of the plane
    // y axis is the cross of x axis and z axis
    Quat<double> quat = robot->GetBaseOrientation().cast<double>();
    Vec3<double> nInWorldFrame = math::invertRigidTransform<double>({0,0,0},quat, n);
    Vec3<double> xAxis = math::quaternionToRotationMatrix(quat).transpose().col(0);
    Vec3<double> yAxis = nInWorldFrame.cross(xAxis);
    yAxis.normalize();
    xAxis = yAxis.cross(nInWorldFrame);
    xAxis.normalize();
    nInWorldFrame.normalize();
    Mat3<double> R;
    R.col(0) = xAxis;
    R.col(1) = yAxis;
    R.col(2) = nInWorldFrame;
    double ratio = 0.7; // TODO: 0.7 for walk mode
    controlFrameRPY = (1-ratio) * controlFrameRPY + ratio * math::rotationMatrixToRPY(R.transpose());
    controlFrameOrientation = math::rpyToQuat(controlFrameRPY);
    controlFrame.block<3,3>(0,0) = R;
    controlFrame.block<3,1>(0,3) = robot->GetBasePosition().cast<double>();
    return controlFrame;
}

Eigen::Matrix<float, 3, 3> qrGroundSurfaceEstimator::GetAlignedDirections()
{
    Mat3<float> R = controlFrame.block<3,3>(0,0).cast<float>();   
    return R;
}

bool qrGroundSurfaceEstimator::ShouldUpdate(const Eigen::Matrix<bool, 4, 1> &contactState)
{
    bool shouldUpdate = false;
    int N=0;
    int i=0;
    for(i=0; i < 4; ++i) {
        if (contactState[i]) {
            if ((!lastContactState[i])) {
                shouldUpdate = true;
            }
            ++N;
        }
    }
    lastContactState = contactState;
    // when all foot on the ground and contact state changed will the plane update
    if (N <= 3 || !shouldUpdate) {
        return false;
    }
    return true;
}