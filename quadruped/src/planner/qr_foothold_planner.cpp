
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

#include "planner/qr_foothold_planner.h"

qrFootholdPlanner::qrFootholdPlanner(qrRobot *robotIn, qrGroundSurfaceEstimator *groundEsitmatorIn)
: robot(robotIn), groundEsitmator(groundEsitmatorIn), terrain(groundEsitmator->terrain),
    timeSinceReset(0.f)
{
    footstepper = new qrFootStepper(terrain, 0.10f, "optimal");
    Reset();
}

void qrFootholdPlanner::Reset()
{
    resetTime = robot->GetTimeSinceReset();
    timeSinceReset = 0.f;
    footstepper->Reset(timeSinceReset);
    comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
    desiredComPose = Eigen::Matrix<float, 6, 1>::Zero();
    desiredFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
}

void qrFootholdPlanner::UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, std::vector<int> legIds)
{
    comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
    desiredComPose << 0.f,0.f,0.f,0.f,0.f,0.f; //comPose;
    desiredFootholds = currentFootholds;
    if (legIds.empty()) { // if is empty, update all legs.
        legIds = {0,1,2,3};
    } else {
        std::cout<<"update foothold of Legs : ";
        for(int legId : legIds) {
            std::cout << legId << " ";
        }
        std::cout << "\n";
    }
    
    if (terrain.terrainType != TerrainType::STAIRS) { 
        ComputeFootholdsOffset(currentFootholds, comPose, desiredComPose, legIds);
    } else {
        ComputeNextFootholds(currentFootholds, comPose, desiredComPose, legIds);
    }
}

Eigen::Matrix<float, 3, 4> qrFootholdPlanner::ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                                Eigen::Matrix<float, 6, 1> currentComPose,
                                                                Eigen::Matrix<float, 6, 1> desiredComPose,
                                                                std::vector<int> legIds)
{
    desiredFootholdsOffset = footstepper->GetOptimalFootholdsOffset(currentFootholds);
    return desiredFootholdsOffset;
}

Eigen::Matrix<float, 3, 4> qrFootholdPlanner::ComputeNextFootholds(Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                                    Eigen::Matrix<float, 6, 1>& currentComPose,
                                                                    Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                                    std::vector<int>& legIds)
{
        
    auto res = footstepper->GetFootholdsInWorldFrame(currentFootholds, currentComPose, desiredComPose, legIds);
    desiredFootholds = std::get<0>(res);
    desiredFootholdsOffset = std::get<1>(res);
    return desiredFootholdsOffset;
}