// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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

    // TODO: add robot module
qrFootholdPlanner::qrFootholdPlanner(qrRobot *robotIn, qrGroundSurfaceEstimator *groundEsitmatorIn)
: robot(robotIn), groundEstimator(groundEsitmatorIn), terrain(groundEstimator->GetTerrain()),
    timeSinceReset(0.f)
{
    robotState = robot->GetRobotState();
    footstepper = new qrFootStepper(terrain, 0.10f, "optimal");
    terrain = groundEstimator->GetTerrain();
    Reset();
}

void qrFootholdPlanner::Reset() {
    resetTime = robot->GetTimeSinceReset();
    timeSinceReset = 0.f;
    footstepper->Reset(timeSinceReset);
    comPose << robotState->GetBasePosition(), robotState->GetRpy();
    desiredComPose = Vec6<float>::Zero();
    desiredFootholdsOffset = Mat3x4<float>::Zero();
}

void qrFootholdPlanner::UpdateOnce(Mat3x4<float> currentFootholds, std::vector<int> legIds) {
    comPose << robotState->GetBasePosition(), robotState->GetRpy();
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
        ComputeFootholdsOffset(currentFootholds);
    } else {
        ComputeNextFootholds(currentFootholds, comPose, desiredComPose, legIds);
    }
}

void qrFootholdPlanner::ComputeFootholdsOffset(Mat3x4<float> currentFootholds) {
    desiredFootholdsOffset = footstepper->GetOptimalFootholdsOffset(currentFootholds);
}

void qrFootholdPlanner::ComputeNextFootholds(Mat3x4<float>& currentFootholds,
                                             Vec6<float>& currentComPose,
                                             Vec6<float>& desiredComPose,
                                             std::vector<int>& legIds)
{
    auto res = footstepper->GetFootholdsInWorldFrame(currentFootholds, currentComPose, desiredComPose, legIds);
    desiredFootholds = std::get<0>(res);
    desiredFootholdsOffset = std::get<1>(res);
}
