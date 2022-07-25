/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "planner/foothold_planner.h"

namespace Quadruped {
    qrFootholdPlanner::qrFootholdPlanner(qrRobot *robotIn, qrGroundSurfaceEstimator *groundEsitmatorIn)
    : robot(robotIn), groundEsitmator(groundEsitmatorIn), terrain(groundEsitmator->terrain),
        timeSinceReset(0.f), stepCount(0)
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
} // namespace Quadruped