/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the ground of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/
#include "estimators/ground_estimator.h"

namespace Quadruped {
  
    GroundSurfaceEstimator::GroundSurfaceEstimator(Robot *robotIn, std::string terrainConfigPath)
        : robot(robotIn)
    {
        // Loadterrain(terrainConfigPath);
        footStepperConfig = YAML::LoadFile(terrainConfigPath);
        terrain.footHoldOffset = footStepperConfig["foothold_offset"].as<float>();
        robot->footHoldOffset = terrain.footHoldOffset;
        Reset(0.f);
        std::cout << "init groundEsitmator finish\n" << std::endl;
    }

    // void GroundSurfaceEstimator::Loadterrain(std::string& terrainConfigPath)
    // {
    //     // YAML::Node footStepperConfig = YAML::LoadFile(terrainConfigPath);
    //     terrain.terrainType = static_cast<TerrainType>(footStepperConfig["terrain_type"].as<int>());
    //     terrain.footHoldOffset = footStepperConfig["foothold_offset"].as<float>();
    //     robot->footHoldOffset = terrain.footHoldOffset;
    //     switch (terrain.terrainType) {
    //         case TerrainType::PLANE: {
    //             // terrain.costMap = terrain.block<N, N>(0, 0);
    //         } break;
    //         case TerrainType::PLUM_PILES: {
    //             float gapWidth = footStepperConfig["gap_width"].as<float>();
    //             std::vector<float> distanceOfGaps = footStepperConfig["gaps"].as<std::vector<float>>();
    //             for (float distance : distanceOfGaps) {
    //                 Gap* gap = new Gap(distance, gapWidth, {0, 0, 0});
    //                 terrain.gaps.push_back(gap);
    //             }
    //         } break;
    //         case TerrainType::STAIRS: {
    //             // todo
    //         } break;
    //         default : throw std::domain_error("no such terrain!");
    //     }
    // }

    void GroundSurfaceEstimator::Update(float currentTime)
    {   
        Eigen::Matrix<bool, 4, 1> contactState = robot->GetFootContacts();
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
        if (N <= 3 || !shouldUpdate) {
            return ;
        }
        // printf(" should Update ground!\n");
        Eigen::Matrix<double, 3, 4> footPositionsInBaseFrame = robot->GetFootPositionsInBaseFrame().cast<double>();
        // std::cout << "footPositionsInBaseFrame" << footPositionsInBaseFrame <<std::endl;
        bodyPositionInWorldFrame = robot->GetBasePosition(); 
        bodyPositionInWorldFrame[2] = robot->stateDataFlow.heightInControlFrame;
        pZ = footPositionsInBaseFrame.row(2);
        W.col(1) = footPositionsInBaseFrame.row(0);
        W.col(2) = footPositionsInBaseFrame.row(1);
        
        Mat3<double> ww = W.transpose()* W;
        a = ww.inverse()* W.transpose()*pZ;
        // std::cout << "a=" << a.transpose() << std::endl;
        GetNormalVector(true);
        ComputeControlFrame();
    }

    void GroundSurfaceEstimator::Reset(float currentTime) {
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
                    Gap* gap = new Gap(distance, gapWidth, {0, 0, 0});
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

    float GroundSurfaceEstimator::GetZInBaseFrame(float x, float y)
    {
        float z = a.transpose() * Vec3<double>(1, x, y);
        return z;
    }
    float GroundSurfaceEstimator::GetZInControlFrame(float x, float y)
    {
        // a1*x + a2*y + (-1)*z + a0 = 0  ==>
        // [a1, a2, -1, a0]^T * [x, y, z, 1] = 0
        // let A_b = [a1, a2, -1, a0], a_b = [a0, a1, a2] 
        // A_c = Rcb * A_b, where Rcb is a homogenous transformation.
        Vec4<double> AInControlFrame =  controlFrame * Vec4<double>(a[1],a[2],-1.f, a[0]);
        float z = (AInControlFrame[0]*x + AInControlFrame[1]*y + AInControlFrame[3]*1) / (-AInControlFrame[2]);
        return z;
    }
  
    Eigen::Matrix<double, 3, 1> GroundSurfaceEstimator::GetNormalVector(bool update)
    {
        if (update){
            double factor = std::sqrt(a[1]*a[1] + a[2]*a[2] + 1);
            n << -a[1], -a[2], 1.0;
            n /= factor;
        }

        // std::cout << "n=" << n.transpose() << std::endl;
        return n; // in base frame
    }

    Eigen::Matrix<double, 4, 4> GroundSurfaceEstimator::ComputeControlFrame()
    {
        // Vec3<float> comRPY = robot->GetBaseRollPitchYaw();
        Quat<double> quat = robot->GetBaseOrientation().cast<double>();
        Mat3<double> BaseR = robotics::math::quaternionToRotationMatrix(quat).transpose();
        Vec3<double> nInWorldFrame = robotics::math::invertRigidTransform<double>({0,0,0},quat, n);
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
        // std::cout << "[UPDATE] controlFrameRPY = " << controlFrameRPY.transpose()*180/3.1415 << std::endl;
        // filtering the controlframe rpy angles.
        
        // float comYaw = comRPY[2];
        // float cy = cos(comYaw);
        // float sy = sin(comYaw);
        // float pitch = atan(n[0]/(n[1]+1e-7)*cy+n[1]/n[2]*sy);
        // float roll = atan((sy*sin(pitch)-n[1]/n[2]*cos(pitch))/cy);
    
        // // float pitch = atan(n[0]/n[2]);
        // // float roll = atan(-n[1]/n[2]);
        
        // controlFrameRPY << roll, pitch, comYaw;
        controlFrameOrientation = robotics::math::rpyToQuat(controlFrameRPY);
        // Mat3<float> R = robotics::math::rpyToRotMat(controlFrameRPY).transpose();
        controlFrame.block<3,3>(0,0) = R;
        controlFrame.block<3,1>(0,3) = robot->GetBasePosition().cast<double>();
        ///
        robot->stateDataFlow.groundRMat = R.cast<float>();
        robot->stateDataFlow.groundOrientation = controlFrameOrientation.cast<float>();
        robot->stateDataFlow.baseRInControlFrame = robot->stateDataFlow.groundRMat.transpose() * robot->stateDataFlow.baseRMat;
        
        return controlFrame;
    }

    Eigen::Matrix<float, 3, 3> GroundSurfaceEstimator::GetAlignedDirections()
    {
        Mat3<float> R = controlFrame.block<3,3>(0,0).cast<float>();   
        return R;
    }
} // namespace Quadruped