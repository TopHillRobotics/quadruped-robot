/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua
* Create: 2021-12-13
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#include "mpc_controller/torque_stance_leg_controller_mpc.h"

using namespace std;
namespace Quadruped {

    TorqueStanceLegControllerMPC::TorqueStanceLegControllerMPC(Robot *robot,
                                                         OpenloopGaitGenerator *gaitGenerator,
                                                         RobotEstimator *robotEstimator,
                                                         GroundSurfaceEstimator *groundEstimatorIn,
                                                         ComAdjuster *comAdjuster,
                                                         PosePlanner *posePlanner,
                                                         FootholdPlanner *footholdPlanner,
                                                         Eigen::Matrix<float, 3, 1> desiredSpeed,
                                                         float desiredTwistingSpeed,
                                                         float desiredBodyHeight,
                                                         int numLegs,
                                                         std::string configFilepath,
                                                         std::vector<float> frictionCoeffs)
    : TorqueStanceLegController(robot, gaitGenerator, robotEstimator,groundEstimatorIn, comAdjuster,posePlanner, 
    footholdPlanner,desiredSpeed, desiredTwistingSpeed, desiredBodyHeight, numLegs, configFilepath, frictionCoeffs)
    {
        YAML::Node param = YAML::LoadFile(configFilepath);
        vector<float> v = param["stance_leg_params"]["X_weight"].as<vector<float >>();
        this->XWeight = Eigen::MatrixXf::Map(&v[0], 13, 1);
    }

    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> TorqueStanceLegControllerMPC::GetAction()
    {
        Eigen::Matrix<int, 4, 1> contacts;
        Eigen::Matrix<float, 3, 1> robotComPosition = Eigen::Matrix<float, 3, 1>::Zero();
        Eigen::Matrix<float, 3, 1> robotComVelocity;
        Eigen::Matrix<float, 3, 1> robotComRpy;
        Eigen::Matrix<float, 3, 1> robotComRpyRate;
        // Eigen::Matrix<float, 6, 1> robotQ;
        // Eigen::Matrix<float, 6, 1> robotDq;
        Eigen::Matrix<float, 13, 1> robotX;

        Eigen::Matrix<float, 3, 1> desiredComPosition(0.0, 0.0, 0.0);
        Eigen::Matrix<float, 3, 1> desiredComVelocity(0.0, 0.0, 0.0);
        Eigen::Matrix<float, 3, 1> desiredComRpy(0.0, 0.0, 0.0);
        Eigen::Matrix<float, 3, 1> desiredComAngularVelocity(0.0, 0.0, 0.0);
        // Eigen::Matrix<float, 6, 1> desiredQ;
        // Eigen::Matrix<float, 6, 1> desiredDq;
        Eigen::Matrix<float, 13, 1> desiredX;
        Eigen::Matrix<float, 3, 4> contactForces;

        /// leg contact status  ///
        float normallizedPhase = 1;
        int N=0;
        /// leg contact status  ///
        for (int legId = 0; legId < 4; legId++) {
            int legState = gaitGenerator->desiredLegState[legId];
            if (legState == LegState::STANCE || legState == LegState::EARLY_CONTACT) {
                contacts[legId] = true;
                N++;
            } else {
                contacts[legId] = false;
                normallizedPhase = gaitGenerator->normalizedPhase[legId];
            }
        }
        Eigen::Matrix<float, 3, 4> footPoseWorld = robot->state.GetFootPositionsInWorldFrame();
        // std::cout << "footPoseWorld = " << footPoseWorld <<std::endl;
        
        Vec6<float> pose;
        Vec6<float> twist; // v, wb
        if (!robot->stop) {
            auto res = posePlanner->GetIntermediateBasePose(normallizedPhase, currentTime); //todo  in world frame;
            pose = std::get<0>(res);
            twist = std::get<1>(res);
        } else {
            printf("80\n");
            auto res = posePlanner->GetIntermediateBasePose(currentTime);
            pose = std::get<0>(res);
            twist = std::get<1>(res);  
        }
        
        count++;
        // robotComPosition = {0., 0., robot->basePosition[2]};
        robotComPosition = robot->GetBasePosition(); // todo in world, walk mode
        robotComRpy = robot->GetBaseRollPitchYaw();
        Quat<float> robotComOrientation = robot->GetBaseOrientation();        
        robotComRpyRate = robot->GetBaseRollPitchYawRate();
        robotComRpyRate = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComRpyRate); // in world frame
        
        robotComVelocity = robotEstimator->GetEstimatedVelocity(); //base frame        
        robotComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, robotComVelocity); // in world frame
        robotX << robotComRpy, robotComPosition, robotComRpyRate, robotComVelocity, -9.8;
        // std::cout << "robotX " <<robotX.transpose() << std::endl;
        desiredComPosition = pose.head(3); //{0, 0, desiredBodyHeight};
        desiredComVelocity = twist.head(3); // {desiredSpeed[0], desiredSpeed[1], 0.};
        // desiredComVelocity = robotics::math::invertRigidTransform({0,0,0}, robotComOrientation, desiredComVelocity); // in world frame
        // float pitch = min( 0.2/3000.0*count, 0.2);
        // float roll = min( 0.2/3000.0*count, 0.2);
        // desiredComRpy << 0, 0, 0;
        desiredComRpy = pose.tail(3);
        desiredComAngularVelocity = twist.tail(3);
        // desiredComAngularVelocity << 0,0, desiredTwistingSpeed;
        desiredX << desiredComRpy, desiredComPosition, desiredComAngularVelocity, desiredComVelocity, -9.8;
        // std::cout << "desiredX " <<desiredX.transpose() << std::endl;
        std::cout << "base pose = " << robot->GetBasePosition().transpose() << " " << robot->GetBaseRollPitchYaw().transpose() << endl;
        std::cout << "plan pose = " << pose.transpose() << std::endl;
        std::cout << "normallizedPhase = " << normallizedPhase << std::endl;
            
        std::cout << "contact status: " << contacts.transpose() << std::endl;
        std::cout << "lastLegState =  " << gaitGenerator->lastLegState.transpose() <<std::endl;
        std::cout << "foot  force  =  " << robot->state.footForce.transpose() << std::endl;
                
        
        if (robot->controlParams["mode"] ==  LocomotionMode::WALK_LOCOMOTION && 
        ((N<4 && normallizedPhase < 0.7) || robot->stop)) {
            contactForcesMPC << ComputeContactForceMPC(robot, robotX, desiredX, Eigen::Matrix<int,4,1>::Constant(1), XWeight);
        } else {
            contactForcesMPC << ComputeContactForceMPC(robot, robotX, desiredX, contacts, XWeight);
        }

        
        for(int step =0; step<predHorizon; ++step) {
            contactForcesMPC.block<3,4>(step*3,0) = robotics::math::RigidTransform<float,4>({0.f, 0.f,0.f}, 
                                                        robotComOrientation, contactForcesMPC.block<3,4>(step*3,0));
        }
        contactForces = contactForcesMPC.block<3, 4>(0, 0);
        std::cout << "contactForces" << contactForces << std::endl;
        mpcStep++;
        //
        map<int, MotorCommand> action;
        map<int, float> motorTorques;
        for (int legId = 0; legId < 4; ++legId) {
            motorTorques = this->robot->state.MapContactForceToJointTorques(legId, contactForces.col(legId));
            for (map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
                MotorCommand temp{0., 0., 0., 0., it->second};
                action[it->first] = temp;
            }
        }
        std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> actionContactForce(action, contactForces);
        // printf("mpc cost time = %f ms\n", 1000.0*(robot->GetTimeSinceReset() - currentTime));
        return actionContactForce;
    }
}
