/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua
* Create: 2021-12-13
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#include "controllers/mpc/torque_stance_leg_controller_mpc.h"

namespace Quadruped {

    TorqueStanceLegControllerMPC::TorqueStanceLegControllerMPC(Robot *robot,
                                                         GaitGenerator *gaitGenerator,
                                                         StateEstimatorContainer<float>* stateEstimators,
                                                         ComAdjuster *comAdjuster,
                                                         PosePlanner *posePlanner,
                                                         FootholdPlanner *footholdPlanner,
                                                         UserParameters& userParameters,
                                                         std::string configFilepath
                                                         )
    : TorqueStanceLegController(robot, gaitGenerator, stateEstimators, comAdjuster,posePlanner, 
    footholdPlanner, userParameters, configFilepath)
    {
        std::vector<float> v = param["stance_leg_params"][controlModeStr]["X_weight"].as<std::vector<float>>();
        this->XWeight = Eigen::MatrixXf::Map(&v[0], 13, 1);
    }

    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> TorqueStanceLegControllerMPC::GetAction()
    {
        count++;
        computeForceInWorldFrame = true;
        UpdateDesCommand();

        Eigen::Matrix<float, 13, 1> robotX;
        Eigen::Matrix<float, 13, 1> desiredX;
        
        robotX << desiredStateCommand->stateCur.segment(3, 3),   // robotComRpy
                    desiredStateCommand->stateCur.segment(0, 3), // robotComPosition
                    desiredStateCommand->stateCur.segment(9, 3), // robotComRpyRate
                    desiredStateCommand->stateCur.segment(6, 3), // robotComVelocity
                    -9.8f;
        std::cout << "robotX " <<robotX.transpose() << std::endl;
        
        desiredX << desiredStateCommand->stateDes.segment(3, 3), // desiredComRpy
                    desiredStateCommand->stateDes.segment(0, 3), // desiredComPosition
                    desiredStateCommand->stateDes.segment(9, 3), // desiredComAngularVelocity
                    desiredStateCommand->stateDes.segment(6, 3), // desiredComVelocity
                    -9.8f;
        std::cout << "desiredX " <<desiredX.transpose() << std::endl;
        
        if (robot->controlParams["mode"] ==  LocomotionMode::WALK_LOCOMOTION && 
                ((N<4 && moveBasePhase < 0.7) || robot->stop)) {
            contactForcesMPC << ComputeContactForceMPC(robot, robotX, desiredX, Eigen::Matrix<bool,4,1>::Constant(1), XWeight);
        } else {
            contactForcesMPC << ComputeContactForceMPC(robot, robotX, desiredX, contacts, XWeight);
        }
        
        for(int step = 0; step < predHorizon; ++step) {
            contactForcesMPC.block<3,4>(step*3,0) = robotics::math::RigidTransform<float,4>({0.f, 0.f,0.f}, 
                                                        robot->baseOrientation, contactForcesMPC.block<3,4>(step*3, 0));
        }
        Eigen::Matrix<float, 3, 4> contactForces = contactForcesMPC.block<3, 4>(0, 0);
        // std::cout << "contactForces = \n" << contactForces << std::endl;
        mpcStep++;
        
        std::map<int, MotorCommand> action;
        std::map<int, float> motorTorques;
        for (int legId = 0; legId < NumLeg; ++legId) {
            motorTorques = this->robot->MapContactForceToJointTorques(legId, contactForces.col(legId));
            for (std::map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
                MotorCommand temp{0., 0., 0., 0., it->second};
                action[it->first] = temp;
            }
        }
        std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> actionContactForce(action, contactForces);
        return actionContactForce;
    }
}