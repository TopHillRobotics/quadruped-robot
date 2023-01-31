/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: a interface of robot locomotion controller.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "controllers/locomotion_controller.h"
namespace Quadruped {
    LocomotionController::LocomotionController(Robot *robotIn,
                                               GaitGenerator *gaitGeneratorIn,
                                               DesiredStateCommand* desiredStateCommandIn,
                                               StateEstimatorContainer<float> *stateEstimatorIn,
                                               ComAdjuster *comAdjusterIn,
                                               PosePlanner *posePlannerIn,
                                               RaibertSwingLegController *swingLegControllerIn,
                                               StanceLegControllerInterface *stanceLegControllerIn,
                                               UserParameters *userParameters)
    :robot(robotIn), gaitGenerator(gaitGeneratorIn), desiredStateCommand(desiredStateCommandIn), stateEstimator(stateEstimatorIn),
        comAdjuster(comAdjusterIn), posePlanner(posePlannerIn), swingLegController(swingLegControllerIn), stanceLegController(stanceLegControllerIn)
    {
        resetTime = robot->GetTimeSinceReset();
        timeSinceReset = 0.;
        BindCommand();
    }

    void LocomotionController::Reset()
    {
        resetTime = robot->GetTimeSinceReset();
        timeSinceReset = 0.;
        gaitGenerator->Reset(timeSinceReset);
        comAdjuster->Reset(timeSinceReset);
        posePlanner->Reset(timeSinceReset);
        swingLegController->Reset(timeSinceReset);
        stanceLegController->Reset(timeSinceReset);
        BindCommand();
    }

    void LocomotionController::Update()
    {
        if (!robot->stop) { // not stop = (swingSemaphore > 0) or  (swingSemaphore=0 but not switchToSwing)
            timeSinceReset = robot->GetTimeSinceReset() - resetTime;
        }
        // std::cout << "------ LocomotionController update time-------- " << std::setprecision(6) << timeSinceReset << std::endl;
        
        // Find the current gait schedule
        gaitGenerator->Update(timeSinceReset);
        
        bool switchToSwing = false;
        if (robot->controlParams["mode"]==LocomotionMode::WALK_LOCOMOTION) {
            // for walk mode
            const Vec4<int>& newLegState = gaitGenerator->legState;
            const Vec4<int>& curLegState = gaitGenerator->curLegState;
            for(int legId =0; legId<4; legId++) {
                if((newLegState(legId) == LegState::SWING && curLegState(legId) == LegState::STANCE)
                    || newLegState(legId) == LegState::USERDEFINED_SWING) {
                        switchToSwing=true;
                        break;
                    }
            }
            if (switchToSwing) {
                if (swingSemaphore > 0) {
                    swingSemaphore--;
                } else if (swingSemaphore == 0) 
                {
                    swingSemaphore--;
                    stopTick = robot->GetTimeSinceReset();
                    robot->stop = true; // when stop, all legs must stay stance if phase gap=0.25.
                    printf("stop robot!============\n");
                    posePlanner->ResetBasePose(timeSinceReset);
                } else { // swingSemaphore == -1
                    ;
                }
            }
        }
        
        switch (robot->controlParams["mode"]) {
            case LocomotionMode::POSITION_LOCOMOTION: {
                comAdjuster->Update(timeSinceReset);
            } break; 
            case LocomotionMode::WALK_LOCOMOTION: {
                if (switchToSwing && swingSemaphore >=0) {
                    posePlanner->Update(timeSinceReset);
                    printf("update pose plan finish\n");
                }
            } break;
            default: {
                // comAdjuster->Update(timeSinceReset);
                break;
            }
        }
        swingLegController->Update(timeSinceReset);
        stanceLegController->Update(timeSinceReset); // robot->GetTimeSinceReset() - resetTime
    }

    /*
    void LocomotionController::UpdateAdaptive()
    {
        timeSinceReset = robot->GetTimeSinceReset() - resetTime;
        gaitGenerator->Update(timeSinceReset);
        stateEstimator->Update(timeSinceReset);
        estimator->Update(timeSinceReset);
        comAdjuster->Update(timeSinceReset);
        Vec4<bool> canUpdateDuration = swingLegController->Update(timeSinceReset);
        
        footholdPlacementOption = detector->detect();
        stepVelocityController->UpdateSwingFootStepper(canUpdateDuration, footholdPlacementOption);
        if(canUpdateDuration.sum()>0){
            groundEstimator->Update(phaseSwitchFootPositionsInWorldFrame);
        }
        if(canControlStepTiming){
            stepVelocityController.Update(timeSinceReset, canUpdateDuration);
            for (int legId=0; legId<4; ++legId){
                if (canUpdateDuration[i]){
                    gaitGenerator->UpdateSwingStanceTime(legId, stepVelocityController->duration);
                }
            }
        }
        stanceLegController->Update(timeSinceReset);
    }
    */

    std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> LocomotionController::GetAction()
    {
        action.clear();
        // Returns the control ouputs (e.g. positions/torques) for all motors. type: map
        // MITTimer T1;
        auto swingAction = swingLegController->GetAction();
        // printf("swing SOLVE TIME: %.3f\n", T1.getMs());
        // robot->stateDataFlow.visualizer.sa[0].Update(T1.getMs());
        // std::cout << "swing action" << std::endl;
        // for (auto& it : swingAction) {
        //     std::cout << it.first << " = " << it.second.transpose() << std::endl;
        // }
        // std::cout << "**********" << std::endl;
        // float tik = robot->GetTimeSinceReset();
        // MITTimer T2;
        auto [stanceAction, qpSol] = stanceLegController->GetAction(); // map<int, MotorCommand>
        // printf("STANCE SOLVE TIME: %.3f\n", T2.getMs());
        // robot->stateDataFlow.visualizer.sa[1].Update(T2.getMs());
        
        // MITTimer T3;
        std::vector<MotorCommand> action;
        // copy motors' actions from subcontrollers to output variable.
        for (int joint_id = 0; joint_id < NumMotor; ++joint_id) {
            auto it = swingAction.find(joint_id);
            if (it != swingAction.end()) {
                action.push_back(it->second);
            } else {
                action.push_back(stanceAction[joint_id]);
            }
        }
        // printf("combine TIME: %.3f\n", T3.getMs());
        
        return {action, qpSol};
    }
    
    std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> LocomotionController::GetFakeAction()
    {
        action.clear();
        Eigen::Matrix<float, 3, 4> qpSol = Eigen::Matrix<float, 3, 4>::Zero();
        std::vector<MotorCommand> action;
        // copy motors' actions from subcontrollers to output variable.         
        for (int joint_id = 0; joint_id < NumMotor; ++joint_id) {
            action.push_back({0,0,0,0,0});
        }
        return {action, qpSol};
    }

    void LocomotionController::ForwardOne()
    {
        robot->stop = false;
        stop = false; 
        swingSemaphore++;
        if (swingSemaphore==0) {
            swingSemaphore++;
            resetTime += (robot->GetTimeSinceReset() - stopTick);
        }
    }
} // namespace Quadruped
