/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: define some action sequences for quadruped.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-3
* Notes: xx
* Modify: init the file. @ Zhao Yao
*/

#ifndef ASCEND_QUADRUPED_CPP_ACTION_H
#define ASCEND_QUADRUPED_CPP_ACTION_H

#include <math.h>
#include <algorithm>
#include "robots/robot.h"
#include "mpc_controller/locomotion_controller.h"

/**
 * @brief namespace Quadruped is used for subdirectory of ascend-quadruped-cpp. \n
 */
namespace Quadruped {
    namespace Action {

        void StandUp(Robot *robot, float standUpTime, float totalTime, float timeStep);

        void SitDown(Robot *robot, float sitDownTime, float timeStep);

        void KeepStand(Robot *robot, float KeepStandTime = 1.0, float timeStep = 0.001);
    
        void ControlFoot(Robot *robot, LocomotionController *locomotionController, float walkTime, float timeStep);
        
    } // Action
} // Quadruped
#endif //ASCEND_QUADRUPED_CPP_ACTION_H
