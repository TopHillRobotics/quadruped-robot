/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: The robot configration
* Author: Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao;
*       add linkId @ Zhu Yijie 2022-04-01;
*/

#ifndef ASCEND_QUADRUPED_CPP_CONFIG_H
#define ASCEND_QUADRUPED_CPP_CONFIG_H

#define A1_BODY_MASS 10
#define Go1_BODY_MASS 10
#define DEFAULT_WINDOW_SIZE 20
#define A1_BODY_HIGHT 0.28  // for robot position init
#define NumLeg 4
#define NumMotor 12
#define BaseFreedomDim 6
#define MAX_TIME_SECONDS 1000.0f
#define JOY_CMD_VELX_MAX 0.3
#define JOY_CMD_VELY_MAX 0.3
#define JOY_CMD_YAW_MAX 0.3
#define JOY_CMD_ROLL_MAX 0.1
#define JOY_CMD_PITCH_MAX 0.1

const int NumMotorOfOneLeg = 3;
const float MAXIMUM_STEP = 0.3001f;

#endif //ASCEND_QUADRUPED_CPP_CONFIG_H
