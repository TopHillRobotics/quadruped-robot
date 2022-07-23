/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: The robot runtime used to lanuch the robot
* Author: Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao;
*       add MAX_TIME_SECONDS @ Zhu Yijie 2021-11-24;
*/

#ifndef ASCEND_QUADRUPED_CPP_CONFIG_H
#define ASCEND_QUADRUPED_CPP_CONFIG_H

#define A1_BODY_MASS 10
#define Go1_BODY_MASS 10
#define DEFAULT_WINDOW_SIZE 20
#define A1_BODY_HIGHT 0.27  // for robot position init
#define NumLeg 4
const int numMotorOfOneLeg = 3;
const float MAXIMUM_STEP = 0.3001f;

#endif //ASCEND_QUADRUPED_CPP_CONFIG_H
