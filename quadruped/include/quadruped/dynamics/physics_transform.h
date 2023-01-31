/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: some physics-relatived computation func.
* Author: Zhu Yijie
* Create: 2022-03-15
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_DYNAMICS_TRANSFORM_H
#define ASCEND_QUADRUPED_CPP_DYNAMICS_TRANSFORM_H

#include "utils/cppTypes.h"
#include "utils/se3.h"

namespace Quadruped {
    /** @brief transform rigid body rotaion inertia.
     * @param Mat3<float> inertia : interia in intric body frame, that is origin at COM
     * @param float mass : body mass
     * @param Vec3<float> p : the reference point reletive to COM where interia is represented.
     * @param Mat3<float> R : the orientation of new frame relative to Body frame, where inertia is represented.
     * @result Mat3<float> : new inertia matrix
     */
    Mat3<float> transformInertia(Mat3<float> inertia, float mass, Vec3<float> p, Mat3<float> R=Mat3<float>::Identity());


}

#endif //ASCEND_QUADRUPED_CPP_DYNAMICS_TRANSFORM_H
