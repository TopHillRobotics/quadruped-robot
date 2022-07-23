/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: motor object on robots.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*       mv implementation of func to .cpp file. @ Zhu Yijie 2021-11-24;
*/

#include "robots/motor.h"

namespace Quadruped {
    MotorCommand::MotorCommand(double pIn, double KpIn, double dIn, double KdIn, double tuaIn)
        : p(pIn), Kp(KpIn), d(dIn), Kd(KdIn), tua(tuaIn)
    {}

    MotorCommand::MotorCommand(const Eigen::Matrix<float, 5, 1> &cmd)
    {
        p = cmd[0];
        Kp = cmd[1];
        d = cmd[2];
        Kd = cmd[3];
        tua = cmd[4];
    }

    Eigen::Matrix<float, 5, 1> MotorCommand::convertToVector() const
    {
        Eigen::Matrix<float, 5, 1> HybridMotorCommandVector;
        HybridMotorCommandVector << p, Kp, d, Kd, tua;
        return HybridMotorCommandVector;
    }

    std::ostream &operator<<(std::ostream &os, MotorCommand &data)
    {
        os << data.p << " " << data.Kp << " " << data.d << " " << data.Kd <<
           " " << data.tua;
        return os;
    }
} // Quadruped