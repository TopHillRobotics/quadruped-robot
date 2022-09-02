/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_FILTER_H
#define ASCEND_QUADRUPED_CPP_FILTER_H

#include <deque>
#include <cmath>
#include "config.h"
#define Nsta 3 // dimension of state
#define Mobs 3 // dimension of observation

/** @brief filtering a sequence of noisy measurment data. */
class qrMovingWindowFilter {
public:
    qrMovingWindowFilter();

    qrMovingWindowFilter(unsigned int windowSize);

    /**
     * @brief Update the moving window sum using Neumaier's algorithm.
     *        For more details please refer to:
     *           https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
     * @param Args:
     *          value: The new value to be added to the window.
     */
    void NeumaierSum(const double &value);

    double CalculateAverage(const double &newValue);

    inline double GetSum()
    {
        return sum;
    }

private:
    unsigned int moveWindowSize;
    double sum; // The moving window sum.
    // The correction term to compensate numerical precision loss during calculation.
    double correction;
    std::deque<double> valueDeque;
};

#endif // ASCEND_QUADRUPED_CPP_FILTER_H
