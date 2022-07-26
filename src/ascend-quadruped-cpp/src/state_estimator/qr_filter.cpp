/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/
#include "state_estimator/qr_filter.h"

namespace Quadruped {

    qrMovingWindowFilter::qrMovingWindowFilter()
    {
        moveWindowSize = DEFAULT_WINDOW_SIZE;
        sum = 0.;
        correction = 0.;
    }

    qrMovingWindowFilter::qrMovingWindowFilter(unsigned int windowSizeIn)
    {
        moveWindowSize = windowSizeIn;
        sum = 0.;
        correction = 0.;
    }

    void qrMovingWindowFilter::NeumaierSum(const double &value)
    {
        double newSum = sum + value;
        if (std::abs(sum) >= std::abs(value)) {
            // If self._sum is bigger, low-order digits of value are lost.
            correction += (sum - newSum) + value;

        } else {
            // low-order digits of sum are lost
            correction += (value - newSum) + sum;
        }
        sum = newSum;
    }

    /**
     * @brief Computes the moving window average in O(1) time.
     * @param Args:
     *   new_value: The new value to enter the moving window.
     * @return Returns:
     *   The average of the values in the window.
     */
    double qrMovingWindowFilter::CalculateAverage(const double &newValue)
    {
        auto dequeLen = valueDeque.size();
        if (dequeLen >= moveWindowSize) {
            // The left most value to be subtracted from the moving sum.
            NeumaierSum(-valueDeque[0]);
            valueDeque.pop_front();
        }

        NeumaierSum(newValue);
        valueDeque.push_back(newValue);
        return (sum + correction) / moveWindowSize;
    }
} // Quadruped
