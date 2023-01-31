/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_FILTER_HXX
#define ASCEND_QUADRUPED_CPP_FILTER_HXX
// #include "estimators/filter.h"

namespace Quadruped {
    template<class T, int N>
    MovingWindowFilter<T, N>::MovingWindowFilter()
    {
        moveWindowSize = DEFAULT_WINDOW_SIZE;
        for (int i = 0; i < N; ++i) {
            sum[i] = 0.;
            correction[i] = 0.;
        }
        
    }

    template<class T, int N>
    MovingWindowFilter<T, N>::MovingWindowFilter(unsigned int windowSizeIn)
    {
        moveWindowSize = windowSizeIn;
        for (int i = 0; i < N; ++i) {
            sum[i] = 0.;
            correction[i] = 0.;
        }
    }

    template<class T, int N>
    void MovingWindowFilter<T, N>::Reset()
    {
        for (int i = 0; i < N; ++i) {
            sum[i] = 0.;
            correction[i] = 0.;
        }
        valueDeque.clear();
    }

    template<class T, int N>
    void MovingWindowFilter<T, N>::NeumaierSum(const Eigen::Matrix<T, N, 1> &value)
    {
        Eigen::Matrix<T, N, 1> newSum = sum + value;
        for (int i = 0; i < N; ++i) {
            if (std::abs(sum[i]) >= std::abs(value[i])) {
            // If self._sum is bigger, low-order digits of value are lost.
            correction[i] += (sum[i] - newSum[i]) + value[i];
            } else {
                // low-order digits of sum are lost
                correction[i] += (value[i] - newSum[i]) + sum[i];
            }
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
    template<class T, int N>
    Eigen::Matrix<T, N, 1> MovingWindowFilter<T, N>::CalculateAverage(const Eigen::Matrix<T, N, 1> &newValue)
    {
        int dequeLen = valueDeque.size();
        if (dequeLen >= moveWindowSize) {
            // The left most value to be subtracted from the moving sum.
            NeumaierSum(-valueDeque[0]);
            valueDeque.pop_front();
            dequeLen--;
        }

        NeumaierSum(newValue);
        valueDeque.push_back(newValue);
        return (sum + correction) / (dequeLen + 1);
    }
} // Quadruped

#endif // ASCEND_QUADRUPED_CPP_FILTER_HXX