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

namespace Quadruped {
    /** @brief filtering a sequence of noisy measurment data. */
    template<class T=double, int N=1>
    class MovingWindowFilter {
    public:
        MovingWindowFilter();

        MovingWindowFilter(unsigned int windowSize);

        void Reset();

        /**
         * @brief Update the moving window sum using Neumaier's algorithm.
         *        For more details please refer to:
         *           https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
         * @param Args:
         *          value: The new value to be added to the window.
         */
        void NeumaierSum(const Eigen::Matrix<T, N, 1> &value);

        Eigen::Matrix<T, N, 1> CalculateAverage(const Eigen::Matrix<T, N, 1> &newValue);

        Eigen::Matrix<T, N, 1> GetSum()
        {
            return sum;
        }

    private:
        unsigned int moveWindowSize;
        Eigen::Matrix<T, N, 1> sum; // The moving window sum.
        // The correction term to compensate numerical precision loss during calculation.
        Eigen::Matrix<T, N, 1> correction;
        std::deque<Eigen::Matrix<T, N, 1>> valueDeque;
    };
} // Quadruped

/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------------- */
namespace Quadruped {
    template<> class MovingWindowFilter<double, 1> {
    public:
        MovingWindowFilter()
        {
            moveWindowSize = DEFAULT_WINDOW_SIZE;
            sum = 0.;
            correction = 0.;
        }

        MovingWindowFilter(unsigned int windowSizeIn)
        {
            moveWindowSize = windowSizeIn;
            sum = 0.;
            correction = 0.;
        }

        void Reset()
        {
            sum = 0.;
            correction = 0.;
            valueDeque.clear();
        }

        void NeumaierSum(const double &value)
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

        double CalculateAverage(const double &newValue)
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
            return (sum + correction) / (dequeLen+1);
        }

        double GetSum()
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
}


#include "estimators/filter.hxx"

#endif // ASCEND_QUADRUPED_CPP_FILTER_H