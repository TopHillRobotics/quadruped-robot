// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_MOVING_WINDOW_FILTER_H
#define QR_MOVING_WINDOW_FILTER_H

#include <deque>
#include <cmath>

#include "Eigen/Dense"

#include "qr_config.h"


#define Nsta 3 // dimension of state
#define Mobs 3 // dimension of observation


namespace Quadruped {

/**
 * @brief filtering a sequence of noisy measurment data.
*/
template<class T=double, int N=1>
class qrMovingWindowFilter {

public:

    /**
     * @brief Constructor of the class MovingWindowFilter.
     */
    qrMovingWindowFilter();

    /**
     * @brief Constructor of the class MovingWindowFilter.
     * @param windowSize: window size for moving window algorithm.
     */
    qrMovingWindowFilter(unsigned int windowSize);

    /**
     * @brief Reset some intermediate variable of the method, include sum, correction, deque.
     */
    void Reset();

    /**
     * @brief Update the moving window sum using Neumaier's algorithm.
     *        For more details please refer to:
     *           https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
     * @param Args:
     *          value: The new value to be added to the window.
     */
    void NeumaierSum(const Eigen::Matrix<T, N, 1> &value);

    /**
     * @brief Push a new value into the window queue,
     * and calculate the average value in the window queue.
     * @param newValue: push a new value into the window queue.
     */
    Eigen::Matrix<T, N, 1> CalculateAverage(const Eigen::Matrix<T, N, 1> &newValue);


    /**
     * @brief Getter method of member sum.
     */
    Eigen::Matrix<T, N, 1> GetSum() {
        return sum;
    };

private:

    /**
     * @brief Window size for moving window algorithm.
     */
    unsigned int moveWindowSize;

    /**
     * @brief The sum of values in the moving window queue.
     */
    Eigen::Matrix<T, N, 1> sum; // The moving window sum.

    /**
     * @brief The correction term to compensate numerical
     * precision loss during calculation.
     */
    Eigen::Matrix<T, N, 1> correction;

    /**
     * @brief Stores the moving window values.
     */
    std::deque<Eigen::Matrix<T, N, 1>> valueDeque;


};


template<class T, int N>
qrMovingWindowFilter<T, N>::qrMovingWindowFilter()
{
    moveWindowSize = DEFAULT_WINDOW_SIZE;
    for (int i = 0; i < N; ++i) {
        sum[i] = 0.;
        correction[i] = 0.;
    }
}


template<class T, int N>
qrMovingWindowFilter<T, N>::qrMovingWindowFilter(unsigned int windowSizeIn)
{
    moveWindowSize = windowSizeIn;
    for (int i = 0; i < N; ++i) {
        sum[i] = 0.;
        correction[i] = 0.;
    }
}


template<class T, int N>
void qrMovingWindowFilter<T, N>::Reset()
{
    for (int i = 0; i < N; ++i) {
        sum[i] = 0.;
        correction[i] = 0.;
    }
    valueDeque.clear();
}


template<class T, int N>
void qrMovingWindowFilter<T, N>::NeumaierSum(const Eigen::Matrix<T, N, 1> &value)
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
Eigen::Matrix<T, N, 1> qrMovingWindowFilter<T, N>::CalculateAverage(const Eigen::Matrix<T, N, 1> &newValue)
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

} // Namespace Quadruped

namespace Quadruped {

template<> class qrMovingWindowFilter<double, 1> {

public:

    /**
     * @brief Constructor of the class MovingWindowFilter.
     */
    qrMovingWindowFilter() {
        moveWindowSize = DEFAULT_WINDOW_SIZE;
        sum = 0.;
        correction = 0.;
    };

    /**
     * @brief Constructor of the class MovingWindowFilter.
     * @windowSizeIn: window size for the moving window algorithm.
     */
    qrMovingWindowFilter(unsigned int windowSizeIn) {
        moveWindowSize = windowSizeIn;
        sum = 0.;
        correction = 0.;
    };

    /**
     * @brief Reset some intermediate variable of the method,
     * include sum, correction, deque.
     */
    void Reset() {
        sum = 0.;
        correction = 0.;
        valueDeque.clear();
    };

    /**
     * @brief Update the moving window sum using Neumaier's algorithm.
     *        For more details please refer to:
     *           https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
     * @param value: The new value to be added to the window.
     */
    void NeumaierSum(const double &value) {
        double newSum = sum + value;
        if (std::abs(sum) >= std::abs(value)) {
            // If self._sum is bigger, low-order digits of value are lost.
            correction += (sum - newSum) + value;
        } else {
            // low-order digits of sum are lost
            correction += (value - newSum) + sum;
        }
        sum = newSum;
    };

    /**
     * @brief Push a new value into the window queue,
     * and calculate the average value in the window queue.
     * @param newValue: push a new value into the window queue.
     */
    double CalculateAverage(const double &newValue) {
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
    };

    /**
     * @brief Getter method of member sum
     */
    double GetSum() {
        return sum;
    };

private:

    /**
     * @brief Window size for moving window algorithm.
     */
    unsigned int moveWindowSize;

    /**
     * @brief The sum of values in the moving window queue.
     */
    double sum;

    /**
     * @brief The correction term to compensate numerical
     * precision loss during calculation.
     */
    double correction;

    /**
     * @brief Stores the moving window values.
     */
    std::deque<double> valueDeque;

};

} // Namespace Quadruped


#endif // QR_MOVING_WINDOW_FILTER_H
