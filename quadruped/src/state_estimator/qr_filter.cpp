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
#include "state_estimator/qr_filter.h"

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
