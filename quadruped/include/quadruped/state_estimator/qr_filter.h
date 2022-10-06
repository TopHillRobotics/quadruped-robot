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

#ifndef QR_FILTER_H
#define QR_FILTER_H

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

#endif // QR_FILTER_H
