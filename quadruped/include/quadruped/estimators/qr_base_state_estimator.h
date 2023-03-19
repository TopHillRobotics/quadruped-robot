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

#ifndef QR_BASE_STATE_ESTIMATOR_H
#define QR_BASE_STATE_ESTIMATOR_H

#include "robots/qr_robot.h"


/**
 * @brief All Estimators should inherit from this class.
 * base class of all generic estimator.
 */
class qrBaseStateEstimator {

public:

    /**
     * @brief Update the estimator.
     */
    virtual void Update(float currentTime) = 0;

    /**
     * @brief Reset the estimator.
     */
    virtual void Reset(float currentTime) = 0;

    /**
     * @brief Destructor of the base estimator class.
     */
    virtual ~qrBaseStateEstimator() = default;

};

#endif // QR_BASE_STATE_ESTIMATOR_H
