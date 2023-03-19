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

#ifndef QR_SAFETY_CHECKER_H
#define QR_SAFETY_CHECKER_H

#include <iostream>

#include "fsm/qr_control_fsm_data.hpp"

/**
 * @brief The SafetyChecker handles the checks requested by the ControlFSM.
 */
template <typename T>

class qrSafetyChecker {

public:

    /**
     * @brief constructor of member qrSafetyChecker
     * @param dataIn: data needed for check, usually robot states and hybrid commands.
     */
    qrSafetyChecker(qrControlFSMData<T>* dataIn): data(dataIn) {
    };

    /**
     * @brief Check robobts' orientation is safe to control.
     * This is a pre-control check.
     * @return Whether it is safe.
     */
    bool CheckSafeOrientation();

    /**
     * @brief Check foot position not to be stepped too far.
     * This is a post-control check.
     * Currently this method is not used. Just return true.
     * @return Whether it is safe.
     */
    bool CheckPDesFoot();

    /**
     * @brief Check force not to be too large.
     * This is a post-control check.
     * @return Whether it is safe.
     */
    bool CheckForceFeedForward();

private:

    /**
     * @brief A pointer to FSM data. The data is a member of FSM.
     */
    qrControlFSMData<T>* data;
};

#endif // QR_SAFETY_CHECKER_H
