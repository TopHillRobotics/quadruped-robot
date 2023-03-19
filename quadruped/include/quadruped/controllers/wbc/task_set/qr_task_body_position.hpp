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

#ifndef QR_TASK_BODY_POSITION_H
#define QR_TASK_BODY_POSITION_H

#include "qr_task.hpp"


template<typename T>
class qrTaskBodyPosition: public qrTask<T> {

public:

    /**
     * @brief Constructor of class qrTaskBodyPosition .
     * @param fbModel: pointer to MIT floating base model.
     */
    qrTaskBodyPosition(const FloatingBaseModel<T> *fb_model);

    virtual ~qrTaskBodyPosition() = default;

    /**
     * @brief A scale factor that will mutiply posErr.
     */
    DVec<T> errScale;

    /**
     * @brief KP for position gains.
     * Used in PD control to get acceleration command.
     */
    DVec<T> Kp;

    /**
     * @brief KP for velocity gains
     * Used in PD control to get acceleration command.
     */
    DVec<T> Kd;

protected:

    /**
     * @see qrTask::UpdateCommand
     */
    virtual bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des);

    /**
     * @see qrTask::UpdateTaskJacobian
     */
    virtual bool UpdateTaskJacobian();

    /**
     * @see qrTask::UpdateTaskJDotQdot
     * @attention In bod orientation task, just set JDotQdot to zero matrix.
     */
    virtual bool UpdateTaskJDotQdot();

    /**
     * @brief Pointer to MIT floating base model.
     * Used to get Jc and JDotQDot.
     */
    const FloatingBaseModel<T> *fbModel;

};

#endif // QR_TASK_BODY_POSITION_H
