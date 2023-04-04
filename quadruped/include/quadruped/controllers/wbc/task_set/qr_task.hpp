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

#ifndef QR_TASK_H
#define QR_TASK_H

#include "dynamics/floating_base_model.hpp"


#define TK qrTask<T>


template<typename T>
class qrTask {

public:

    /**
     * @brief Constructor of class task.
     * @param dim: the dimension of the task.
     */
    qrTask(size_t dim):
        dimTask(dim),
        xddotCmd(dim),
        posErr(dim),
        desiredVel(dim),
        desiredAcc(dim) {
    }

    virtual ~qrTask() = default;

    /**
     * @brief Update the task constraint, including Jt, JtDotQdot and xddotCmd.
     * @param des_pos: desired position of the task.
     * @param des_vel: desired velocity of the task.
     * @param des_acc: desired acceleration of the task.
     * @return true if update has finished
     */
    bool UpdateTask(const void *des_pos, const DVec<T> &des_vel, const DVec<T> &des_acc)
    {
        UpdateTaskJacobian();
        UpdateTaskJDotQdot();
        UpdateCommand(des_pos, des_vel, des_acc);

        return true;
    }

    /**
     * @brief Getter method of member xddotCmd.
     */
    void GetXddotCmd(DVec<T> &xddot_cmd) const {
        xddot_cmd = this->xddotCmd;
    }

    /**
     * @brief Getter method of member Jt.
     */
    void GetJt(DMat<T> &Jt) const {
        Jt = this->Jt;
    }

    /**
     * @brief Getter method of member JtDotQdot.
     */
    void GetJtDotQdot(DVec<T> &JtDot_Qdot) const {
        JtDot_Qdot = this->JtDotQdot;
    }

    /**
     * @brief Getter method of member posErr.
     */
    const DVec<T> &GetPosErr() const {
        return posErr;
    }

    /**
     * @brief Getter method of member desiredVel.
     */
    const DVec<T> &GetDesiredVel() const {
        return desiredVel;
    }

    /**
     * @brief Getter method of member desiredAcc.
     */
    const DVec<T> &GetDesiredAcc() const {
        return desiredAcc;
    }

protected:

    /**
     * @brief Update the desired acceleration command or position error if needed.
     * @return true if update has finished.
     */
    virtual bool UpdateCommand(const void *pos_des, const DVec<T> &vel_des, const DVec<T> &acc_des) = 0;

    /**
     * @brief Update task jacobian.
     * @return true if update has finished.
     */
    virtual bool UpdateTaskJacobian() = 0;

    /**
     * @brief Update JtDotQdot. JtDotQDdot is usually from MIT floating base model.
     * @return true if update has finished.
     */
    virtual bool UpdateTaskJDotQdot() = 0;

    /**
     * @brief The dimension of the task.
     * If dim=3, the robot needs to satisfy command on all three directions.
     */
    size_t dimTask;

    /**
     * @brief The optimized acceleration command.
     * The acceleration command is computed from PD control of desired position and desired velocity.
     */
    DVec<T> xddotCmd;

    /**
     * @brief Derivative of Jt dot Derivative of q.
     * Used in null-space projection.
     */
    DVec<T> JtDotQdot;

    /**
     * @brief Task jacobian.
     */
    DMat<T> Jt;

    /**
     * @brief Position error of the task. Will be used in null-space projection.
     * Computed from (desired position/orientation - current  position/orientation.)
     */
    DVec<T> posErr;

    /**
     * @brief Desired velocity of the task.
     * Used in PD control to compute the acceleration command.
     */
    DVec<T> desiredVel;

    /**
     * @brief Desired acceleration of the task.
     * Used in PD control to compute the acceleration command.
     */
    DVec<T> desiredAcc;

    /**
     * @brief The dimension of the configuration space.
     * Including 6 dimension of floating base and 12 dimension of joints.
     */
    int dimConfig = 18;

};

#endif // QR_TASK_H
