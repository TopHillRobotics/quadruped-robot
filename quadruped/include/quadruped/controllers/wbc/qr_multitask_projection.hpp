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

#ifndef QR_MULTITASK_PROJECTION_H
#define QR_MULTITASK_PROJECTION_H

#include "qr_single_contact.hpp"
#include "task_set/qr_task.hpp"
#include "fsm/qr_control_fsm_data.hpp"


template<typename T>
class qrMultitaskProjection {

public:

    /**
     * @brief Consutructor of class qrMultitaskProjection.
     * @param dim_qdot: dimension of qdot.
     */
    qrMultitaskProjection(size_t dim_qdot);

    ~qrMultitaskProjection() = default;

    /**
     * @brief Compute the contact jacobian and update the configuration.
     * Find desired joint position and velocity for joint PD controller.
     * @param [in] curr_config: currunt joint states.
     * @param [in] task_list: prioritized tasks for robot.
     * @param [in] contact_list: contact constraint for all legs of robot.
     * @param [out] des_pos_cmd: desired joint position.
     * @param [out] des_vel_cmd: desired joint velocity.
     * @return if the work finished successfully.
     */
    bool FindConfiguration(const DVec<T> &curr_config,
                           const std::vector<qrTask<T> *> &task_list,
                           const std::vector<qrSingleContact<T> *> &contact_list,
                           DVec<T> &des_pos_cmd,
                           DVec<T> &des_vel_cmd);

private:

    /**
     * @brief Compute pseudo inverse of matrix J.
     * @param [in] J: input matrix.
     * @param [out] Jinv: inverse matrix of J.
     */
    void PseudoInverse(const DMat<T> &J, DMat<T> &Jinv);

    /**
     * @brief Compute the null-space projection matrix of J.
     * @param [in] J: previous jacobian matrix.
     * @param [out] N: null-space projection matrix.
     */
    void BuildProjectionMatrix(const DMat<T> &J, DMat<T> &N);

    /**
     * @brief Threshold for singular values being zero.
     * Used in computing the pseudo inverse matrix.
     */
    double thresholdInv;

    /**
     * @brief Dimension of qdot.
     */
    const size_t dimQDot;

    /**
     * @brief Number of actuable joints.
     */
    const size_t numActJoint;

    /**
     * @brief Identity matrix used in computing null-space projection.
     */
    DMat<T> identityMat;

    /**
     * @brief Null-space projection matrix of contact jacobian matrix.
     */
    DMat<T> Nc;

};

#endif // QR_MULTITASK_PROJECTION_H
