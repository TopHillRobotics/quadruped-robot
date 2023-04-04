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

#include "controllers/wbc/qr_multitask_projection.hpp"


template<typename T>
qrMultitaskProjection<T>::qrMultitaskProjection(size_t dim_qdot):
    thresholdInv(0.001), dimQDot(dim_qdot), numActJoint(dim_qdot - 6)
{
    identityMat = DMat<T>::Identity(dimQDot, dimQDot);
    Nc = DMat<T>::Identity(dimQDot, dimQDot);
}


template<typename T>
bool qrMultitaskProjection<T>::FindConfiguration(
    const DVec<T> &curr_config,
    const std::vector<qrTask<T> *> &task_list,
    const std::vector<qrSingleContact<T> *> &contact_list,
    DVec<T> &des_pos_cmd,
    DVec<T> &des_vel_cmd)
{
    Nc.setIdentity();

    if (!contact_list.empty()) {
        DMat<T> Jc, Jc_i;
        size_t num_rows = 0;

        /* Construct the contact jacobian Jc. */
        for (size_t i = 0; i < contact_list.size(); ++i) {
            contact_list[i]->GetJc(Jc_i);
            size_t num_new_rows = Jc_i.rows();
            Jc.conservativeResize(num_rows + num_new_rows, dimQDot);
            Jc.block(num_rows, 0, num_new_rows, dimQDot) = Jc_i;
            num_rows += num_new_rows;
        }
        /* Get the projection matrix Nc of contact jacobian matrix, */
        BuildProjectionMatrix(Jc, Nc);
    }

    /* Get first delta_q and q_dot that satisfying the contact constraint. */
    DVec<T> delta_q, qdot;
    DMat<T> Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    qrTask<T> *task = task_list[0];
    task->GetJt(Jt);
    JtPre.noalias() = Jt * Nc;
    PseudoInverse(JtPre, JtPre_pinv);
    
    delta_q = JtPre_pinv * (task->GetPosErr());
    qdot = JtPre_pinv * (task->GetDesiredVel());
    DVec<T> prev_delta_q = delta_q;
    DVec<T> prev_qdot = qdot;

    BuildProjectionMatrix(JtPre, N_nx);
    N_pre.noalias() = Nc * N_nx;

    /* Iteration for satisfying the rest tasks. */
    for (size_t i(1); i < task_list.size(); ++i) {
        task = task_list[i];
        task->GetJt(Jt);
        JtPre.noalias() = Jt * N_pre;
        PseudoInverse(JtPre, JtPre_pinv);
        
        /* delta_q_cmd(i) = delta_q(i-1) + Ji_prev_inv * ( delta_x - Ji * delta_q_cmd(i - 1). */
        delta_q = prev_delta_q + JtPre_pinv * (task->GetPosErr() - Jt * prev_delta_q);

        /* q_dot_cmd(i) = q_dot(i-1) + Ji_prev_inv * ( x_dot(i) - Ji * q_dot_cmd(i - 1). */
        qdot = prev_qdot + JtPre_pinv * (task->GetDesiredVel() - Jt * prev_qdot);

        /* Preparing N_prev, prev_delta_q and prev_qdot for next task. */
        if (i < task_list.size()-1) {
            BuildProjectionMatrix(JtPre, N_nx);
            N_pre *= N_nx;
            prev_delta_q = delta_q;
            prev_qdot = qdot;
        }
    }

    /* Get desired position and velocity command of motors. */
    des_pos_cmd = curr_config.segment(6, NumMotor) + delta_q.segment(6, NumMotor);
    des_vel_cmd = qdot.segment(6, NumMotor);
    return true;
}


template<typename T>
void qrMultitaskProjection<T>::BuildProjectionMatrix(const DMat<T> &J, DMat<T> &N)
{
    DMat<T> J_pinv;
    PseudoInverse(J, J_pinv);
    N.noalias() = identityMat - J_pinv * J;
}


template<typename T>
void qrMultitaskProjection<T>::PseudoInverse(const DMat<T> &J, DMat<T> &Jinv)
{
    pseudoInverse(J, thresholdInv, Jinv);
}

template class qrMultitaskProjection<float>;
