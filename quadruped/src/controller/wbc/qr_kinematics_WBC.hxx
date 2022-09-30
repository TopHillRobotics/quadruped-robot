// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

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

#include "controller/wbc/qr_kinematics_WBC.hpp"

// template <typename T>
// KinWBC<T>::KinWBC(size_t num_qdot)
//     : threshold_(0.001), num_qdot_(num_qdot), num_act_joint_(num_qdot - 6) {
//     I_mtx = DMat<T>::Identity(num_qdot_, num_qdot_);
// }

// template <typename T>
// bool KinWBC<T>::FindConfiguration(
//     const DVec<T>& curr_config, const std::vector<Task<T>*>& task_list,
//     const std::vector<qrContactSpec<T>*>& contact_list, DVec<T>& jpos_cmd,
//     DVec<T>& jvel_cmd,
//     ControlFSMData<T>* controlFSMData)
// {
//     // Contact Jacobian Setup
//     DMat<T> Nc = DMat<T>::Identity(num_qdot_, num_qdot_);
//     if(!contact_list.empty()){
//         DMat<T> Jc, Jc_i;
//         contact_list[0]->getContactJacobian(Jc);
//         size_t num_rows = Jc.rows();

//         for (size_t i(1); i < contact_list.size(); ++i) {
//         contact_list[i]->getContactJacobian(Jc_i);
//         size_t num_new_rows = Jc_i.rows();
//         Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
//         Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
//         num_rows += num_new_rows;
//         }

//         // Projection Matrix
//         _BuildProjectionMatrix(Jc, Nc);
//     }
//     // MITTimer T312;
    
//     // First Task
//     DVec<T> delta_q, qdot;
//     DMat<T> Jt, JtPre, JtPre_pinv, N_nx, N_pre;

//     qrTask<T>* task = task_list[0];
//     task->getTaskJacobian(Jt);
//     JtPre.noalias() = Jt * Nc;
//     // MITTimer T313;
//     _PseudoInverse(JtPre, JtPre_pinv);
    
//     delta_q = JtPre_pinv * (task->getPosError());
//     qdot = JtPre_pinv * (task->getDesVel());

//     DVec<T> prev_delta_q = delta_q;
//     DVec<T> prev_qdot = qdot;

//     // MITTimer T311;
//     _BuildProjectionMatrix(JtPre, N_nx);  
    
//     N_pre.noalias() = Nc * N_nx;
    
//     for (size_t i(1); i < task_list.size(); ++i) {
//         task = task_list[i];

//         task->getTaskJacobian(Jt);
//         JtPre.noalias() = Jt * N_pre;

//         _PseudoInverse(JtPre, JtPre_pinv);
//         delta_q = prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
//         qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);

//         // For the next task
//         _BuildProjectionMatrix(JtPre, N_nx);
//         N_pre *= N_nx;
//         prev_delta_q = delta_q;
//         prev_qdot = qdot;
//     }
//     jpos_cmd =  curr_config.segment(6, NumMotor) + delta_q.segment(6, NumMotor);
//     jvel_cmd = qdot.segment(6, NumMotor);

//     return true;
// }

// template <typename T>
// void KinWBC<T>::_BuildProjectionMatrix(const DMat<T>& J, DMat<T>& N) {
//     DMat<T> J_pinv;
//     _PseudoInverse(J, J_pinv);
//     N.noalias() = I_mtx - J_pinv * J;
// }

// template <typename T>
// void KinWBC<T>::_PseudoInverse(const DMat<T>& J, DMat<T>& Jinv) {
//     pseudoInverse(J, threshold_, Jinv);
// }

// template class KinWBC<float>;
