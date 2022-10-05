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

#ifndef QR_WHOLE_BODY_CONTROL_HPP
#define QR_WHOLE_BODY_CONTROL_HPP

#include "controller/wbc/qr_contact_spec.hpp"
#include "controller/wbc/qr_task.hpp"

// Assume first 6 (or 3 in 2D case) joints are for the representation of
// a floating base.

// #define WB WBC<T>

// template <typename T>
// class WBC {
// public:
//     WBC(size_t num_qdot) : num_act_joint_(num_qdot - 6), num_qdot_(num_qdot) {
//         Sa_ = DMat<T>::Zero(num_act_joint_, num_qdot_);
//         Sv_ = DMat<T>::Zero(6, num_qdot_);

//         Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity();
//         Sv_.block(0, 0, 6, 6).setIdentity();
//     }
//     virtual ~WBC() {}

//     /**
//      * @brief UpdateSetting interface method
//      */
//     virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
//                                 const DVec<T>& cori, const DVec<T>& grav,
//                                 void* extra_setting = NULL) = 0;

//     /**
//      * @brief MakeTorque interface method
//      */
//     virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL, 
//         ControlFSMData<T>* controlFSMData=NULL) {printf("not implement!\n");}

// protected:

//     /**
//      * @brief full rank fat matrix only
//      */
//     void _WeightedInverse(const DMat<T>& J, const DMat<T>& Winv, DMat<T>& Jinv,
//                             double threshold = 0.0001) {
//         DMat<T> temp(Winv * J.transpose());
//         DMat<T> lambda(J * temp);
//         DMat<T> lambda_inv;
//         pseudoInverse(lambda, threshold, lambda_inv);
//         Jinv.noalias() = temp * lambda_inv;
//     }

//     size_t num_act_joint_; // 12
//     size_t num_qdot_; // 18

//     DMat<T> Sa_;  // Actuated joint
//     DMat<T> Sv_;  // Virtual joint

//     DMat<T> A_;
//     DMat<T> Ainv_;
//     DVec<T> cori_;
//     DVec<T> grav_;

//     bool b_updatesetting_;
//     bool b_internal_constraint_;
// };

#endif // QR_WHOLE_BODY_CONTROL_HPP