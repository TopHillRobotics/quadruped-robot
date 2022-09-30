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

#ifndef QR_KINEMATICS_WHOLE_BODY_CONTROL_HPP
#define QR_KINEMATICS_WHOLE_BODY_CONTROL_HPP

// #include "controller/wbc/qr_contact_spec.hpp"
// #include "controller/wbc/qr_task.hpp"

// template <typename T>
// class KinWBC {
// public:
//     KinWBC(size_t num_qdot);
//     ~KinWBC() {}

//     /**
//      * @brief 
//      */
//     bool FindConfiguration(const DVec<T>& curr_config,
//                             const std::vector<Task<T>*>& task_list,
//                             const std::vector<qrContactSpec<T>*>& contact_list,
//                             DVec<T>& jpos_cmd, DVec<T>& jvel_cmd, ControlFSMData<T>* controlFSMData);

//     DMat<T> Ainv_;

// private:

//     /**
//      * @brief 
//      */
//     void _PseudoInverse(const DMat<T>& J, DMat<T>& Jinv);
    
//     /**
//      * @brief 
//      */
//     void _BuildProjectionMatrix(const DMat<T>& J, DMat<T>& N);

//     double threshold_;
//     size_t num_qdot_;
//     size_t num_act_joint_;
//     DMat<T> I_mtx;
// };

#endif // QR_KINEMATICS_WHOLE_BODY_CONTROL_HPP