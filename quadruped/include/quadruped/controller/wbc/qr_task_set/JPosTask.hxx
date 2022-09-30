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

#include "JPosTask.hpp"
#include <Configuration.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include <Utilities/Utilities_print.h>

template <typename T>
JPosTask<T>::JPosTask(const FloatingBaseModel<T>* robot)
    : qrTask<T>(cheetah::num_act_joint), robot_sys_(robot) {
  TK::Jt_ = DMat<T>::Zero(cheetah::num_act_joint, cheetah::dim_config);
  (TK::Jt_.block(0, 6, cheetah::num_act_joint, cheetah::num_act_joint))
      .setIdentity();
  TK::JtDotQdot_ = DVec<T>::Zero(cheetah::num_act_joint);

  _Kp = DVec<T>::Constant(cheetah::num_act_joint, 50.);
  _Kd = DVec<T>::Constant(cheetah::num_act_joint, 5.);
}

template <typename T>
JPosTask<T>::~JPosTask() {}

template <typename T>
bool JPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                 const DVec<T>& acc_des) {
  DVec<T>* pos_cmd = (DVec<T>*)pos_des;

  for (size_t i(0); i < cheetah::num_act_joint; ++i) {
    TK::pos_err_[i] = (*pos_cmd)[i] - robot_sys_->_state.q[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = _Kp[i] * TK::pos_err_[i] +
                     _Kd[i] * (vel_des[i] - robot_sys_->_state.qd[i]) +
                     acc_des[i];
  }
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(op_cmd_, std::cout, "op cmd");
  // pretty_print(*pos_cmd, std::cout, "pos cmd");

  return true;
}

template <typename T>
bool JPosTask<T>::_UpdateTaskJacobian() {
  return true;
}

template <typename T>
bool JPosTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

template class JPosTask<double>;
template class JPosTask<float>;
