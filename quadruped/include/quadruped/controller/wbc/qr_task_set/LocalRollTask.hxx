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

// #include "LocalRollTask.hpp"
// (Rx, Ry, Rz)

template <typename T>
LocalRollTask<T>::LocalRollTask(const FloatingBaseModel<T>* robot)
    : qrTask<T>(1), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, dim_config);
  TK::Jt_(0, 0) = 1.;
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 350.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 13.);
}

template <typename T>
LocalRollTask<T>::~LocalRollTask() {}

template <typename T>
bool LocalRollTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                      const DVec<T>& acc_des) {
  T* roll_cmd = (T*)pos_des;
  Vec3<T> rpy = math::quatToRPY(_robot_sys->_state.bodyOrientation);

  // Rx
  TK::pos_err_[0] = _Kp_kin[0] * (*roll_cmd - rpy[0]);
  TK::vel_des_[0] = vel_des[0];
  TK::acc_des_[0] = acc_des[0];

  TK::op_cmd_[0] =
      _Kp[0] * (*roll_cmd - rpy[0]) +
      _Kd[0] * (TK::vel_des_[0] - _robot_sys->_state.bodyVelocity[0]) +
      TK::acc_des_[0];

  // printf("[Body Ori Roll Task]\n");
  // pretty_print(TK::pos_err_, std::cout, "pos_err_");
  // pretty_print(rpy, std::cout, "rpy");
  // pretty_print(_robot_sys->_state.bodyVelocity, std::cout, "body velocity");
  // pretty_print(TK::op_cmd_, std::cout, "op cmd");
  // pretty_print(acc_des, std::cout, "acc_des");
  // printf("\n");
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool LocalRollTask<T>::_UpdateTaskJacobian() {
  return true;
}

template <typename T>
bool LocalRollTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

// template class LocalRollTask<double>;
template class LocalRollTask<float>;
