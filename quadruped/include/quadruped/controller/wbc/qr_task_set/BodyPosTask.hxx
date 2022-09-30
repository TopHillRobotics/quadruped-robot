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

template <typename T>
BodyPosTask<T>::BodyPosTask(const FloatingBaseModel<T>* robot)
    : qrTask<T>(3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, this->dim_config);
  TK::Jt_.block(0, 3, 3, 3).setIdentity();
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 30.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 1.0);
}

template <typename T>
BodyPosTask<T>::~BodyPosTask() {}

template <typename T>
bool BodyPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;
  Vec3<T> link_pos = _robot_sys->_state.bodyPosition;

  Quat<T> quat = _robot_sys->_state.bodyOrientation;
  Mat3<T> Rot = math::quaternionToRotationMatrix(quat);

  SVec<T> curr_vel = _robot_sys->_state.bodyVelocity;
  curr_vel.tail(3) = Rot.transpose() * curr_vel.tail(3); // world frame

  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ((*pos_cmd)[i] - link_pos[i]); // kp_kin=1
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];

    TK::op_cmd_[i] = 
          /*100 */   _Kp[i] * ((*pos_cmd)[i] - link_pos[i]) +
          /* 10 */    _Kd[i] * (TK::vel_des_[i] - curr_vel[i + 3]) +
                     TK::acc_des_[i];
    TK::op_cmd_[i] = std::min(std::max(TK::op_cmd_[i], (T)-10), (T)10);
  }
  // Quat<T> quat = _robot_sys->_state.bodyOrientation;
  // Mat3<T> Rot = math::quaternionToRotationMatrix(quat);
  // TK::pos_err_ = Rot * TK::pos_err_;
  // TK::vel_des_ = Rot * TK::vel_des_;
  // TK::acc_des_ = Rot * TK::acc_des_;

  // printf("[Body Pos Task]\n");
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(TK::pos_err_, std::cout, "pos_err_");
  // pretty_print(curr_vel, std::cout, "curr_vel");
  // pretty_print(*pos_cmd, std::cout, "pos cmd");
  // pretty_print(TK::op_cmd_, std::cout, "op cmd");
  // pretty_print(TK::vel_des_, std::cout, "vel des");
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJacobian() {
  Quat<T> quat = _robot_sys->_state.bodyOrientation;
  Mat3<T> Rot = math::quaternionToRotationMatrix(quat);
  TK::Jt_.block(0, 3, 3, 3) = Rot.transpose();
  // TK::Jt_.block(0,3, 3,3) = Rot;
  // pretty_print(TK::Jt_, std::cout, "Jt");
  // TK::Jt_.block(0,3, 3,3) = Rot*TK::Jt_.block(0,3,3,3);
  return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

template class BodyPosTask<double>;
// template class BodyPosTask<float>;
