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

// (X, Y, Z)
template <typename T>
LinkPosTask<T>::LinkPosTask(const FloatingBaseModel<T>* robot, int link_idx,
                            bool virtual_depend)
    : qrTask<T>(3),
      robot_sys_(robot),
      link_idx_(link_idx),
      virtual_depend_(virtual_depend) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, this->dim_config);
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp = DVec<T>::Constant(TK::dim_task_, 100.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 5.);
  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
}

template <typename T>
LinkPosTask<T>::~LinkPosTask() {}

template <typename T>
bool LinkPosTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  Vec3<T>* pos_cmd = (Vec3<T>*)pos_des; // in world frame
  Vec3<T> link_pos;

  link_pos = robot_sys_->_pGC[link_idx_];
  Vec3<T> v_error;
  // X, Y, Z
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ((*pos_cmd)[i] - link_pos[i]);
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
  }

  // Op acceleration command
  for (size_t i(0); i < TK::dim_task_; ++i) {
    v_error[i] = TK::vel_des_[i] - robot_sys_->_vGC[link_idx_][i];
    TK::op_cmd_[i] = 
      /* 100 */ _Kp[i] * TK::pos_err_[i] + 
      /* 5 */   _Kd[i] * v_error[i] +
                TK::acc_des_[i];
  }

  // printf("[Link Pos Task]\n");
  // pretty_print(acc_des, std::cout, "acc_des");
  // pretty_print(TK::pos_err_, std::cout, "pos_err_");
  // pretty_print(v_error, std::cout, "v_error");
  // pretty_print(*pos_cmd, std::cout, "pos cmd");
  // pretty_print(robot_sys_->_vGC[link_idx_], std::cout, "velocity");
  // pretty_print(TK::op_cmd_, std::cout, "op cmd");
  // TK::op_cmd_.setZero();
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool LinkPosTask<T>::_UpdateTaskJacobian() {
  TK::Jt_ = robot_sys_->_Jc[link_idx_];
  if (!virtual_depend_) {
    TK::Jt_.block(0, 0, 3, 6) = DMat<T>::Zero(3, 6);
  }
  return true;
}

template <typename T>
bool LinkPosTask<T>::_UpdateTaskJDotQdot() {
  TK::JtDotQdot_ = robot_sys_->_Jcdqd[link_idx_];
  return true;
}

// template class LinkPosTask<double>;
template class LinkPosTask<float>;
