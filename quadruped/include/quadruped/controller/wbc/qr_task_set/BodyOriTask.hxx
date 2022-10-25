// The MIT License

template <typename T>
BodyOriTask<T>::BodyOriTask(const FloatingBaseModel<T>* robot)
    : qrTask<T>(3), _robot_sys(robot) {
  TK::Jt_ = DMat<T>::Zero(TK::dim_task_, this->dim_config);
  TK::Jt_.block(0, 0, 3, 3).setIdentity();
  TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

  _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
  _Kp = DVec<T>::Constant(TK::dim_task_, 50.);
  _Kd = DVec<T>::Constant(TK::dim_task_, 1.0);
}

template <typename T>
BodyOriTask<T>::~BodyOriTask() {}

template <typename T>
bool BodyOriTask<T>::_UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                    const DVec<T>& acc_des) {
  Quat<T>* ori_cmd = (Quat<T>*)pos_des;
  Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);

  Quat<T> link_ori_inv;
  link_ori_inv[0] = link_ori[0];
  link_ori_inv[1] = -link_ori[1];
  link_ori_inv[2] = -link_ori[2];
  link_ori_inv[3] = -link_ori[3];
  // link_ori_inv /= link_ori.norm();

  // Explicit because operational space is in global frame
  Quat<T> ori_err = math::quatProduct(*ori_cmd, link_ori_inv);
  if (ori_err[0] < 0.) {
    ori_err *= (-1.);
  }
  Vec3<T> ori_err_so3;
  ori_err_so3 = math::quatToso3(ori_err);
  SVec<T> curr_vel = _robot_sys->_state.bodyVelocity; // w,v  in body frame
  // std::cout << "curr_vel=" << curr_vel << std::endl;
  // Configuration space: Local
  // Operational Space: Global
  Mat3<T> Rot = math::quaternionToRotationMatrix(link_ori);
  // for (size_t i(0); i < 3; ++i) {
  //     TK::vel_des_[i] = vel_des[i];
  //     TK::acc_des_[i] = acc_des[i];
  
  // }
  Vec3<T> vel_err = Rot.transpose()*(TK::vel_des_ - curr_vel.head(3)); //body frame-> world frame

  // Rx, Ry, Rz
  for (int i(0); i < 3; ++i) {
    TK::pos_err_[i] = _Kp_kin[i] * ori_err_so3[i];
    TK::vel_des_[i] = vel_des[i];
    TK::acc_des_[i] = acc_des[i];
  
  
    TK::op_cmd_[i] = _Kp[i] * ori_err_so3[i] + // 50
                     _Kd[i] * vel_err[i] + // 1
                     TK::acc_des_[i];
    TK::op_cmd_[i] = std::min(std::max(TK::op_cmd_[i], (T)-10), (T)10);
  }
  //  printf("[Body Ori Task]\n");
   //pretty_print(TK::pos_err_, std::cout, "pos_err_");
  //  pretty_print(*ori_cmd, std::cout, "des_ori");
  //  pretty_print(TK::vel_des_, std::cout, "des_w");
  // pretty_print(acc_des, std::cout, "des_acc");
  //  pretty_print(link_ori, std::cout, "curr_ori");
  // pretty_print(ori_err, std::cout, "ori_err");
  //  pretty_print(ori_err_so3, std::cout, "ori_err_so3");
  // pretty_print(vel_err, std::cout, "vel_err");
  // pretty_print(link_ori_inv, std::cout, "ori_inv");
  
  // pretty_print(TK::op_cmd_, std::cout, "op_cmd");
  
  // pretty_print(TK::Jt_, std::cout, "Jt");

  return true;
}

template <typename T>
bool BodyOriTask<T>::_UpdateTaskJacobian() {
  Quat<T> quat = _robot_sys->_state.bodyOrientation;
  Mat3<T> Rot = math::quaternionToRotationMatrix(quat);
  TK::Jt_.block(0, 0, 3, 3) = Rot.transpose();
  //pretty_print(Rot, std::cout, "Rot mat");
  return true;
}

template <typename T>
bool BodyOriTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

template class BodyOriTask<double>;
template class BodyOriTask<float>;
