// (X, Y, Z)
template <typename T>
BodyPosTask<T>::BodyPosTask(const FloatingBaseModel<T>* robot)
    : Task<T>(3), _robot_sys(robot) {
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
  Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat);

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
  return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJacobian() {
  Quat<T> quat = _robot_sys->_state.bodyOrientation;
  Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat);
  TK::Jt_.block(0, 3, 3, 3) = Rot.transpose();
  return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJDotQdot() {
  return true;
}

// template class BodyPosTask<double>;
template class BodyPosTask<float>;
