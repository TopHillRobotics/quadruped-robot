// (Rx, Ry, Rz)
template<typename T>
BodyOriTask<T>::BodyOriTask(const FloatingBaseModel<T> *robot)
    : Task<T>(3), _robot_sys(robot)
{
    TK::Jt_ = DMat<T>::Zero(TK::dim_task_, this->dim_config);
    TK::Jt_.block(0, 0, 3, 3).setIdentity();
    TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);

    _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
    _Kp = DVec<T>::Constant(TK::dim_task_, 50.);
    _Kd = DVec<T>::Constant(TK::dim_task_, 1.0);
}

template<typename T>
BodyOriTask<T>::~BodyOriTask()
{}

template<typename T>
bool BodyOriTask<T>::_UpdateCommand(const void *pos_des, const DVec<T> &vel_des,
                                    const DVec<T> &acc_des)
{
    Quat<T> *ori_cmd = (Quat<T> *)pos_des;
    Quat<T> link_ori = (_robot_sys->_state.bodyOrientation);

    Quat<T> link_ori_inv;
    link_ori_inv[0] = link_ori[0];
    link_ori_inv[1] = -link_ori[1];
    link_ori_inv[2] = -link_ori[2];
    link_ori_inv[3] = -link_ori[3];
    // link_ori_inv /= link_ori.norm();

    // Explicit because operational space is in global frame
    Quat<T> ori_err = robotics::math::quatProduct(*ori_cmd, link_ori_inv);
    if (ori_err[0] < 0.) {
        ori_err *= (-1.);
    }
    Vec3<T> ori_err_so3;
    robotics::math::quaternionToso3(ori_err, ori_err_so3);
    SVec<T> curr_vel = _robot_sys->_state.bodyVelocity;// w,v  in body frame
    // Configuration space: Local
    // Operational Space: Global
    Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(link_ori);
    Vec3<T> vel_err = Rot.transpose() * (TK::vel_des_ - curr_vel.head(3));//body frame-> world frame

    // Rx, Ry, Rz
    for (int i(0); i < 3; ++i) {
        TK::pos_err_[i] = _Kp_kin[i] * ori_err_so3[i];
        TK::vel_des_[i] = vel_des[i];
        TK::acc_des_[i] = acc_des[i];

        TK::op_cmd_[i] = _Kp[i] * ori_err_so3[i] +// 50
            _Kd[i] * vel_err[i] +                 // 1
            TK::acc_des_[i];
        TK::op_cmd_[i] = std::min(std::max(TK::op_cmd_[i], (T)-10), (T)10);
    }
    return true;
}

template<typename T>
bool BodyOriTask<T>::_UpdateTaskJacobian()
{
    Quat<T> quat = _robot_sys->_state.bodyOrientation;
    Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat);
    TK::Jt_.block(0, 0, 3, 3) = Rot.transpose();
    return true;
}

template<typename T>
bool BodyOriTask<T>::_UpdateTaskJDotQdot()
{
    return true;
}

// template class BodyOriTask<double>;
template class BodyOriTask<float>;
