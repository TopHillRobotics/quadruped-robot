// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

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

#include "controllers/wbc/task_set/qr_task_body_position.hpp"


template <typename T>
qrTaskBodyPosition<T>::qrTaskBodyPosition(const FloatingBaseModel<T>* fb_model):
    qrTask<T>(3), fbModel(fb_model)
{
    TK::Jt = DMat<T>::Zero(TK::dimTask, this->dimConfig);
    TK::Jt.block(0, 3, 3, 3).setIdentity();
    TK::JtDotQdot = DVec<T>::Zero(TK::dimTask);

    errScale = DVec<T>::Constant(TK::dimTask, 1.);
    Kp = DVec<T>::Constant(TK::dimTask, 30.);
    Kd = DVec<T>::Constant(TK::dimTask, 1.0);
}


template <typename T>
bool qrTaskBodyPosition<T>::UpdateCommand(const void* des_pos, const DVec<T>& des_vel, const DVec<T>& des_acc) {
    Vec3<T>* pos_cmd = (Vec3<T>*)des_pos;
    Vec3<T> link_pos = fbModel->_state.bodyPosition; /* Body position in world frame. */

    Quat<T> quat = fbModel->_state.bodyOrientation;
    Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat); /* Rotation matrix from base frame to world frame. */

    SVec<T> curr_vel = fbModel->_state.bodyVelocity;
    curr_vel.tail(3) = Rot.transpose() * curr_vel.tail(3); /* Current velocity in world frame. */


    for (int i = 0; i < 3; ++i) {
        TK::posErr[i] = errScale[i] * ((*pos_cmd)[i] - link_pos[i]); // kp_kin=1
        TK::desiredVel[i] = des_vel[i];
        TK::desiredAcc[i] = des_acc[i];

        /* PD control to get acceleration command. */
        TK::xddotCmd[i] =
            Kp[i] * ((*pos_cmd)[i] - link_pos[i]) + Kd[i] * (TK::desiredVel[i] - curr_vel[i + 3]) + TK::desiredAcc[i];
        TK::xddotCmd[i] = std::min(std::max(TK::xddotCmd[i], (T)-10), (T)10);
    }
    return true;
}


template <typename T>
bool qrTaskBodyPosition<T>::UpdateTaskJacobian() {
    Quat<T> quat = fbModel->_state.bodyOrientation;
    Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat);
    TK::Jt.block(0, 3, 3, 3) = Rot.transpose();
    return true;
}


template <typename T>
bool qrTaskBodyPosition<T>::UpdateTaskJDotQdot() {
  return true;
}


template class qrTaskBodyPosition<float>;
