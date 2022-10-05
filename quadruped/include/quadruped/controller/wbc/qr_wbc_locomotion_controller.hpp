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

#ifndef QR_WBC_LOCOMOTION_CONTROLLER_HPP
#define QR_WBC_LOCOMOTION_CONTROLLER_HPP

#include "controller/wbc/qr_wbc_controller.hpp"
#include "controller/wbc/qr_single_contact.hpp"
#include "controller/wbc/qr_task_set/BodyOriTask.hpp"
#include "controller/wbc/qr_task_set/BodyPosTask.hpp"
#include "controller/wbc/qr_task_set/LinkPosTask.hpp"

// template<typename T>
// class qrWbcLocomotionCtrl: public qrWbcController<T>{
//   public:
//     qrWbcLocomotionCtrl(FloatingBaseModel<T>& model, ControlFSMData<T>* controlFSMData);
//     virtual ~qrWbcLocomotionCtrl();

//   protected:
//     virtual void _ContactTaskUpdate(WbcCtrlData *input, ControlFSMData<T>* controlFSMData);
//     void _ParameterSetup(const UserParameters* param);
//     void _CleanUp();
    
//     //virtual void _LCM_PublishData();

//     qrTask<T>* _body_pos_task;
//     qrTask<T>* _body_ori_task;

//     qrTask<T>* _foot_task[4];
//     qrContactSpec<T>* _foot_contact[4];

//     Vec3<T> pre_foot_vel[4];

//     Vec3<T> _Fr_result[4];
//     Quat<T> _quat_des;
// };

#endif //QR_WBC_LOCOMOTION_CONTROLLER_HPP