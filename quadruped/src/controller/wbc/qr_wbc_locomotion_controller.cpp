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

#include "controller/wbc/qr_wbc_locomotion_controller.hpp"

// template<typename T>
// qrWbcLocomotionCtrl<T>::qrWbcLocomotionCtrl(FloatingBaseModel<T>& model, ControlFSMData<T>* controlFSMDataIn)
//   : qrWbcController<T>(model, controlFSMDataIn)
// {
//     _body_pos_task = new BodyPosTask<T>(&(qrWbcController<T>::_model));
//     _body_ori_task = new BodyOriTask<T>(&(qrWbcController<T>::_model));

//     _foot_contact[0] = new qrSingleContact<T>(&(qrWbcController<T>::_model), Quadruped::linkID::FR);
//     _foot_contact[1] = new qrSingleContact<T>(&(qrWbcController<T>::_model), Quadruped::linkID::FL);
//     _foot_contact[2] = new qrSingleContact<T>(&(qrWbcController<T>::_model), Quadruped::linkID::HR);
//     _foot_contact[3] = new qrSingleContact<T>(&(qrWbcController<T>::_model), Quadruped::linkID::HL);

//     _foot_task[0] = new LinkPosTask<T>(&(qrWbcController<T>::_model), Quadruped::linkID::FR);
//     _foot_task[1] = new LinkPosTask<T>(&(qrWbcController<T>::_model), Quadruped::linkID::FL);
//     _foot_task[2] = new LinkPosTask<T>(&(qrWbcController<T>::_model), Quadruped::linkID::HR);
//     _foot_task[3] = new LinkPosTask<T>(&(qrWbcController<T>::_model), Quadruped::linkID::HL);
// }

// template<typename T>
// qrWbcLocomotionCtrl<T>::~qrWbcLocomotionCtrl()
// {
//     delete _body_pos_task;
//     delete _body_ori_task;

//     for (size_t i(0); i < 4; ++i) {
//         delete _foot_contact[i];
//         delete _foot_task[i];
//     }
// }

// template<typename T>
// void qrWbcLocomotionCtrl<T>::_ContactTaskUpdate(WbcCtrlData *input, ControlFSMData<T> *controlFSMData)
// {
//     qrWbcController<T>::_input_data = input;
//     // _ParameterSetup(controlFSMData->userParameters);
//     for (size_t i(0); i < 3; ++i) {
//         ((BodyPosTask<T> *)_body_pos_task)->_Kp[i] = 100.;
//         ((BodyPosTask<T> *)_body_pos_task)->_Kd[i] = 10.;

//         ((BodyOriTask<T> *)_body_ori_task)->_Kp[i] = 100.; // 100， 50
//         ((BodyOriTask<T> *)_body_ori_task)->_Kd[i] = 10.;  // 10， 5

//         for (size_t j(0); j < 4; ++j) {
//             ((LinkPosTask<T> *)_foot_task[j])->_Kp[i] = 500;
//             ((LinkPosTask<T> *)_foot_task[j])->_Kd[i] = 10.;
//         }
//     }
//     // Wash out the previous setup
//     _CleanUp();

//     _quat_des = robotics::math::rpyToQuat(input->pBody_RPY_des);

//     Vec3<T> zero_vec3;
//     zero_vec3.setZero();
//     _body_ori_task->UpdateTask(&_quat_des, input->vBody_Ori_des, zero_vec3);
//     _body_pos_task->UpdateTask(&(input->pBody_des), input->vBody_des, input->aBody_des);

//     qrWbcController<T>::_task_list.push_back(_body_ori_task);
//     qrWbcController<T>::_task_list.push_back(_body_pos_task);

//     for (size_t leg(0); leg < NumLeg; ++leg) {
//         if (input->contact_state[leg]) { // Contact
//             // std::cout << "[WBC] Stance leg" << leg << ", F = " << input->Fr_des[leg] <<std::endl;
//             _foot_contact[leg]->setRFDesired((DVec<T>)(input->Fr_des[leg]));
//             _foot_contact[leg]->UpdateqrContactSpec();
//             qrWbcController<T>::_contact_list.push_back(_foot_contact[leg]);

//         } else {// No Contact (swing)
//             // std::cout << "[WBC] Swing leg " << leg << std::endl;
//             _foot_task[leg]->UpdateTask(
//                 &(input->pFoot_des[leg]), // in world frame, relative to origin.
//                 input->vFoot_des[leg],
//                 input->aFoot_des[leg]);
//             //zero_vec3);
//             qrWbcController<T>::_task_list.push_back(_foot_task[leg]);
//         }
//     }

//     // qrWbcController<T>::_task_list.push_back(_body_ori_task);
//     // qrWbcController<T>::_task_list.push_back(_body_pos_task);
// }

// template<typename T>
// void qrWbcLocomotionCtrl<T>::_ParameterSetup(const UserParameters *param)
// {
//     /*
//   for(size_t i(0); i<3; ++i){
//     ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
//     ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->Kd_body[i];
//     ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
//     ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kd_ori[i];
//     for(size_t j(0); j<4; ++j){
//       ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = param->Kp_foot[i];
//       ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = param->Kd_foot[i];
//       //((LinkPosTask<T>*)_foot_task[j])->_Kp_kin[i] = 1.5;
//     }
//     qrWbcController<T>::_Kp_joint[i] = param->Kp_joint[i];
//     qrWbcController<T>::_Kd_joint[i] = param->Kd_joint[i];
//     //qrWbcController<T>::_Kp_joint_swing[i] = param->Kp_joint_swing[i];
//     //qrWbcController<T>::_Kd_joint_swing[i] = param->Kd_joint_swing[i];
//    }*/
// }

// template<typename T>
// void qrWbcLocomotionCtrl<T>::_CleanUp()
// {
//     qrWbcController<T>::_contact_list.clear();
//     qrWbcController<T>::_task_list.clear();
// }

// /*
// template<typename T>
// void qrWbcLocomotionCtrl<T>::_gazebo_PublishData() {
//   int iter(0);
//   for(size_t leg(0); leg<4; ++leg){
//     _Fr_result[leg].setZero();
    
//     if(_input_data->contact_state[leg]>0.){
//       for(size_t i(0); i<3; ++i){
//         _Fr_result[leg][i] = qrWbcController<T>::_wbic_data->_Fr[3*iter + i];
//       }
//       ++iter;
//     }
//     if(_input_data->contact_state[leg] > 0.){ // Contact
//       qrWbcController<T>::_wbc_data_lcm.contact_est[leg] = 1;
//     }else{
//       qrWbcController<T>::_wbc_data_lcm.contact_est[leg] = 0;
//     }
//   }
//   for(size_t i(0); i<3; ++i){
//    qrWbcController<T>::_wbc_data_lcm.foot_pos[i] = qrWbcController<T>::_model._pGC[linkID::FR][i];
//     qrWbcController<T>::_wbc_data_lcm.foot_vel[i] = qrWbcController<T>::_model._vGC[linkID::FR][i];
//     qrWbcController<T>::_wbc_data_lcm.foot_pos[i + 3] = qrWbcController<T>::_model._pGC[linkID::FL][i];
//     qrWbcController<T>::_wbc_data_lcm.foot_vel[i + 3] = qrWbcController<T>::_model._vGC[linkID::FL][i];
//     qrWbcController<T>::_wbc_data_lcm.foot_pos[i + 6] = qrWbcController<T>::_model._pGC[linkID::HR][i];
//     qrWbcController<T>::_wbc_data_lcm.foot_vel[i + 6] = qrWbcController<T>::_model._vGC[linkID::HR][i];
//     qrWbcController<T>::_wbc_data_lcm.foot_pos[i + 9] = qrWbcController<T>::_model._pGC[linkID::HL][i];
//     qrWbcController<T>::_wbc_data_lcm.foot_vel[i + 9] = qrWbcController<T>::_model._vGC[linkID::HL][i];
//     for(size_t leg(0); leg<4; ++leg){
//       qrWbcController<T>::_wbc_data_lcm.Fr_des[3*leg + i] = _input_data->Fr_des[leg][i];
//       qrWbcController<T>::_wbc_data_lcm.Fr[3*leg + i] = _Fr_result[leg][i];
//       qrWbcController<T>::_wbc_data_lcm.foot_pos_cmd[3*leg + i] = _input_data->pFoot_des[leg][i];
//       qrWbcController<T>::_wbc_data_lcm.foot_vel_cmd[3*leg + i] = _input_data->vFoot_des[leg][i];
//       qrWbcController<T>::_wbc_data_lcm.foot_acc_cmd[3*leg + i] = _input_data->aFoot_des[leg][i];
//       // TEST
//       //qrWbcController<T>::_wbc_data_lcm.foot_acc_numeric[3*leg + i] = 
//         //(_input_data->vFoot_des[leg][i] - pre_foot_vel[leg][i])/0.002;
//       //pre_foot_vel[leg][i] = _input_data->vFoot_des[leg][i];
//       wbc_commands[leg][i] = qrWbcController<T>::_des_jpos[3*leg + i];
//       vwbc_commands[leg][i] = qrWbcController<T>::_des_jvel[3*leg + i];
//       std::cout << wbc_commands[leg][i] << std::endl;
//       //qrWbcController<T>::_wbc_data_lcm.jpos[3*leg + i] = qrWbcController<T>::_state.q[3*leg + i];
//       //qrWbcController<T>::_wbc_data_lcm.jvel[3*leg + i] = qrWbcController<T>::_state.qd[3*leg + i];
//     }
//     qrWbcController<T>::_wbc_data_lcm.body_pos_cmd[i] = _input_data->pBody_des[i];
//     qrWbcController<T>::_wbc_data_lcm.body_vel_cmd[i] = _input_data->vBody_des[i];
//     qrWbcController<T>::_wbc_data_lcm.body_ori_cmd[i] = _quat_des[i];
//     Quat<T> quat = qrWbcController<T>::_state.bodyOrientation;
//     Mat3<T> Rot = robotics::math::quaternionToRotationMatrix(quat);
//     Vec3<T> global_body_vel = Rot.transpose() * qrWbcController<T>::_state.bodyVelocity.tail(3);
//     qrWbcController<T>::_wbc_data_lcm.body_pos[i] = qrWbcController<T>::_state.bodyPosition[i];
//     qrWbcController<T>::_wbc_data_lcm.body_vel[i] = global_body_vel[i];
//     qrWbcController<T>::_wbc_data_lcm.body_ori[i] = qrWbcController<T>::_state.bodyOrientation[i];
//     qrWbcController<T>::_wbc_data_lcm.body_ang_vel[i] = qrWbcController<T>::_state.bodyVelocity[i];
//   }
//   qrWbcController<T>::_wbc_data_lcm.body_ori_cmd[3] = _quat_des[3];
//   qrWbcController<T>::_wbc_data_lcm.body_ori[3] = qrWbcController<T>::_state.bodyOrientation[3];
//   qrWbcController<T>::_wbcLCM.publish("wbc_lcm_data", &(qrWbcController<T>::_wbc_data_lcm) );
// }
// */
// template class qrWbcLocomotionCtrl<float>;
