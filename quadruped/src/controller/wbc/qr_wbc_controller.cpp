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

#include "controller/wbc/qr_wbc_controller.hpp"

// template<typename T>
// qrWbcController<T>::qrWbcController(FloatingBaseModel<T>& model,
//                                ControlFSMData<T>* controlFSMDataIn) 
//     : _full_config(NumMotor + 7), _tau_ff(NumMotor), _des_jpos(NumMotor), _des_jvel(NumMotor), 
//     controlFSMData(controlFSMDataIn), _model(model)
// //_wbcLCM(getLcmUrl(255))
// {
//     _iter = 0;
//     _full_config.setZero();

//     _kin_wbc = new KinWBC<T>(dim_config); // 18

//     _wbic = new WBIC<T>(dim_config, &(_contact_list), &(_task_list));
//     _wbic_data = new WBIC_ExtraData<T>();
//     _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
//     //_wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
//     // _wbic_data->_W_floating[5] = 1;
//     // _wbic_data->_W_floating[1] = 1;
//     _wbic_data->_W_rf = DVec<T>::Constant(12, 1);

//     _Kp_joint.resize(numMotorOfOneLeg, 5.); // 100, 100, 100
//     _Kd_joint.resize(numMotorOfOneLeg, 1.5); // 1, ,2 ,2

//     //_Kp_joint_swing.resize(numMotorOfOneLeg, 10.);
//     //_Kd_joint_swing.resize(numMotorOfOneLeg, 1.5);

//     _state.q = DVec<T>::Zero(NumMotor);
//     _state.qd = DVec<T>::Zero(NumMotor);
// }

// template<typename T>
// qrWbcController<T>::~qrWbcController()
// {
//     delete _kin_wbc;
//     delete _wbic;
//     delete _wbic_data;

//     typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
//     while (iter < _task_list.end()) {
//         delete (*iter);
//         ++iter;
//     }
//     _task_list.clear();

//     typename std::vector<qrContactSpec<T> *>::iterator iter2 = _contact_list.begin();
//     while (iter2 < _contact_list.end()) {
//         delete (*iter2);
//         ++iter2;
//     }
//     _contact_list.clear();
// }

// template<typename T>
// void qrWbcController<T>::_ComputeWBC()
// {
//     // MITTimer T31;
//     _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
//                                 _des_jpos, _des_jvel, controlFSMData);
//     // printf("T31 SOLVE TIME: %.3f\n", T31.getMs());
    
//     // WBIC
//     // MITTimer T32;
//     _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
//     // printf("T32 SOLVE TIME: %.3f\n", T32.getMs());
//     // MITTimer T33;
//     _wbic->MakeTorque(_tau_ff, _wbic_data, controlFSMData);
//     // printf("T33 SOLVE TIME: %.3f\n", T33.getMs());
// }

// template<typename T>
// void qrWbcController<T>::run(void *precomputeData)
// {
//     // MITTimer T1;

//     if (_iter%2 == 0) {
//         MITTimer T1;
//         // Update Model
//         _UpdateModel(controlFSMData->quadruped); // 0.05ms
//         // printf("####UpdateModel SOLVE TIME: %.3f\n", T1.getMs());
        
//         // MITTimer T2;
//         // Task & Contact Update 0.003ms
//         _ContactTaskUpdate(static_cast<WbcCtrlData*>(precomputeData), controlFSMData);
//         // printf("####TaskUpdate SOLVE TIME: %.3f\n", T2.getMs());
//         // MITTimer T3;
//         // WBC Computation
//         _ComputeWBC(); // 0.165ms
//         // printf("####WBC SOLVE TIME: %.3f\n", T1.getMs());
//         controlFSMData->quadruped->stateDataFlow.visualizer.sa[2].Update(T1.getMs());

//     }
//     // Update Leg Command
//     _UpdateLegCMD(controlFSMData);
//     // printf("wbc[%llu] SOLVE TIME: %.3f\n", _iter, T1.getMs());
//     ++_iter;
// }

// template<typename T>
// void qrWbcController<T>::_UpdateLegCMD(ControlFSMData<T> *data)
// {
//     std::vector<Quadruped::MotorCommand>& cmd = data->legCmd;
//     // std::cout<< "****************************" << std::endl;
//     // for (int i(0); i<12; ++i) {
//     //     std::cout << cmd[i] << std::endl;
//     // }
//     for (size_t leg(0); leg < NumLeg; ++leg) {
//         for (size_t j(0); j < numMotorOfOneLeg; ++j) {
//             int motorId = numMotorOfOneLeg * leg + j;
//             // cmd[motorId].SetZero();
//             if (_input_data->contact_state[leg]) { // stance
//                 cmd[motorId].tua = _tau_ff[motorId];
//                 cmd[motorId].p = _des_jpos[motorId];
//                 cmd[motorId].d = _des_jvel[motorId];
//                 cmd[motorId].Kp = 10; // 5
                
//                 // cmd[motorId].Kd = 2;
                
//                 // if (motorId%3 == 2) {
                   
//                     // cmd[motorId].Kp = 10;
//                     // cmd[motorId].Kd = 3;
//                     // if (controlFSMData->gaitGenerator->legState[leg] == Quadruped::LegState::EARLY_CONTACT) {
//                     //     cmd[motorId].Kp = 10;
//                     //     cmd[motorId].Kd = 5;
//                         // float a = controlFSMData->gaitGenerator->contactStartPhase[leg];
//                         // if ( a > 0.7 && a<0.9 && motorId%3 == 1) {
//                         //     cmd[motorId].tua /= 2.0f;
//                         // }
//                     // }
//                 // }
//             } else { // swing
//                 // cmd[motorId].p = _des_jpos[motorId];
//                 // cmd[motorId].d = _des_jvel[motorId];
//                 // std::cout << " leg = " << leg << "joint = " << j << "angle = " << cmd[motorId].p <<  std::endl; // cmd[leg].tauFeedForward[j] <<
//                 // cmd[motorId].Kp =  _Kp_joint[j];
//                 // cmd[motorId].Kd = _Kd_joint[j];
//             }
//         }
//     }
//     // std::cout<< "***" << std::endl;
//     // for (int i(0); i<12; ++i) {
//     //     std::cout << cmd[i] << std::endl;
//     // }
//     // std::cout<< "****************************" << std::endl;
//     // Knee joint non flip barrier
//     auto jointAngles = data->quadruped->GetMotorAngles();
//     for(size_t leg(0); leg < NumLeg; ++leg){
//         size_t motorId = leg * 3 + 2;
//         if(cmd[motorId].p > -0.3){
//             cmd[motorId].p = -0.3;
//         }
//         if(jointAngles[motorId] > -0.3){
//             float knee_pos = jointAngles[motorId];
//             cmd[motorId].tua = -1./(knee_pos * knee_pos + 0.02);
//         }
//     }
// }

// template<typename T>
// void qrWbcController<T>::_UpdateModel(qrRobot *robot)
// {
//     _state.bodyOrientation = robot->GetBaseOrientation();
//     _state.bodyPosition = robot->GetBasePosition();
//     auto vBody = robot->GetEstimatedVelocityInBaseFrame();
//     auto q = robot->GetMotorAngles();
//     auto dq = robot->GetMotorVelocities();
//     auto omegaBody = robot->GetBaseRollPitchYawRate();
//     for (size_t i(0); i < 3; ++i) {
//         _state.bodyVelocity[i] = omegaBody[i]; // in body frame
//         _state.bodyVelocity[i + 3] = vBody[i];

//         for (size_t leg(0); leg < NumLeg; ++leg) {
//             size_t motorId = 3 * leg + i;
//             _state.q[motorId] = q[motorId];
//             _state.qd[motorId] = dq[motorId];
//             _full_config[motorId + 6] = _state.q[motorId];
//         }
//     }
//     _model.setState(_state);
//     _model.contactJacobians();
//     _model.massMatrix();
//     _model.generalizedGravityForce();
//     _model.generalizedCoriolisForce();

//     _A = _model.getMassMatrix();
//     _grav = _model.getGravityForce();
//     _coriolis = _model.getCoriolisForce();
//     _Ainv = _A.inverse();
// }

// template<typename T>
// void qrWbcController<T>::setFloatingBaseWeight(const T &weight)
// {
//     _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
// }

// template class qrWbcController<float>;