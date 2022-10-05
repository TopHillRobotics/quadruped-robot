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

#ifndef WBC_CONTROLLER_HPP
#define WBC_CONTROLLER_HPP

#include "robots/qr_robot.h"
#include "dynamics/qr_floating_base_model.hpp"
#include "controller/wbc/qr_WBIC.hpp"
#include "controller/wbc/qr_kinematics_WBC.hpp"
// #include "controller/desired_state_command.hpp"

// template<typename T>
// class qrWbcController
// {
// public:
//     qrWbcController(FloatingBaseModel<T>& model, ControlFSMData<T>* controlFSMData);
    
//     virtual ~qrWbcController();

//     void run(void * input);
    
//     void setFloatingBaseWeight(const T & weight);
//     WbcCtrlData* _input_data;
// protected:
//     virtual void _ContactTaskUpdate(WbcCtrlData *input, ControlFSMData<T>* data) {};
    
//     virtual void _LCM_PublishData() {}
    
//     void _UpdateModel(qrRobot* robot);
    
//     void _UpdateLegCMD(ControlFSMData<T>* data);
    
//     void _ComputeWBC();

//     const size_t dim_config = 18;

//     ControlFSMData<T>* controlFSMData;

//     KinWBC<T>* _kin_wbc;
//     WBIC<T>* _wbic;
//     WBIC_ExtraData<T>* _wbic_data;

//     FloatingBaseModel<T>& _model;
//     std::vector<qrContactSpec<T> * > _contact_list;
//     std::vector<Task<T> * > _task_list;

//     DMat<T> _A;
//     DMat<T> _Ainv;
//     DVec<T> _grav;
//     DVec<T> _coriolis;

//     FBModelState<T> _state;

//     DVec<T> _full_config;
//     DVec<T> _tau_ff;
//     DVec<T> _des_jpos;
//     DVec<T> _des_jvel;

//     std::vector<T> _Kp_joint, _Kd_joint;

//     unsigned long long _iter;

// };

#endif // WBC_CONTROLLER_H