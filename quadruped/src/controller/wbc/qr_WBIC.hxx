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

#include "controller/wbc/qr_WBIC.hpp"

template <typename T>
WBIC<T>::WBIC(size_t num_qdot, const std::vector<qrContactSpec<T>*>* contact_list,
    const std::vector<Task<T>*>* task_list)
  : WBC<T>(num_qdot), _dim_floating(6) 
{
    _contact_list = contact_list; // stance leg 
    _task_list = task_list; // base motion & swing leg

    _eye = DMat<T>::Identity(WB::num_qdot_, WB::num_qdot_); // (18, 18)
    _eye_floating = DMat<T>::Identity(_dim_floating, _dim_floating); // (6, 6)
}

template <typename T>
// void WBIC<T>::MakeTorque(DVec<T>& cmd, void* extra_input, ControlFSMData<T>* controlFSMData)
void WBIC<T>::MakeTorque(DVec<T>& cmd, void* extra_input)
{
    if (!WB::b_updatesetting_) {
        printf("[Wanning] WBIC setting is not done\n");
    }
    if (extra_input) _data = static_cast<WBIC_ExtraData<T>*>(extra_input);

    // resize G, g0, CE, ce0, CI, ci0
    _SetOptimizationSize();
    _SetCost();

    DVec<T> qddot_pre;
    DMat<T> JcBar;
    DMat<T> Npre;
    // MITTimer T31;
    if (_dim_rf > 0) {
        // Contact Setting
        _ContactBuilding();
        // Set inequality constraints
        _SetInEqualityConstraint();
        
        WB::_WeightedInverse(_Jc, WB::Ainv_, JcBar);
        qddot_pre = JcBar * (-_JcDotQdot);
        Npre.noalias() = _eye - JcBar * _Jc;
    } else {
        qddot_pre = DVec<T>::Zero(WB::num_qdot_);
        Npre = _eye;
    }

    // Task
    qrTask<T>* task;
    DMat<T> Jt, JtBar, JtPre;
    DVec<T> JtDotQdot, xddot;
    // MITTimer T32;
    for (size_t i(0); i < _task_list->size(); ++i) {
        task = (*_task_list)[i];

        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        task->getCommand(xddot);
        
        JtPre.noalias() = Jt * Npre;
        WB::_WeightedInverse(JtPre, WB::Ainv_, JtBar);

        qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
        Npre = Npre * (_eye - JtBar * JtPre);
    }

    // Set equality constraints
    _SetEqualityConstraint(qddot_pre);

    // Optimization
    T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
    (void)f;

    for (size_t i(0); i < _dim_floating; ++i) qddot_pre[i] += z[i];
    _GetSolution(qddot_pre, cmd);

    _data->_opt_result = DVec<T>(_dim_opt);
    for (size_t i(0); i < _dim_opt; ++i) {
        _data->_opt_result[i] = z[i];
    }
}

template <typename T>
void WBIC<T>::_SetEqualityConstraint(const DVec<T>& qddot) 
{
    if (_dim_rf > 0) {
        _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) = WB::A_.block(0, 0, _dim_floating, _dim_floating);
        _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) = -WB::Sv_ * _Jc.transpose();
        _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_ - _Jc.transpose() * _Fr_des);
    } else {
        _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) = WB::A_.block(0, 0, _dim_floating, _dim_floating);
        _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_);
    }

    for (size_t i(0); i < _dim_eq_cstr; ++i) {
        for (size_t j(0); j < _dim_opt; ++j) {
        CE[j][i] = _dyn_CE(i, j);
        }
        ce0[i] = -_dyn_ce0[i];
    }
}

template <typename T>
void WBIC<T>::_SetInEqualityConstraint() 
{
    _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;
    _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;

    for (size_t i(0); i < _dim_Uf; ++i) {
        for (size_t j(0); j < _dim_opt; ++j) {
        CI[j][i] = _dyn_CI(i, j);
        }
        ci0[i] = -_dyn_ci0[i];
    }
}

template <typename T>
void WBIC<T>::_ContactBuilding() 
{
    DMat<T> Uf;
    DVec<T> Uf_ieq_vec;
    // Initial
    DMat<T> Jc;
    DVec<T> JcDotQdot;
    size_t dim_accumul_rf, dim_accumul_uf;
    (*_contact_list)[0]->getContactJacobian(Jc);
    (*_contact_list)[0]->getJcDotQdot(JcDotQdot);
    (*_contact_list)[0]->getRFConstraintMtx(Uf);
    (*_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);

    dim_accumul_rf = (*_contact_list)[0]->getDim(); // 3
    dim_accumul_uf = (*_contact_list)[0]->getDimRFConstraint(); // 6
    
    _Jc.block(0, 0, dim_accumul_rf, WB::num_qdot_) = Jc; // (3, 18)
    _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
    _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf; // (6, 3)
    _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;
    _Fr_des.head(dim_accumul_rf) = (*_contact_list)[0]->getRFDesired();

    size_t dim_new_rf, dim_new_uf;
    for (size_t i(1); i < _contact_list->size(); ++i) {
        (*_contact_list)[i]->getContactJacobian(Jc);
        (*_contact_list)[i]->getJcDotQdot(JcDotQdot);

        dim_new_rf = (*_contact_list)[i]->getDim();
        dim_new_uf = (*_contact_list)[i]->getDimRFConstraint();
        
        // Jc append
        _Jc.block(dim_accumul_rf, 0, dim_new_rf, WB::num_qdot_) = Jc;

        // JcDotQdot append
        _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

        // Uf
        (*_contact_list)[i]->getRFConstraintMtx(Uf);
        _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

        // Uf inequality vector
        (*_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec);
        _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

        // Fr desired
        _Fr_des.segment(dim_accumul_rf, dim_new_rf) = (*_contact_list)[i]->getRFDesired();
        
        dim_accumul_rf += dim_new_rf;
        dim_accumul_uf += dim_new_uf;
    }
}

template <typename T>
void WBIC<T>::_GetSolution(const DVec<T>& qddot, DVec<T>& cmd) 
{
    DVec<T> tot_tau;
    if (_dim_rf > 0) {
        _data->_Fr = DVec<T>(_dim_rf);
        // get Reaction forces
        for (size_t i(0); i < _dim_rf; ++i) {
        _data->_Fr[i] = z[i + _dim_floating] + _Fr_des[i];
        }

        tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_ - _Jc.transpose() * _data->_Fr;
    } else {
        tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_;
    }
    _data->_qddot = qddot;
    cmd = tot_tau.tail(WB::num_act_joint_);
}

template <typename T>
void WBIC<T>::_SetCost() 
{
    // Set Cost
    size_t idx_offset(0);
    for (size_t i(0); i < _dim_floating; ++i) {
        G[i + idx_offset][i + idx_offset] = _data->_W_floating[i]; // [0.1,0.1,...0.1] (6,6)
    }
    idx_offset += _dim_floating;
    for (size_t i(0); i < _dim_rf; ++i) {
        G[i + idx_offset][i + idx_offset] = /*0.00001 **/ _data->_W_rf[i]; // [1, 1, 1, ...,1] (3*contactNum,3*contactNum) 
    }
}

template <typename T>
void WBIC<T>::UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
    const DVec<T>& cori, const DVec<T>& grav, void* extra_setting)
{
    WB::A_ = A; // floating base model mass inertia matrix
    WB::Ainv_ = Ainv;
    WB::cori_ = cori;
    WB::grav_ = grav;
    WB::b_updatesetting_ = true;

    (void)extra_setting;
}

template <typename T>
void WBIC<T>::_SetOptimizationSize() 
{
    // Dimension
    _dim_rf = 0;
    _dim_Uf = 0;  // Dimension of inequality constraint
    for (size_t i(0); i < (*_contact_list).size(); ++i) {
        _dim_rf += (*_contact_list)[i]->getDim(); // +3
        _dim_Uf += (*_contact_list)[i]->getDimRFConstraint(); // +6
    }

    _dim_opt = _dim_floating + _dim_rf; // 6 + 3*ContactNum
    _dim_eq_cstr = _dim_floating; // 6

    // Matrix Setting
    G.resize(0., _dim_opt, _dim_opt);
    g0.resize(0., _dim_opt);
    CE.resize(0., _dim_opt, _dim_eq_cstr);
    ce0.resize(0., _dim_eq_cstr);

    // Eigen Matrix Setting
    _dyn_CE = DMat<T>::Zero(_dim_eq_cstr, _dim_opt);
    _dyn_ce0 = DVec<T>(_dim_eq_cstr);
    if (_dim_rf > 0) {
        CI.resize(0., _dim_opt, _dim_Uf);
        ci0.resize(0., _dim_Uf);
        _dyn_CI = DMat<T>::Zero(_dim_Uf, _dim_opt);
        _dyn_ci0 = DVec<T>(_dim_Uf);

        _Jc = DMat<T>(_dim_rf, WB::num_qdot_);
        _JcDotQdot = DVec<T>(_dim_rf);
        _Fr_des = DVec<T>(_dim_rf);

        _Uf = DMat<T>(_dim_Uf, _dim_rf);
        _Uf.setZero();
        _Uf_ieq_vec = DVec<T>(_dim_Uf);
    } else {
        CI.resize(0., _dim_opt, 1);
        ci0.resize(0., 1);
    }
}

template class WBIC<float>;