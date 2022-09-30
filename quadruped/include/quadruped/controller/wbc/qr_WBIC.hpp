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

#ifndef QR_WHOLE_BODY_IMPLUSE_CONTROL_HPP
#define QR_WHOLE_BODY_IMPLUSE_CONTROL_HPP

#include "quadprogpp/QuadProg++.hh"
#include "quadprogpp/Array.hh"

#include "controller/wbc/qr_WBC.hpp"


// template <typename T>
// class WBIC_ExtraData {
// public:
//     // Output
//     DVec<T> _opt_result;
//     DVec<T> _qddot;
//     DVec<T> _Fr;

//     // Input
//     DVec<T> _W_floating;
//     DVec<T> _W_rf;

//     WBIC_ExtraData() {}
//     ~WBIC_ExtraData() {}
//     };

//     template <typename T>
//     class WBIC : public WBC<T> {
//     public:
//     WBIC(size_t num_qdot, const std::vector<qrContactSpec<T>*>* contact_list,
//         const std::vector<Task<T>*>* task_list);
//     virtual ~WBIC() {}

//     virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
//                                 const DVec<T>& cori, const DVec<T>& grav,
//                                 void* extra_setting = NULL);

//     virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL);

// private:
//     const std::vector<qrContactSpec<T>*>* _contact_list;
//     const std::vector<Task<T>*>* _task_list;

//     void _SetEqualityConstraint(const DVec<T>& qddot);
//     void _SetInEqualityConstraint();
//     void _ContactBuilding();

//     void _GetSolution(const DVec<T>& qddot, DVec<T>& cmd);
//     void _SetCost();
//     void _SetOptimizationSize();

//     size_t _dim_opt;      // Contact pt delta, First task delta, reaction force
//     size_t _dim_eq_cstr;  // equality constraints

//     size_t _dim_rf;  // inequality constraints
//     size_t _dim_Uf;

//     size_t _dim_floating;

//     WBIC_ExtraData<T>* _data;

//     quadprogpp::Vector<double> z;
//     // Cost
//     quadprogpp::Matrix<double> G;
//     quadprogpp::Vector<double> g0;

//     // Equality
//     quadprogpp::Matrix<double> CE;
//     quadprogpp::Vector<double> ce0;

//     // Inequality
//     quadprogpp::Matrix<double> CI;
//     quadprogpp::Vector<double> ci0;

//     DMat<T> _dyn_CE;
//     DVec<T> _dyn_ce0;
//     DMat<T> _dyn_CI;
//     DVec<T> _dyn_ci0;

//     DMat<T> _eye;
//     DMat<T> _eye_floating;

//     DMat<T> _S_delta;
//     DMat<T> _Uf;
//     DVec<T> _Uf_ieq_vec;

//     DMat<T> _Jc;
//     DVec<T> _JcDotQdot;
//     DVec<T> _Fr_des;

//     DMat<T> _B;
//     DVec<T> _c;
//     DVec<T> task_cmd_;
// };

#endif //QR_WHOLE_BODY_IMPLUSE_CONTROL_HPP