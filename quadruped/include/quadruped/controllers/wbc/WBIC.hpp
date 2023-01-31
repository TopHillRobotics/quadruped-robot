#ifndef WHOLE_BODY_IMPULSE_CONTROL_H
#define WHOLE_BODY_IMPULSE_CONTROL_H

#include "Array.hh"
#include "QuadProg++.hh"

#include "./single_contact.hpp"
#include "./task_set/task.hpp"
#include "fsm/control_fsm_data.hpp"

template<typename T>
class WBIC_ExtraData {
public:
    // Output
    DVec<T> _opt_result;
    DVec<T> _qddot;
    DVec<T> _Fr;
    // Input
    DVec<T> _W_floating;
    DVec<T> _W_rf;

    WBIC_ExtraData() {}
    ~WBIC_ExtraData() {}
};

template<typename T>
class WBIC {
public:
    WBIC(size_t num_qdot, const std::vector<SingleContact<T> *> *contact_list,
         const std::vector<Task<T> *> *task_list);
    virtual ~WBIC() {}

    void GetModelRes(const FloatingBaseModel<T> & model);

    void MakeTorque(DVec<T> &cmd, void *extraInput = NULL,
                    ControlFSMData<T> *controlFSMData = NULL);

private:
    // full rank fat matrix only
    void _WeightedInverse(const DMat<T> &J, const DMat<T> &Winv, DMat<T> &Jinv,
                          double threshold = 0.0001);

    // Assume first 6 (or 3 in 2D case) joints are for the representation of
    // a floating base.
    const size_t _dim_floating;
    const size_t num_qdot_;     // 18
    const size_t num_act_joint_;// 12

    // DMat<T> Sa_;// Actuated joint
    DMat<T> Sv_;// Virtual joint

    DMat<T> A_;// floating base model mass inertia matrix
    DMat<T> Ainv_;
    DVec<T> cori_;
    DVec<T> grav_;

    bool bUpdateSetting;
    //
    const std::vector<SingleContact<T> *> *_contact_list;
    const std::vector<Task<T> *> *_task_list;

    void _SetEqualityConstraint(const DVec<T> &qddot);
    void _SetInEqualityConstraint();
    void _ContactBuilding();

    void _GetSolution(const DVec<T> &qddot, DVec<T> &cmd);
    void _SetCost();
    void _SetOptimizationSize();

    size_t _dim_opt;    // Contact pt delta, First task delta, reaction force
    size_t _dim_eq_cstr;// equality constraints

    size_t _dim_rf;// inequality constraints
    size_t _dim_Uf;

    WBIC_ExtraData<T> *_data;

    quadprogpp::Vector<double> z;
    // Cost
    quadprogpp::Matrix<double> G;
    quadprogpp::Vector<double> g0;

    // Equality
    quadprogpp::Matrix<double> CE;
    quadprogpp::Vector<double> ce0;

    // Inequality
    quadprogpp::Matrix<double> CI;
    quadprogpp::Vector<double> ci0;

    DMat<T> _dyn_CE;
    DVec<T> _dyn_ce0;
    DMat<T> _dyn_CI;
    DVec<T> _dyn_ci0;

    DMat<T> _eye;

    DMat<T> _Uf;
    DVec<T> _Uf_ieq_vec;

    DMat<T> _Jc;
    DVec<T> _JcDotQdot;
    DVec<T> _Fr_des;
};

#include "controllers/wbc/WBIC.hxx"
#endif
