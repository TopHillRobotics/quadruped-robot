#ifndef KINEMATICS_WHOLE_BODY_CONTROL
#define KINEMATICS_WHOLE_BODY_CONTROL

#include "./single_contact.hpp"
#include "./task_set/task.hpp"
#include "fsm/control_fsm_data.hpp"

template<typename T>
class KinWBC {
public:
    KinWBC(size_t num_qdot);
    ~KinWBC() {}

    bool FindConfiguration(const DVec<T> &curr_config,
                           const std::vector<Task<T> *> &task_list,
                           const std::vector<SingleContact<T> *> &contact_list,
                           DVec<T> &jpos_cmd, DVec<T> &jvel_cmd, ControlFSMData<T> *controlFSMData);

private:
    void _PseudoInverse(const DMat<T> &J, DMat<T> &Jinv);
    void _BuildProjectionMatrix(const DMat<T> &J, DMat<T> &N);

    double threshold_;
    const size_t num_qdot_;
    const size_t num_act_joint_;
    DMat<T> I_mtx;
    DMat<T> Nc;
};

#include "controllers/wbc/kinWBC.hxx"
#endif
