#ifndef WBC_LOCOMOTION_CONTROLLER
#define WBC_LOCOMOTION_CONTROLLER
#include "./WBIC.hpp"
#include "./kinWBC.hpp"
#include "./task_set/BodyOriTask.hpp"
#include "./task_set/BodyPosTask.hpp"
#include "./task_set/LinkPosTask.hpp"
#include "controllers/desired_state_command.hpp"
#include "robots/robot.h"

template<typename T>
class WbcLocomotionCtrl {
public:
    WbcLocomotionCtrl(FloatingBaseModel<T> &model, ControlFSMData<T> *controlFSMData);
    virtual ~WbcLocomotionCtrl();
    void Run(void *precomputeData);
protected:
    void _UpdateModel(Quadruped::Robot *robot);
    void _ContactTaskUpdate(WbcCtrlData *ctrlData, ControlFSMData<T> *controlFSMData);
    void _UpdateLegCMD(ControlFSMData<T> *data);

    Task<T> *bodyPosTask;
    Task<T> *bodyOriTask;
    Task<T> *footTask[4];
    SingleContact<T> *footContact[4];
    std::vector<SingleContact<T> *> contactList;
    std::vector<Task<T> *> taskList;

    const size_t dimConfig;
    WbcCtrlData *_ctrlData; 
    ControlFSMData<T> *controlFSMData;
    KinWBC<T> *_kin_wbc;
    WBIC<T> *_wbic;
    WBIC_ExtraData<T> *_wbicData;
    FloatingBaseModel<T> &_model;
    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;
    FBModelState<T> _state;
    DVec<T> fullConfig;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;
    
    unsigned long long _iter;
    Vec3<T> zeroVec3;
};

#endif