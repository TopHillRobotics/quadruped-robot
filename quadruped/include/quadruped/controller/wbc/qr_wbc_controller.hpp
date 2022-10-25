// The MIT License

#ifndef WBC_CONTROLLER_HPP
#define WBC_CONTROLLER_HPP

#include "robots/qr_robot.h"
#include "dynamics/qr_floating_base_model.hpp"
#include "controller/wbc/qr_WBIC.hpp"
#include "controller/wbc/qr_kinematics_WBC.hpp"
#include "planner/qr_gait_generator.h"

template<typename T>
class qrWbcController
{
public:
    qrWbcController(qrRobot *robot);
    
    virtual ~qrWbcController();

    void run(void * input, std::vector<qrMotorCommand>& cmd);
    
    void setFloatingBaseWeight(const T & weight);
    qrWbcCtrlData* _input_data;
protected:
    virtual void _ContactTaskUpdate(qrWbcCtrlData *input) {};
    
    virtual void _LCM_PublishData() {}
    
    void _UpdateModel(qrRobot* robot);
    
    void _UpdateLegCMD(std::vector<qrMotorCommand> &cmd);
    
    void _ComputeWBC();

    const size_t dim_config = 18;

    // ControlFSMData<T>* controlFSMData;
    qrRobot *robot;

    KinWBC<T>* _kin_wbc;
    WBIC<T>* _wbic;
    WBIC_ExtraData<T>* _wbic_data;

    FloatingBaseModel<T> _model;
    std::vector<qrContactSpec<T> * > _contact_list;
    std::vector<qrTask<T> * > _task_list;

    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    FBModelState<T> _state;

    DVec<T> _full_config;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;

    std::vector<T> _Kp_joint, _Kd_joint;

    unsigned long long _iter;

};

#endif // WBC_CONTROLLER_H