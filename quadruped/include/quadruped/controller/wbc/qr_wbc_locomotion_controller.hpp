// The MIT License

#ifndef QR_WBC_LOCOMOTION_CONTROLLER_HPP
#define QR_WBC_LOCOMOTION_CONTROLLER_HPP

#include "controller/wbc/qr_wbc_controller.hpp"
#include "controller/wbc/qr_single_contact.hpp"
#include "controller/wbc/qr_task_set/BodyOriTask.hpp"
#include "controller/wbc/qr_task_set/BodyPosTask.hpp"
#include "controller/wbc/qr_task_set/LinkPosTask.hpp"

template<typename T>
class qrWbcLocomotionCtrl: public qrWbcController<T>{
  public:
    qrWbcLocomotionCtrl(qrRobot* robot);
    virtual ~qrWbcLocomotionCtrl();

  protected:
    virtual void _ContactTaskUpdate(qrWbcCtrlData *input);
    // void _ParameterSetup(const UserParameters* param);
    void _CleanUp();
    
    //virtual void _LCM_PublishData();

    qrTask<T>* _body_pos_task;
    qrTask<T>* _body_ori_task;

    qrTask<T>* _foot_task[4];
    qrContactSpec<T>* _foot_contact[4];

    Vec3<T> pre_foot_vel[4];

    Vec3<T> _Fr_result[4];
    Quat<T> _quat_des;
};

#endif //QR_WBC_LOCOMOTION_CONTROLLER_HPP