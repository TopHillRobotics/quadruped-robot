#include "controllers/wbc/wbc_locomotion_controller.hpp"

template<typename T>
WbcLocomotionCtrl<T>::WbcLocomotionCtrl(FloatingBaseModel<T> &model, ControlFSMData<T> *controlFSMDataIn)
    : fullConfig(NumMotor + 7), _tau_ff(NumMotor), _des_jpos(NumMotor), _des_jvel(NumMotor),
      controlFSMData(controlFSMDataIn), _model(model), dimConfig(NumMotor + BaseFreedomDim), _iter(0)
{
    fullConfig.setZero();
    zeroVec3.setZero();
    _state.q = DVec<T>::Zero(NumMotor);
    _state.qd = DVec<T>::Zero(NumMotor);

    _kin_wbc = new KinWBC<T>(dimConfig);// 18
    _wbic = new WBIC<T>(dimConfig, &contactList, &taskList);

    _wbicData = new WBIC_ExtraData<T>();
    _wbicData->_W_floating = DVec<T>::Constant(BaseFreedomDim, 0.1);// 6
    _wbicData->_W_rf = DVec<T>::Constant(12, 1);

    bodyPosTask = new BodyPosTask<T>(&_model);
    bodyOriTask = new BodyOriTask<T>(&_model);

    footContact[0] = new SingleContact<T>(&_model, Quadruped::linkID::FR);
    footContact[1] = new SingleContact<T>(&_model, Quadruped::linkID::FL);
    footContact[2] = new SingleContact<T>(&_model, Quadruped::linkID::HR);
    footContact[3] = new SingleContact<T>(&_model, Quadruped::linkID::HL);

    footTask[0] = new LinkPosTask<T>(&_model, Quadruped::linkID::FR);
    footTask[1] = new LinkPosTask<T>(&_model, Quadruped::linkID::FL);
    footTask[2] = new LinkPosTask<T>(&_model, Quadruped::linkID::HR);
    footTask[3] = new LinkPosTask<T>(&_model, Quadruped::linkID::HL);

    for (size_t i(0); i < 3; ++i) {
        ((BodyPosTask<T> *)bodyPosTask)->_Kp[i] = 100.;
        ((BodyPosTask<T> *)bodyPosTask)->_Kd[i] = 10.;

        ((BodyOriTask<T> *)bodyOriTask)->_Kp[i] = 100.;// 100， 50
        ((BodyOriTask<T> *)bodyOriTask)->_Kd[i] = 10.; // 10， 5

        for (size_t j(0); j < NumLeg; ++j) {
            ((LinkPosTask<T> *)footTask[j])->_Kp[i] = 500;
            ((LinkPosTask<T> *)footTask[j])->_Kd[i] = 10.;
        }
    }
}

template<typename T>
WbcLocomotionCtrl<T>::~WbcLocomotionCtrl()
{
    delete bodyPosTask;
    delete bodyOriTask;
    for (size_t i(0); i < 4; ++i) {
        delete footContact[i];
        delete footTask[i];
    }

    delete _kin_wbc;
    delete _wbic;
    delete _wbicData;

    typename std::vector<Task<T> *>::iterator iter = taskList.begin();
    while (iter < taskList.end()) {
        delete (*iter);
        ++iter;
    }
    taskList.clear();

    typename std::vector<SingleContact<T> *>::iterator iter2 = contactList.begin();
    while (iter2 < contactList.end()) {
        delete (*iter2);
        ++iter2;
    }
    contactList.clear();
}

template<typename T>
void WbcLocomotionCtrl<T>::Run(void *precomputeData)
{
    // MITTimer T1;
    if (_iter % 2 == 0) {
        MITTimer T1;
        // Update Model
        _UpdateModel(controlFSMData->quadruped);// 0.05ms
        // Task & Contact Update, 0.003ms
        _ContactTaskUpdate(static_cast<WbcCtrlData *>(precomputeData), controlFSMData);
        // WBC Computation, 0.165ms
        _kin_wbc->FindConfiguration(fullConfig, taskList, contactList,
                                    _des_jpos, _des_jvel, controlFSMData);
        _wbic->MakeTorque(_tau_ff, _wbicData, controlFSMData);
        // controlFSMData->quadruped->stateDataFlow.visualizer.sa[2].Update(T1.getMs());
    }
    // Update Leg Command
    _UpdateLegCMD(controlFSMData);
    // printf("wbc[%llu] SOLVE TIME: %.3f\n", _iter, T1.getMs());
    ++_iter;
}

template<typename T>
void WbcLocomotionCtrl<T>::_UpdateModel(Quadruped::Robot *robot)
{
    _state.bodyOrientation = robot->GetBaseOrientation();
    _state.bodyPosition = robot->GetBasePosition();
    auto vBody = robot->GetEstimatedVelocityInBaseFrame();
    auto q = robot->GetMotorAngles();
    auto dq = robot->GetMotorVelocities();
    auto omegaBody = robot->GetBaseRollPitchYawRate();
    for (size_t i(0); i < 3; ++i) {
        _state.bodyVelocity[i] = omegaBody[i];// in body frame
        _state.bodyVelocity[i + 3] = vBody[i];

        for (size_t leg(0); leg < NumLeg; ++leg) {
            size_t motorId = 3 * leg + i;
            _state.q[motorId] = q[motorId];
            _state.qd[motorId] = dq[motorId];
            fullConfig[motorId + BaseFreedomDim] = _state.q[motorId];
        }
    }
    _model.setState(_state);
    _model.contactJacobians();
    _model.massMatrix();
    _model.generalizedGravityForce();
    _model.generalizedCoriolisForce();

    _wbic->GetModelRes(_model);
}

template<typename T>
void WbcLocomotionCtrl<T>::_ContactTaskUpdate(WbcCtrlData *ctrlData, ControlFSMData<T> *controlFSMData)
{
    _ctrlData = ctrlData;

    // Wash out the previous setup
    contactList.clear();
    taskList.clear();

    Quat<T> quatDes = robotics::math::rpyToQuat(ctrlData->pBody_RPY_des);

    bodyOriTask->UpdateTask(&quatDes, ctrlData->vBody_Ori_des, zeroVec3);
    bodyPosTask->UpdateTask(&(ctrlData->pBody_des), ctrlData->vBody_des, ctrlData->aBody_des);

    taskList.push_back(bodyOriTask);
    taskList.push_back(bodyPosTask);

    for (size_t leg(0); leg < NumLeg; ++leg) {
        if (ctrlData->contact_state[leg]) {// Contact
            // std::cout << "[WBC] Stance leg" << leg << ", F = " << ctrlData->Fr_des[leg] <<std::endl;
            footContact[leg]->setRFDesired((DVec<T>)(ctrlData->Fr_des[leg]));
            footContact[leg]->UpdateContactSpec();
            contactList.push_back(footContact[leg]);
        } else {// No Contact (swing)
            // std::cout << "[WBC] Swing leg " << leg << std::endl;
            footTask[leg]->UpdateTask(
                &(ctrlData->pFoot_des[leg]),// in world frame, relative to origin.
                ctrlData->vFoot_des[leg],
                ctrlData->aFoot_des[leg]);
            //zeroVec3);
            taskList.push_back(footTask[leg]);
        }
    }
}

template<typename T>
void WbcLocomotionCtrl<T>::_UpdateLegCMD(ControlFSMData<T> *data)
{
    std::vector<Quadruped::MotorCommand> &cmd = data->legCmd;

    for (int leg(0); leg < NumLeg; ++leg) {
        for (int j(0); j < NumMotorOfOneLeg; ++j) {
            int motorId = NumMotorOfOneLeg * leg + j;
            // cmd[motorId].SetZero();
            if (_ctrlData->contact_state[leg]) {// stance
                cmd[motorId].tua = _tau_ff[motorId];
                cmd[motorId].p = _des_jpos[motorId];
                cmd[motorId].d = _des_jvel[motorId];
                cmd[motorId].Kp = 10;// 5
                // cmd[motorId].Kd = 2;
            }
        }
    }
    // std::cout<< "****************************" << std::endl;
    // Knee joint non flip barrier
    auto jointAngles = data->quadruped->GetMotorAngles();
    for (size_t leg(0); leg < NumLeg; ++leg) {
        size_t motorId = leg * 3 + 2;
        if (cmd[motorId].p > -0.3) {
            cmd[motorId].p = -0.3;
        }
        if (jointAngles[motorId] > -0.3) {
            float knee_pos = jointAngles[motorId];
            cmd[motorId].tua = -1. / (knee_pos * knee_pos + 0.02);
        }
    }
}

template class WbcLocomotionCtrl<float>;
