// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

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

#include "controllers/wbc/qr_wbc_locomotion_controller.hpp"


template<typename T>
qrWbcLocomotionController<T>::qrWbcLocomotionController(FloatingBaseModel<T> &model, qrControlFSMData<T> *controlFSMDataIn):
    fullConfig(NumMotor + 7), jointTorqueCmd(NumMotor), desiredJPos(NumMotor), desiredJVel(NumMotor),
    controlFSMData(controlFSMDataIn), fbModel(model), dimConfig(NumMotor + BaseFreedomDim), iteration(0)
{
    fullConfig.setZero();
    zeroVec3.setZero();
    modelState.q = DVec<T>::Zero(NumMotor);
    modelState.qd = DVec<T>::Zero(NumMotor);

    multitask = new qrMultitaskProjection<T>(dimConfig);
    wbic = new qrWholeBodyImpulseCtrl<T>(dimConfig, &contactList, &taskList);

    wbicExtraData = new qrWBICExtraData<T>();
    // wbicExtraData->weightFb = DVec<T>::Constant(BaseFreedomDim, 1);
    wbicExtraData->weightFb = DVec<T>::Constant(BaseFreedomDim, 0.1);
    wbicExtraData->weightFr = DVec<T>::Constant(12, 1);

    taskBodyOri = new qrTaskBodyOrientation<T>(&fbModel);
    taskBodyPos = new qrTaskBodyPosition<T>(&fbModel);

    footContact[0] = new qrSingleContact<T>(&fbModel, Quadruped::linkID::FR);
    footContact[1] = new qrSingleContact<T>(&fbModel, Quadruped::linkID::FL);
    footContact[2] = new qrSingleContact<T>(&fbModel, Quadruped::linkID::HR);
    footContact[3] = new qrSingleContact<T>(&fbModel, Quadruped::linkID::HL);

    taskFootPos[0] = new qrTaskLinkPosition<T>(&fbModel, Quadruped::linkID::FR);
    taskFootPos[1] = new qrTaskLinkPosition<T>(&fbModel, Quadruped::linkID::FL);
    taskFootPos[2] = new qrTaskLinkPosition<T>(&fbModel, Quadruped::linkID::HR);
    taskFootPos[3] = new qrTaskLinkPosition<T>(&fbModel, Quadruped::linkID::HL);

    for (size_t i = 0; i < 3; ++i) {
        // ((qrTaskBodyPosition<T> *)taskBodyPos)->Kp[i] = 280.;
        // ((qrTaskBodyPosition<T> *)taskBodyPos)->Kd[i] = 20.;
        ((qrTaskBodyPosition<T> *)taskBodyPos)->Kp[i] = 100.;
        ((qrTaskBodyPosition<T> *)taskBodyPos)->Kd[i] = 10.;

        // ((qrTaskBodyOrientation<T> *)taskBodyOri)->Kp[i] = 120.;
        ((qrTaskBodyOrientation<T> *)taskBodyOri)->Kp[i] = 100.;
        ((qrTaskBodyOrientation<T> *)taskBodyOri)->Kd[i] = 10.;

        for (size_t j(0); j < NumLeg; ++j) {
            ((qrTaskLinkPosition<T> *)taskFootPos[j])->Kp[i] = 500;
            ((qrTaskLinkPosition<T> *)taskFootPos[j])->Kd[i] = 10.;
        }
    }
}


template<typename T>
qrWbcLocomotionController<T>::~qrWbcLocomotionController()
{
    delete taskBodyPos;
    delete taskBodyOri;
    for (size_t i(0); i < 4; ++i) {
        delete footContact[i];
        delete taskFootPos[i];
    }

    delete multitask;
    delete wbic;
    delete wbicExtraData;

    typename std::vector<qrTask<T> *>::iterator iter = taskList.begin();
    while (iter < taskList.end()) {
        delete (*iter);
        ++iter;
    }
    taskList.clear();

    typename std::vector<qrSingleContact<T> *>::iterator iter2 = contactList.begin();
    while (iter2 < contactList.end()) {
        delete (*iter2);
        ++iter2;
    }
    contactList.clear();
}


template<typename T>
void qrWbcLocomotionController<T>::Run(void *precomputeData)
{
    /* WBC is computed every 3 iterations. */
    if (iteration % 2 == 0) {
        qrWbcCtrlData *beforeWbcData = static_cast<qrWbcCtrlData *>(precomputeData);
        // std::cout <<"pBody_des = " << beforeWbcData->pBody_des.transpose() << std::endl;
        // std::cout <<"vBody_des = " << beforeWbcData->vBody_des.transpose() << std::endl;
        // std::cout <<"pBody_RPY_des = " << beforeWbcData->pBody_RPY_des.transpose() << std::endl;
        // std::cout <<"vBody_Ori_des = " << beforeWbcData->vBody_Ori_des.transpose() << std::endl;
        // std::cout <<"contact_state = " << beforeWbcData->contact_state.transpose() << std::endl;

        /* Update floating base model. */
        UpdateModel(controlFSMData->quadruped);

        /* Update Task & Contact Jacobian and Command. */
        ContactTaskUpdate(static_cast<qrWbcCtrlData *>(precomputeData), controlFSMData);

        /* Null space projection. Get desired position and velocity for PD controller. */
        multitask->FindConfiguration(
            fullConfig, taskList, contactList, desiredJPos, desiredJVel);
        wbic->MakeTorque(jointTorqueCmd, wbicExtraData);
    }

    UpdateLegCMD(controlFSMData);

    ++iteration;
}


template<typename T>
void qrWbcLocomotionController<T>::UpdateModel(Quadruped::qrRobot *robot)
{
    /* Setup base and joint information of the floating base system. */
    modelState.bodyOrientation = robot->GetBaseOrientation();
    modelState.bodyPosition = robot->GetBasePosition();
    auto vBody = robot->GetBaseVelocityInBaseFrame();
    auto q = robot->GetMotorAngles();
    auto dq = robot->GetMotorVelocities();
    auto omegaBody = robot->GetBaseRollPitchYawRate();
    for (size_t i = 0; i < 3; ++i) {
        modelState.bodyVelocity[i] = omegaBody[i];// in body frame
        modelState.bodyVelocity[i + 3] = vBody[i];

        for (size_t leg = 0; leg < NumLeg; ++leg) {
            size_t motorId = 3 * leg + i;
            modelState.q[motorId] = q[motorId];
            modelState.qd[motorId] = dq[motorId];
            fullConfig[motorId + BaseFreedomDim] = modelState.q[motorId];
        }
    }

    /* Update the floating base model. */
    fbModel.setState(modelState);
    fbModel.contactJacobians();
    fbModel.massMatrix();
    fbModel.generalizedGravityForce();
    fbModel.generalizedCoriolisForce();

    /* Update WBIC data from floating base model. */
    wbic->GetModelRes(fbModel);
}


template<typename T>
void qrWbcLocomotionController<T>::ContactTaskUpdate(qrWbcCtrlData *ctrlData, qrControlFSMData<T> *controlFSMData)
{
    /* Contains orientation, CoM position and foot positions tasks. */
    wbcCtrlData = ctrlData;

    contactList.clear();
    taskList.clear();

    Quat<T> quatDes = robotics::math::rpyToQuat(ctrlData->pBody_RPY_des); /* Desired quaternion. */

    taskBodyOri->UpdateTask(&quatDes, ctrlData->vBody_Ori_des, zeroVec3);
    taskBodyPos->UpdateTask(&(ctrlData->pBody_des), ctrlData->vBody_des, ctrlData->aBody_des);

    taskList.push_back(taskBodyOri);
    taskList.push_back(taskBodyPos);

    for (size_t leg = 0; leg < NumLeg; ++leg) {
        if (ctrlData->contact_state[leg]) {
            footContact[leg]->SetDesiredFr((DVec<T>)(ctrlData->Fr_des[leg]));
            footContact[leg]->UpdateContactSpec();
            contactList.push_back(footContact[leg]);
        } else {
            taskFootPos[leg]->UpdateTask(
                &(ctrlData->pFoot_des[leg]),
                ctrlData->vFoot_des[leg],
                ctrlData->aFoot_des[leg]);
            taskList.push_back(taskFootPos[leg]);
        }
    }
}

//write the data into cmd
template<typename T>
void qrWbcLocomotionController<T>::UpdateLegCMD(qrControlFSMData<T> *data)
{
    std::vector<Quadruped::qrMotorCommand> &cmd = data->legCmd;

    for (int leg = 0; leg < NumLeg; ++leg) {
        for (int j = 0; j < NumMotorOfOneLeg; ++j) {
            int motorId = NumMotorOfOneLeg * leg + j;

            /* Only consider stance legs. */
            if (wbcCtrlData->contact_state[leg]) {// stance
                cmd[motorId].tua = jointTorqueCmd[motorId];
            }
        }
    }
}


template class qrWbcLocomotionController<float>;
