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

#include "controllers/wbc/qr_wholebody_impulse_ctrl.hpp"

#include "utils/qr_cpptypes.h"


template<typename T>
qrWholeBodyImpulseCtrl<T>::qrWholeBodyImpulseCtrl(
    size_t dim_qdot,
    const std::vector<qrSingleContact<T> *> *contact_list,
    const std::vector<qrTask<T> *> *task_list):

    dimFb(BaseFreedomDim),
    dimQdot(dim_qdot),
    numActJoint(dim_qdot - dimFb)
{
    Sf = DMat<T>::Zero(BaseFreedomDim, dimQdot);
    Sf.block(0, 0, 6, 6).setIdentity();
    identityMat = DMat<T>::Identity(dimQdot, dimQdot);

    contactList = contact_list;
    taskLisk = task_list;
}


template<typename T>
void qrWholeBodyImpulseCtrl<T>::GetModelRes(const FloatingBaseModel<T> & model)
{
    A = model.getMassMatrix();
    Gravity = model.getGravityForce();
    Coriolis = model.getCoriolisForce();
    Ainv = A.inverse();
    
    settingUpdated = true;
}


template<typename T>
void qrWholeBodyImpulseCtrl<T>::MakeTorque(DVec<T> &cmd, void *extraInput)
{
    if (!settingUpdated) {
        printf("[Wanning] WBIC setting is not done\n");
    }
    if (extraInput) {
        extraData = static_cast<qrWBICExtraData<T> *>(extraInput);
    }

    SetOptimizationSize();
    SetCost();

    /* Get dynamically consistent pseudo inverse matrix of jacobian,
     * and then compute the inital projection matrix for acceleration.
     */
    DVec<T> qddot_pre;
    DMat<T> JcBar;
    DMat<T> Npre;
    if (dimFr > 0) {
        ContactBuilding();
        SetInequalityConstraint(); /* Setup Inequality constraints by the way. */
        WeightedInverse(JC, Ainv, JcBar);
        qddot_pre = JcBar * (-JCDotQdot);
        Npre.noalias() = identityMat - JcBar * JC;
    } else {
        qddot_pre = DVec<T>::Zero(dimQdot);
        Npre = identityMat;
    }

    qrTask<T> *task;
    DMat<T> Jt, JtBar, JtPre;
    DVec<T> JtDotQdot, xddot;

    /* Null-space projection for accleration command. */
    for (size_t i = 0; i < taskLisk->size(); ++i) {
        task = (*taskLisk)[i];
        task->GetJt(Jt);
        task->GetJtDotQdot(JtDotQdot);
        task->GetXddotCmd(xddot);

        JtPre.noalias() = Jt * Npre;
        WeightedInverse(JtPre, Ainv, JtBar);

        qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
        if (i < taskLisk->size() - 1) {
            Npre = Npre * (identityMat - JtBar * JtPre);
        }
    }

    SetEqualityConstraint(qddot_pre);/* Setup the dynamics constraint. */

    T f = solve_quadprog(qpG, qpg0, qpCE, qpce0, qpCI, qpci0, qpz);

    /* qddot = qddot_cmd + delta_q */
    for (size_t i = 0; i < dimFb; ++i) {
        qddot_pre[i] += qpz[i];
    }

    GetSolution(qddot_pre, cmd);

    extraData->optimizedResult = DVec<T>(dimOptimal);
    for (size_t i = 0; i < dimOptimal; ++i) {
        extraData->optimizedResult[i] = qpz[i];
    }
}

template<typename T>
void qrWholeBodyImpulseCtrl<T>::SetEqualityConstraint(const DVec<T> &qddot)
{
    if (dimFr > 0) {
        /* The dynamicsCE will seems like: [ A6x6 | -Sf * JC^T ]. */
        CE.block(0, 0, dimEqConstraint, dimFb) = A.block(0, 0, dimFb, dimFb);
        CE.block(0, dimFb, dimEqConstraint, dimFr) = -Sf * JC.transpose();
        ce0 = -Sf * (A * qddot + Coriolis + Gravity - JC.transpose() * desiredFr);
    } else {
        CE.block(0, 0, dimEqConstraint, dimFb) = A.block(0, 0, dimFb, dimFb);
        ce0 = -Sf * (A * qddot + Coriolis + Gravity);
    }

    /* Convert to quadprogpp matrix/vector. */
    for (size_t i = 0; i < dimEqConstraint; ++i) {
        for (size_t j = 0; j < dimOptimal; ++j) {
            qpCE[j][i] = CE(i, j);
        }
        qpce0[i] = -ce0[i];
    }
}


template<typename T>
void qrWholeBodyImpulseCtrl<T>::SetInequalityConstraint()
{
    /* Force limitation and friction cone constraints.
     * W * fr > 0; fr = fr_MPC + v_fr.
     */
    CI.block(0, dimFb, dimIneqConstraint, dimFr) = UF;
    ci0 = ineqVec - UF * desiredFr;

    /* Convert to quadprogpp matrix/vector. */
    for (size_t i(0); i < dimIneqConstraint; ++i) {
        for (size_t j(0); j < dimOptimal; ++j) {
            qpCI[j][i] = CI(i, j);
        }
        qpci0[i] = -ci0[i];
    }
}


template<typename T>
void qrWholeBodyImpulseCtrl<T>::ContactBuilding()
{
    DMat<T> Uf;
    DVec<T> Uf_ieq_vec;
    DMat<T> Jc;
    DVec<T> JcDotQdot;
    size_t dim_accumul_rf = 0, dim_accumul_uf = 0;
    size_t dim_new_rf = 0, dim_new_uf = 0;

    for (size_t i(0); i < contactList->size(); ++i) {
        (*contactList)[i]->GetJc(Jc); /* Jc is 3x18 matrix. */
        (*contactList)[i]->GetJcDotQdot(JcDotQdot);
        dim_new_rf = (*contactList)[i]->GetDimContact();
        dim_new_uf = (*contactList)[i]->GetDimUf();

        /* Stacked contact jacobian. */
        JC.block(dim_accumul_rf, 0, dim_new_rf, dimQdot) = Jc;

        /* Stacked JCDotQdot. */
        JCDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

        /* Force part of matrix of inequality constraint. */
        (*contactList)[i]->GetUf(Uf);
        UF.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

        /* Vector of inequality constraint. */
        (*contactList)[i]->GetIneqVec(Uf_ieq_vec);
        ineqVec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

        /* Desired reaction force frame MPC. */
        desiredFr.segment(dim_accumul_rf, dim_new_rf) = (*contactList)[i]->GetDesiredFr();

        dim_accumul_rf += dim_new_rf;
        dim_accumul_uf += dim_new_uf;
    }
}


template<typename T>
void qrWholeBodyImpulseCtrl<T>::GetSolution(const DVec<T> &qddot, DVec<T> &cmd)
{
    DVec<T> tot_tau;

    if (dimFr > 0) {
        extraData->optimalFr = DVec<T>(dimFr);

        /* Store total reaction force to extra data. */
        for (size_t i = 0; i < dimFr; ++i) {
            extraData->optimalFr[i] = qpz[i + dimFb] + desiredFr[i];
        }
        tot_tau = A * qddot + Coriolis + Gravity - JC.transpose() * extraData->optimalFr;
    } else {
        tot_tau = A * qddot + Coriolis + Gravity;
    }
    cmd = tot_tau.tail(numActJoint);

    // std::cout <<"_dim_rf = " << dimFr << ", F2 = " << extraData->optimalFr << std::endl;
}


template<typename T>
void qrWholeBodyImpulseCtrl<T>::SetCost()
{
    size_t idx_offset(0);

    /* The cost includes two parts, the floating base part and reaction force part.
     * All stored into a diagnose matrix.
     */
    for (size_t i = 0; i < dimFb; ++i) {
        qpG[i + idx_offset][i + idx_offset] = extraData->weightFb[i];
    }

    idx_offset += dimFb;
    for (size_t i = 0; i < dimFr; ++i) {
        qpG[i + idx_offset][i + idx_offset] = extraData->weightFr[i];
    }
}


template<typename T>
void qrWholeBodyImpulseCtrl<T>::SetOptimizationSize()
{
    dimFr = 0;
    dimIneqConstraint = 0;
    for (size_t i(0); i < contactList->size(); ++i) {
        dimFr += (*contactList)[i]->GetDimContact();
        dimIneqConstraint += (*contactList)[i]->GetDimUf();
    }

    dimOptimal = dimFb + dimFr;/* 6 + 3 * ContactNum */
    dimEqConstraint = dimFb;

    qpG.resize(0., dimOptimal, dimOptimal);
    qpg0.resize(0., dimOptimal);
    qpCE.resize(0., dimOptimal, dimEqConstraint);
    qpce0.resize(0., dimEqConstraint);

    CE = DMat<T>::Zero(dimEqConstraint, dimOptimal);
    ce0 = DVec<T>(dimEqConstraint);
    if (dimFr > 0) {
        qpCI.resize(0., dimOptimal, dimIneqConstraint);
        qpci0.resize(0., dimIneqConstraint);
        CI = DMat<T>::Zero(dimIneqConstraint, dimOptimal);
        ci0 = DVec<T>(dimIneqConstraint);

        JC = DMat<T>(dimFr, dimQdot);
        JCDotQdot = DVec<T>(dimFr);
        desiredFr = DVec<T>(dimFr);

        UF = DMat<T>(dimIneqConstraint, dimFr);
        UF.setZero();
        ineqVec = DVec<T>(dimIneqConstraint);
    } else {
        qpCI.resize(0., dimOptimal, 1);
        qpci0.resize(0., 1);
    }
}


template<typename T>
void qrWholeBodyImpulseCtrl<T>::WeightedInverse(const DMat<T> &J, const DMat<T> &Winv, DMat<T> &Jinv, double threshold)
{
    /* J_bar = A_inv * J^T ( J * A_inv * J^T)^(-1). */
    DMat<T> temp(Winv * J.transpose());
    DMat<T> lambda(J * temp);
    DMat<T> lambda_inv;
    pseudoInverse(lambda, threshold, lambda_inv);
    Jinv.noalias() = temp * lambda_inv;
}


template class qrWholeBodyImpulseCtrl<float>;
