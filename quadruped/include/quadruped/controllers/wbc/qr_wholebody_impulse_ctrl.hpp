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

#ifndef QR_WHOLE_BODY_IMPULSE_CTRL_H
#define QR_WHOLE_BODY_IMPULSE_CTRL_H

#include "Array.hh"
#include "QuadProg++.hh"

#include "qr_single_contact.hpp"
#include "task_set/qr_task.hpp"
#include "fsm/qr_control_fsm_data.hpp"


template<typename T>
class qrWBICExtraData {

public:

    qrWBICExtraData() = default;

    ~qrWBICExtraData() = default;

    /**
     * @brief The result of the QP problem.
     */
    DVec<T> optimizedResult;

    /**
     * @brief The optimized reaction force.
     * Force from MPC plus force from QP problem.
     */
    DVec<T> optimalFr;

    /**
     * @brief Weight of the floating base part in the QP problem.
     */
    DVec<T> weightFb;

    /**
     * @brief Weight of the reaction force part in the QP problem.
     */
    DVec<T> weightFr;


};

template<typename T>
class qrWholeBodyImpulseCtrl {

public:

    /**
     * @brief Constructor of class qrWholeBodyImpulseCtrl .
     * @param dim_qdot: dimention of qdot.
     * @param contact_list: contact constraint for all legs of robot.
     * @param tast_list: task list for the robot.
     */
    qrWholeBodyImpulseCtrl(size_t dim_qdot, const std::vector<qrSingleContact<T> *> *contact_list,
         const std::vector<qrTask<T> *> *task_list);

    virtual ~qrWholeBodyImpulseCtrl() = default;

    /**
     * @brief Get result of the MIT floating base model,
     * especially generalized mass matrix, coriolis force and generalized gravity.
     * @param fb_model: MIT floating base model.
     */
    void GetModelRes(const FloatingBaseModel<T> & fb_model);

    /**
     * @brief Given the task list, caculate current robot configuration
     * and caculate the torque command by robot dynamic formulation.
     * @param [out] cmd: output torque command for stance leg.
     * @param [in] extraInput: if use extra data, this pointer points to WBIC extra data structure.
     */
    void MakeTorque(DVec<T> &cmd, void *extraInput = NULL);

private:

    /**
     * @brief Compute a dynamically consistent weighted inverse matrix.
     * Only useful for full rank fat matrix.
     * @param [in] J: the input jacobian matrix
     * @param [in] Winv: inverse matrix of generalized mass matrix.
     * @param [out] Jinv: the pseudo inverse matrix of J.
     * @param threshold: threshold for singular values being zero.
     */
    void WeightedInverse(const DMat<T> &J, const DMat<T> &Winv, DMat<T> &Jinv, double threshold = 0.0001);

    /**
     * @brief Setup the dynamic equality constraint.
     * @param qddot: derivative of qdot.
     */
    void SetEqualityConstraint(const DVec<T> &qddot);

    /**
     * @brief Setup inequality constraint, including conic and boundary constraints.
     */
    void SetInequalityConstraint();

    /**
     * @brief Build the contact-related matrices, including JC, JCDotQdot and UF.
     */
    void ContactBuilding();

    /**
     * @brief Given the acceleration command of generalized coordinate,
     * compute the total torque command of stance leg by dynamic formulation.
     */
    void GetSolution(const DVec<T> &qddot, DVec<T> &cmd);

    /**
     * @brief Set the weight of target used in QP problem.
     */
    void SetCost();

    /**
     * @brief Setup the size of  matrices used in the QP problem.
     */
    void SetOptimizationSize();

    /**
     * @brief Dimension of floating base. Usually set to 6.
     */
    const size_t dimFb;

    /**
     * @brief Dimension of qdot. Usually set to 18.
     */
    const size_t dimQdot;

    /**
     * @brief Numeber of actuated joints. For quadruped, this is set to 12.
     */
    const size_t numActJoint;

    /**
     * @brief Selection matrix of floating base in dynamics equation.
     */
    DMat<T> Sf;

    /**
     * @brief Generalized mass inertia matrix of floating base model.
     */
    DMat<T> A;

    /**
     * @brief Inverse of generalized mass inertia matrix.
     */
    DMat<T> Ainv;

    /**
     * @brief Coriolis and centrifugal matrix of floating base model.
     */
    DVec<T> Coriolis;

    /**
     * @brief Generalized gravitational matrix of floating base model .
     */
    DVec<T> Gravity;

    /**
     * @brief Set to true if WBIC has get the results of MIT floating base model.
     */
    bool settingUpdated;

    /**
     * @brief List that stores all contact constraints.
     * Will be used for null-space projection.
     */
    const std::vector<qrSingleContact<T> *> *contactList;

    /**
     * @brief List that stores all kinematic tasks,
     * including body orientation, body position and link positions.
     */
    const std::vector<qrTask<T> *> *taskLisk;

    /**
     * @brief Dimension of the optimal variable, including dimension of floating base and reaction forces.
     */
    size_t dimOptimal;

    /**
     * @brief Dimension of equality constraints. Usually set to 6.
     */
    size_t dimEqConstraint;

    /**
     * @brief Dimension of reaction force.
     * Equal to 3 times num of contact points.
     */
    size_t dimFr;

    /**
     * @brief Dimension of inequality constraints.
     * Equal to 6 times num of contact points.
     */
    size_t dimIneqConstraint;

    /**
     * @brief Pointer to ExtraData, which stores weights and results of the QP problem.
     */
    qrWBICExtraData<T> *extraData;

    /**
     * @brief Equality constraint matrix of the QP problem in Eigen form.
     */
    DMat<T> CE;

    /**
     * @brief Linear vector of the equality constraint of the QP problem in Eigen form.
     */
    DVec<T> ce0;

    /**
     * @brief Inequality constraint matrix of the QP problem in Eigen form.
     */
    DMat<T> CI;

    /**
     * @brief Linear vector of the inequality constraint of the QP problem in Eigen form.
     */
    DVec<T> ci0;

    /**
     * @brief Identity matrix used for initial calculation of acceleration command.
     */
    DMat<T> identityMat;

    /**
     * @brief Force constraint segment of inequality constraint matrix.
     * @see qrSingleContact::Uf
     */
    DMat<T> UF;

    /**
     * @brief Linear vector of inequality constraint matrix
     * @see qrSingleContact::ineqVec
     */
    DVec<T> ineqVec;

    /**
     * @brief Stacked contact jacobian including all single contact jacobians.
     * @see qrSingleContact::Jc
     */
    DMat<T> JC;

    /**
     * @brief Stacked JcDotQdot including all single JcDotQdot.
     * @see qrSingleContact::JcDotQdot
     */
    DVec<T> JCDotQdot;

    /**
     * @brief Desired reaction force computed from MPC solver.
     */
    DVec<T> desiredFr;

    /**
     * @brief Result vector of the QP problem in quadprogpp form.
     */
    quadprogpp::Vector<double> qpz;

    /**
     * @brief Hessian matrix of the QP problem in quadprogpp form.
     */
    quadprogpp::Matrix<double> qpG;

    /**
     * @brief Gradient vector of the QP problem in quadprogpp form.
     */
    quadprogpp::Vector<double> qpg0;

    /**
     * @brief Equality constraint matrix of the QP problem in quadprogpp form.
     */
    quadprogpp::Matrix<double> qpCE;

    /**
     * @brief Linear vector of the equality constraint of the QP problem in quadprogpp form.
     */
    quadprogpp::Vector<double> qpce0;

    /**
     * @brief Inequality constraint matrix of the QP problem in quadprogpp form.
     */
    quadprogpp::Matrix<double> qpCI;

    /**
     * @brief Linear vector of the inequality constraint of the QP problem in quadprogpp form.
     */
    quadprogpp::Vector<double> qpci0;

};

#endif // QR_WHOLE_BODY_IMPULSE_CTRL_H
