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

#ifndef QR_MPC_STANCE_LEG_CONTROLLER_H
#define QR_MPC_STANCE_LEG_CONTROLLER_H

#include "qr_mpc_interface.h"
#include "fsm/qr_control_fsm_data.hpp"
#include "controllers/balance_controller/qr_torque_stance_leg_controller.h"
#include "controllers/wbc/qr_wbc_locomotion_controller.hpp"

namespace Quadruped {

class MPCStanceLegController : public TorqueStanceLegController {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief constructor of MPCStanceLegController.
     * @param robot: pointer to robot.
     * @param gaitGenerator: pointer to gait generator.
     * @param stateEstimators: pointer to state estimator container.
     * @param comAdjuster: pointer to CoM adjuster.
     * @param posePlanner: pointer to pose planner.
     * @param footholdPlanner: pointer to foothold planner.
     * @param userParameters: reference to user parameters.
     * @param configFilepath: the string of path to config file.
     */
    MPCStanceLegController(qrRobot *robot,
                           qrGaitGenerator *gaitGenerator,
                           qrStateEstimatorContainer *stateEstimators,
                           qrComAdjuster *comAdjuster,
                           qrPosePlanner *posePlanner,
                           qrFootholdPlanner *footholdPlanner,
                           qrUserParameters &userParameters,
                           std::string configFilepath);

    /**
     * @brief Reset the stance leg controller with current time.
     * @param currentTime: current time to reset.
     */
    virtual void Reset(float t);

    /**
     * @brief Get the motor commands of the stance leg controller
     * @return commands and forces calculated by MPC.
     */
    virtual std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

    /**
     * @brief Setup MPC problem and solve the MPC problem.
     * @param legCommand: output of the MPC problem.
     * This may be removed in the future.
     * @param gaitType: the gait type of current state.
     * This may be removed in the future.
     * @param robotMode: current robot mode.
     * This may be removed in the future.
     */
    void Run(std::map<int, qrMotorCommand> &legCommand, int gaitType, int robotMode = 0);

private:

    /**
     * @brief Setup the desired command of the MPC problem.
     */
    void SetupCommand();

    /**
     * @brief Update MPC state of this iteration.
     * @param robot: pointer to robot.
     */
    void UpdateMPC(qrRobot *robot);

    /**
     * @brief Solve the MPC problem
     * @param robot: pointer to robot.
     */
    void SolveDenseMPC(qrRobot *robot);

    /**
     * @brief Turn rate of yaw.
     */
    float yawTurnRate = 0.0;

    /**
     * @brief Desired yaw.
     */
    float yawDesTrue = 0.0;

    /**
     * @brief Desired roll.
     */
    float rollDes;

    /**
     * @brief Desired pitch.
     */
    float pitchDes;

    /**
     * @brief Desired velocity along x axis.
     */
    float xVelDes = 0.0;

    /**
     * @brief Desired velocity along y axis.
     */
    float yVelDes = 0.0;

    /**
     * @brief Current body height.
     */
    float bodyHeight;

    /**
     * @brief Roll, pitch and yaw after compensation.
     */
    Vec3<float> rpyComp;

    /**
     * @brief Times of iterations in one MPC iteration
     */
    int iterationsInaMPC;

    /**
     * @brief Future steps in one MPC iteration
     */
    const int horizonLength;

    /**
     * @brief Times of MPC iterations in a gait period
     */
    int numHorizonL = 1;

    /**
     * @brief The iterations in one MPC
     */
    int defaultIterationsInMpc;

    /**
     * @brief Control frequency of the controller
     */
    float dt;

    /**
     * @brief Time for one step of MPC
     */
    float dtMPC;

    /**
     * @brief An iteration count for controlling the frequency of MPC updating.
     */
    unsigned long long iterationCounter = 0;

    /**
     * @brief MPC table storing the contact state with %horizonLength.
     */
    Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> mpcTable;

    /**
     * @brief The force that leg excerts to the ground in base frame.
     */
    Eigen::Matrix<float, 3, 4> f_ff;

    /**
     * @brief The reaction force frome ground in world frame.
     */
    Eigen::Matrix<float, 3, 4> f;

    /**
     * @brief Desired position in world frame.
     */
    Vec3<float> posDesiredinWorld;

    /**
     * @brief Desired linear velocity in world frame.
     */
    Vec3<float> vDesWorld;

    /**
     * @brief Current foot positions in world frame.
     */
    Eigen::Matrix<float, 3, 4> pFoot;

    /**
     * @brief MPC trajectory reference.
     */
    float trajAll[20 * 36];

    /**
     * @brief Q vector which stores the pose & twist weights.
     */
    float Q[12];

    /**
     * @brief A bool variable indicating whether MPC has been updated.
     */
    bool mpcUpdated = false;

    /**
     * @brief A bool variable indicating whether Whole Body Control will be used.
     */
    bool useWBC = false;
};

} // Namespace Quadruped

#endif // QR_MPC_STANCE_LEG_CONTROLLER_H
