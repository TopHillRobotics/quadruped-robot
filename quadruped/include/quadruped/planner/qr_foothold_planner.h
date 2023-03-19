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

#ifndef QR_Foothold_PLANNER_H
#define QR_Foothold_PLANNER_H

#include "fsm/qr_control_fsm_data.hpp"
#include "planner/qr_foot_stepper.h"


namespace Quadruped {

/**
 * @brief plan the foothold fpr next swing stage.
 */
class qrFootholdPlanner {

public:

    /**
     * @brief Constructor of qrFootholdPlanner.
     * @param robotIn The robot object pointer.
     * @param groundEstimator The ground estimator.
     */
    qrFootholdPlanner(qrRobot *quadrupedIn, qrGaitGenerator* gaitGeneratorIn,
                    qrStateEstimatorContainer* stateEstimatorsIn,
                    qrUserParameters* userParametersIn,
                    qrDesiredStateCommand* desiredStateCommandIn);

    /**
     * @brief Deconstruct a qrFootholdPlanner object.
     */
    ~qrFootholdPlanner() = default;

    /**
     * @brief Reset the foothold planner.
     */
    void Reset(float t);

    /**
     * @brief Update the foothold planner.
     */
    void Update() {
    };

    /**
     * @brief Only be called at the moment right before lift up legs.
     */
    void UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, std::vector<int> legIds={});

    /**
     * @brief Compute desired foot-end position in walk mode.
     * @param currentFootholds current foot-end position of all the leg.
     * @param currentComPose current com postion and pose.
     * @param desiredComPose desired com postion and pose.
     * @param legIds the order of legs.
     */
    Eigen::Matrix<float, 3, 4> ComputeNextFootholds(Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                    Eigen::Matrix<float, 6, 1>& currentComPose,
                                                    Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                    std::vector<int>& legIds);

    /**
     * @brief Compute desired foot-end position delta in position mode.
     * @param currentFootholds current foot-end position of all the leg.
     */
    Eigen::Matrix<float, 3, 4> ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                      Eigen::Matrix<float, 6, 1> currentComPose,
                                                      Eigen::Matrix<float, 6, 1> desiredComPose,
                                                      std::vector<int> legIds);

    /**
     * @brief Get desired com position and rpy.
     */
    inline const Eigen::Matrix<float, 6, 1> &GetDesiredComPose() const {
        return desiredComPose;
    };

    /**
    * @brief Get desired foot-end position delta.
    * i.e. currentFootholds + desiredFootholdsOffset = desiredFootholds.
    */
    inline const Eigen::Matrix<float, 3, 4> &GetFootholdsOffset() const {
        return desiredFootholdsOffset;
    };

    /**
    * @brief Get desired com position and rpy.
    */
    inline Eigen::Matrix<float, 6, 1> GetComGoal(Eigen::Matrix<float, 6, 1> currentComPose) {
        desiredComPose << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;
        return desiredComPose;
    };

    /**
     * @brief For walk mode in world frame.
     */
    inline Vec3<float> GetFootholdInWorldFrame(int legId) {
        return desiredFootholds.col(legId);
    };

    /**
     * @brief Compute desired foothold.
     * @param swingFootIds: the leg which is swing.
     */
    void ComputeHeuristicFootHold(std::vector<u8> swingFootIds);

    /**
     * @brief Compute desired foothold by MIT mehtod.
     * @param swingFootIds: which leg for caculation.
     */
    void ComputeMITFootHold(int legId);

public:

    /**
     * @brief qrRobot object.
     */
    qrRobot *robot;

    /**
     * @brief gaitGenerator object.
     */
    qrGaitGenerator* gaitGenerator;

    /**
     * @brief qrGroundSurfaceEstimator object.
     */
    qrGroundSurfaceEstimator *groundEsitmator;

    /**
     * @brief Some parameters for computation, inlcude EKF,
     * terrian, moving window, and so on;
     */
    qrUserParameters* userParameters;

    /**
     * @brief Desired state command for locomotion.
     */
    qrDesiredStateCommand* desiredStateCommand;

    /**
     * @brief qrFootStepper object.
     */
    qrFootStepper *footstepper;

    /**
     * @brief Current time from robot when call the reset fuction.
     */
    float resetTime;

    /**
     * @brief Reset time for footStepper.
     */
    float timeSinceReset;

    /**
     * @brief The config of footStepper.
     */
    YAML::Node footStepperConfig;

    /**
     * @brief Width of gap.
     */
    float gapWidth;

    /**
     * @brief Foot-end position delta for position mode.
     */
    float footHoldOffset;

    /**
     * @brief Describe gap information of the map.
     */
    std::vector<float> gaps;

    /**
     * @brief Describe terrain information of the map.
     */
    qrTerrain& terrain;

    /**
     * @brief Default map size.
     */
    constexpr static int N = 50; // default map size

    /**
     * @brief Current com position and rpy.
     */
    Eigen::Matrix<float, 6, 1> comPose;

    /**
     * @brief Desired com position and rpy.
     */
    Eigen::Matrix<float, 6, 1> desiredComPose;

    /**
     * @brief Desired foot-end position delta for position mode.
     */
    Eigen::Matrix<float, 3, 4> desiredFootholdsOffset;

    /**
     * @brief Desired foot-end position for walk mode.
     */
    Eigen::Matrix<float, 3, 4> desiredFootholds;

    /**
     * @brief The relative phase for the desired state.
     */
    Vec4<float> phase;

    /**
     * @brief Coefficient for desired foothold computation in swing leg.
     */
    Vec3<float> swingKp;

    /**
      * @brief The steps the dog move down.
      */
    int moveDown[4] = {0,0,0,0};

    /**
     * @brief firstSwingBaseState: position, velocity, roll pitch yaw and roll pitch yaw rate when reset.
     */
    Vec12<float> firstSwingBaseState;

};

} // Mamespace Quadruped

#endif // QR_FOOTHOLD_PLANNER_H
