
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

#ifndef QR_FOOTHOLD_PLANNER_H_
#define QR_FOOTHOLD_PLANNER_H_


#include "planner/qr_foot_stepper.h"

/**
 * @brief plan the foothold fpr next swing stage.
 */
class qrFootholdPlanner {
public:
    /** 
     * @brief Constructor of qrFootholdPlanner.
     * @param robotIn The robot object pointer.
     * @param groundEstimator The ground estimator
     */
    qrFootholdPlanner(qrRobot *robotIn, qrGroundSurfaceEstimator *groundEsitmatorIn);

    /**
     * @brief Deconstruct a qrFootholdPlanner object.
     */
    ~qrFootholdPlanner() = default;

    /** 
     * @brief Reset the foothold planner
     */
    void Reset();

    /**
    * @brief only be called at the moment right before lift up legs.
    */
    void UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, std::vector<int> legIds={});

    /**
     * @brief compute desired foot-end position in walk mode
     * @param currentFootholds current foot-end position of all the leg
     * @param currentComPose current com postion and pose
     * @param desiredComPose desired com postion and pose
     * @param legIds the order of legs
     */
    Eigen::Matrix<float, 3, 4> ComputeNextFootholds(Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                    Eigen::Matrix<float, 6, 1>& currentComPose,
                                                    Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                    std::vector<int>& legIds);

    /**
     * @brief compute desired foot-end position delta in position mode
     * @param currentFootholds current foot-end position of all the leg
     */
    Eigen::Matrix<float, 3, 4> ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                        Eigen::Matrix<float, 6, 1> currentComPose,
                                                        Eigen::Matrix<float, 6, 1> desiredComPose,
                                                        std::vector<int> legIds);
    
    /**
     * @brief get desired com position and rpy
     */
    inline const Eigen::Matrix<float, 6, 1> &GetDesiredComPose() const
    {
        return desiredComPose;
    }

    /**
    * @brief get desired foot-end position delta
    * i.e. currentFootholds + desiredFootholdsOffset = desiredFootholds
    */
    inline const Eigen::Matrix<float, 3, 4> &GetFootholdsOffset() const
    {
        return desiredFootholdsOffset;
    }

    /**
    * @brief get desired com position and rpy
    */
    inline Eigen::Matrix<float, 6, 1> GetComGoal(Eigen::Matrix<float, 6, 1> currentComPose)
    {
        desiredComPose << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;
        return desiredComPose;
    }

    /**
     * @brief return foot-end position for walk mode in world frame
     */
    inline Vec3<float> GetFootholdInWorldFrame(int legId) {
        return desiredFootholds.col(legId);
    }

public:
    /** 
     * @brief qrRobot object.
     */
    qrRobot *robot;

    /** 
     * @brief qrGroundSurfaceEstimator object.
     */
    qrGroundSurfaceEstimator *groundEsitmator;

    /** 
     * @brief qrFootStepper object.
     */
    qrFootStepper *footstepper;

    /**
     * @brief current time from robot when call the reset fuction
     * 
     */
    float resetTime;

    /**
     * @brief reset time for footStepper
     * 
     */
    float timeSinceReset;

    /**
     * @brief the config of footStepper
     * 
     */
    YAML::Node footStepperConfig;

    /**
     * @brief describe terrain information of the map
     * 
     */
    qrTerrain& terrain;
    
    /**
     * @brief current com position and rpy
     */
    Eigen::Matrix<float, 6, 1> comPose;

    /**
     * @brief desired com position and rpy
     */
    Eigen::Matrix<float, 6, 1> desiredComPose;

    /**
     * @brief desired foot-end position delta for position mode
     */
    Eigen::Matrix<float, 3, 4> desiredFootholdsOffset;

    /**
     * @brief desired foot-end position for walk mode 
     */
    Eigen::Matrix<float, 3, 4> desiredFootholds;
};

#endif //QR_FOOTHOLD_PLANNER_H_