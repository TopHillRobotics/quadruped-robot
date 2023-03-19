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

#ifndef QR_COM_ADJUSTER_H
#define QR_COM_ADJUSTER_H

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <cmath>

#include "robots/qr_robot.h"
#include "estimators/qr_robot_estimator.h"
#include "gait/qr_openloop_gait_generator.h"
#include "utils/qr_se3.h"


namespace Quadruped {

/**
 * Plan the path of the center of mass(CoM) so that
 * the center of mass(CoM) falls inside the polygon
 */
class qrComAdjuster {

private:

    /**
     * @brief The vector index of ADJEST_LEG means the order of the legs,
     *        the value of ADJEST_LEG means adjacent two legs of the indexed leg, 
     *        where cw means clockwise leg, the ccw means counter-clockwise leg.
     */         
    const std::vector<std::map<std::string, int>> ADJEST_LEG{std::map<std::string, int>{{"cw", 2}, {"ccw", 1}},
                                                             std::map<std::string, int>{{"cw", 0}, {"ccw", 3}},
                                                             std::map<std::string, int>{{"cw", 3}, {"ccw", 0}},
                                                             std::map<std::string, int>{{"cw", 1}, {"ccw", 2}}
    };

    /**
     * @brief Robot class for CoM plan.
     */
    qrRobot *robot;

    /**
     * @brief GaitGenerator object.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief RobotEstimator object.
     */
    qrRobotEstimator *robotEstimator;

    /**
     * @brief Com position from robot state.
     */
    Eigen::Vector3f basePosition;

    /**
     * @brief CoM pose from robot state.
     */
    Eigen::Vector4f baseOrientation;

    /**
     * @brief CoM positon after ajusted in base frame.
     */
    Eigen::Matrix<float, 3, 1> comPosInBaseFrame;

    /**
     * @brief CoM position after ajusted in world frame.
     */
    Eigen::Matrix<float, 3, 1> comPosInWorldFrame;

    /**
     * @brief The state of each leg from qrGaitGenerator class,
     * e.g. SWING/STAND
     */
    Eigen::Matrix<int, 4, 1> legState;

    /**
     * @brief The relative phase for the desired state.
     */
    Eigen::Matrix<float, 4, 1> normalizedPhase;

    /**
     * @brief The foot-end position from robot state in base frame.
     */
    Eigen::Matrix<float, 3, 4> footPosition; // in base frame

    /**
     * @brief The probability of foot end touching the ground(range:0.~1.).
     */
    float contactK[4]; // is the foot contact with ground.

    /**
     * @brief The probability of each leg being the swing leg.
     */
    float swingK[4]; // is it a swing foot ?

    /**
     * @brief The probability of leg contact.
     * i.e weightFactor[legId] = contactK[legId] + swingK[legId]
     */
    float weightFactor[4]; // weight factors of vertices.

    /**
     * @brief Coordinates of each point in the support polygon.
     */
    Eigen::Matrix<float, 3, 4> supportPolygonVertices;

    /**
     * @brief Used to caculate contactK and swingK.
     */
    float delta;

public:

    /**
     * @brief Constructor of ComAdjuster class.
     * @param robotIn: the robot for CoM planner.
     * @param gaitGeneratorIn: generate desired gait.
     * @param robotEstimatorIn: estimate CoM pose and position.
     */
    qrComAdjuster(qrRobot *robotIn,
                qrGaitGenerator *gaitGeneratorIn,
                qrRobotEstimator *robotEstimatorIn);

    /**
     * @brief Destructor of ComAdjuster class.
     */
    ~qrComAdjuster() = default;

    /**
     * @brief Called during the start of a controller.
     * @param currentTime: The wall time in seconds.
     */
    void Reset(float currentTime);

    /**
     * @brief Update the contact state and virtual polygon.
     * the virtual support polygon biases away from legs nearing.
     * the end of their contact phase and towards legs about to.
     * touchdown.
     * @param currentTime: The wall time in seconds.
     * @return desired CoM position
     */
    Eigen::Matrix<float, 3, 1> Update(float currentTime);

    /**
     * @brief Getter method of member comPosInBaseFrame.
     */
    inline Eigen::Matrix<float, 3, 1> &GetComPosInBaseFrame() {
        return comPosInBaseFrame;
    };

};

} // Namespace Quadruped

#endif // QR_COM_ADJUSTER_H
