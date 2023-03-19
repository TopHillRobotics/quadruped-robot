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

#ifndef QR_CONTACT_DETECTION_H
#define QR_CONTACT_DETECTION_H


#include "robots/qr_robot.h"
#include "estimators/qr_moving_window_filter.hpp"

/* this is a external lib, but using some variables defined in qr_filter.hpp*/
#include "TinyEKF.h"

#include "estimators/qr_ground_surface_estimator.h"
#include "gait/qr_gait.h"
#include "gait/qr_walk_gait_generator.h"
#include "utils/physics_transform.h"


namespace Quadruped {

class qrContactDetection {

public:

    /**
     * @brief Constructor of the qrContactDetection class.
     * @param robotIn: the robot class for the contact detection.
     * @param gaitGeneratorIn: generate desired gait schedule for locomotion.
     * @param groundEstimatorIn: estimate the 3D plane where the feet contact.
     */
    qrContactDetection(qrRobot* robotIn, qrGaitGenerator* gaitGeneratorIn, qrGroundSurfaceEstimator* groundEstimatorIn);

    /**
     * @brief Update the contact state, the prior data.
     * and observed data are used for estimation by Kalman filter.
     * after then, calculate the probability of whether a slip occurs.
     * @param currentTime: currunt time since the timer started.
     */
    void Update(float currentTime);

    /**
     * @brief Reset the estimation.
     * @param currentTime: currunt time since the timer started.
     */
    void Reset(float currentTime) {
    };

    /**
     * @brief Calculate the probability of whether a slip occurs.
     * @param currentTime: currunt time since the timer started.
     */
    void UpdateSlip(float currentTime);

    /**
     * @brief Warning: do nothing.
     * @param currentTime: currunt time since the timer started.
     */
    void GMObserver(float currentTime);

    /**
     * @brief Caculate the external torques on the legs.
     * @param currentTime: currunt time since the timer started.
     * @return exteral torques on the legs.
     */
    Vec4<float> JointObserver(float currentTime);

    /**
     * @brief Getter method of member isContact.
     */
    Vec4<bool> GetIsContact() {
        return isContact;
    };

private:

    /**
     * @brief the robot class for the anomaly detection.
     */
    qrRobot* robot;

    /**
     * @brief generate desired gait schedule for locomotion.
     */
    qrGaitGenerator* gaitGenerator;

    /**
     * @brief estimate the 3D plane where the feet contact.
     */
    qrGroundSurfaceEstimator* groundEstimator;

    /**
     * @brief kalman fitler for the contact state detection.
     */
    TinyEKF<4, 12>* filterContact; // dim state=4, dim obs=8

    /**
     * @brief kalman fitler for the slip detection.
     */
    TinyEKF<4, 4>* filterSlip; // dim state=4, dim obs=4

    /**
     * @brief the time since the timer restarted.
     */
    float timeSinceReset;

    /**
     * @brief stores last time stamp.
     */
    float lastTime;

    /**
     * @brief stores foothold velocities expressed in control frame in last loop.
     */
    Vec4<float> lastFootVzInControlFrame = Vec4<float>::Zero();

    /**
     * @brief contact result for each leg of robot.
     */
    Vec4<bool> isContact = {true, true, true, true}; // contact result for each leg

    /**
     * @brief threshold for contact detection.
     */
    double thresold[2] = {0.5,0.4};

    /**
     * @brief probability of contact for each obs factor.
     */
    Eigen::Matrix<float, 4, 4> pContact; // probability of contact for each obs factor

    /**
     * @brief moving window filters for foot velocity.
     */
    qrMovingWindowFilter<float, 3> windowFilter[4];

    // for kalman filter computation
    /**
     * @brief (kalman filter:) state transition function.
     */
    double fx[4]; // fx = f(x,u)

    /**
     * @brief (kalman filter:) transfer function from the state quantity to the observed quantity.
     */
    double hx[12]; // hx = h(x), it is determined by H matrix.

    /**
     * @brief (kalman filter:) state transition matrix.
     */
    double F[4][4]; // F = df/dx

    /**
     * @brief (kalman filter:) observation matrix.
     */
    double H[12][4]; // H = dh/dx

    /**
     * @brief (kalman filter:) observed quantity.
     */
    double z[12]; // z = hx, true observation.

    // param of kalman filter
    /**
     * @brief parameter for contact probability estimation,
     * this method is explained in MIT cheetah3: Virtual Predictive Support Polygon.
     */
    float sigmaPhase= 0.1f;

    /**
     * @brief external torques on legs of robot.
     */
    Vec4<float> externalTorques;
    // for trot
    // Vec4<float> meanTorques = {4.0f,4.0f,4.0f,4.0f};
    // Vec4<float> sigmaTorques = {3.0f,3.0f,3.0f,3.0f};

    // for walk
    /**
     * @brief mean of the external torque on 4 legs.
     */
    Vec4<float> meanTorques = {1.0f,1.0f,1.0f,1.0f};

    /**
     * used for compute ekf observer Variance.
     */
    Vec4<float> sigmaTorques = {2.0f,2.0f,2.0f,2.0f};
    
    /**
     * used for compute ekf observer Variance.
     */
    Vec4<float> meanPz = {-A1_BODY_HIGHT,-A1_BODY_HIGHT,-A1_BODY_HIGHT,-A1_BODY_HIGHT};

    /**
     * used for compute ekf observer Variance.
     */
    Vec4<float> sigmaPz = {0.15f,0.15f,0.15f,0.15f};

    // for leg force computation   
    /**
     * @brief external force on legs.
     */
    Eigen::Matrix<float, 3, 4> externalForces;

    /**
     * @brief inertia matrix of the leg.
     */
    std::vector<Mat3<float>> legInertias;

    /**
     * @brief joint angular velocity in last time.
     */
    Eigen::Matrix<float,12,1> lastJointVs;
    
    /// slip ///
    /**
     * @brief probability of slip for each obs factor.
     */
    Eigen::Matrix<float, 2, 4> pSlip; // probability of slip for each obs factor

    /**
     * @brief slip result for each leg.
     */
    Vec4<bool> isSlip = {false, false, false, false}; // contact result for each leg

    /**
     * @brief stores velocity values for a period of time,
     * used for get the standard deviation.
     */
    std::deque<Vec3<float>> vQue;

    /**
     * @brief sum of velocity values over a period of time.
     */
    Vec3<float> vSum = Vec3<float>::Zero();

    /**
     * @brief average of velocity values over a period of time.
     */
    Vec3<float> vAvg = Vec3<float>::Zero();

    /**
     * @brief (kalman filter:) state transition function for slip detection.
     */
    double fxSlip[4]; // fx = f(x,u)

    /**
     * @brief (kalman filter:) transfer function from the state quantity to the observed quantity.
     */
    double hxSlip[4]; // hx = h(x), it is determined by H matrix.

    /**
     * @brief (kalman filter:) state transition matrix.
     */
    double FSlip[4][4]; // F = df/dx

    /**
     * @brief (kalman filter:) observation matrix.
     */
    double HSlip[4][4]; // H = dh/dx

    /**
     * @brief (kalman filter:) observed quantity.
     */
    double zSlip[4]; // z = hx, true observation.

    //! maybe need delete.
    long long count=0;

};

/**
 * @brief detect whether the robot foot slides
 */
class SlipDetection {

public:

    /**
     * @brief constructor of SlipDetection class
     * @param robotIn: the robot class for slip detection
     * @param gaitGeneratorIn: generate desired gait schedule for locomotion
     * @param groundEstimatorIn: estimate the 3D plane where the feet contact
     */
    SlipDetection(qrRobot* robotIn, qrGaitGenerator* gaitGeneratorIn, qrGroundSurfaceEstimator* groundEstimatorIn);

    /**
     * @brief Detect whether the robot foot slides
     * @param currentTime: currunt time since the timer started
     */
    //! maybe need delete
    void Update(float currentTime);

private:

    /**
     * @brief robot class for slip detection
     */
    qrRobot* robot;

    /**
     * @brief the time since the timer restarted
     */
    float timeSinceReset;

    /**
     * @brief stores last time stamp
     */
    float lastTime;

    /**
     * @brief contact result for each leg of robot
     */
    bool isContact;
        
};

/**
 * @brief detects whether legs are outside the workspace
 */
//! maybe need delete
class WorkspaceDetection {

public:

    /**
     * @brief constructor of workspace detection class
     * @param robotIn: the robot class for the contact detection
     * @param groundEstimatorIn: estimate the 3D plane where the feet contact
     */
    WorkspaceDetection(qrRobot* robotIn, qrGroundSurfaceEstimator* groundEstimatorIn): robot(robotIn), groundEstimator(groundEstimator) {
    };

    /**
     * @brief Cohen-Sutherland method to detect if the footposition outside the workspace
     * if outside, then normalize the foot position in hip frame and limit it to workspace
     * @return the limited footposition in base frame
     */
    //! maybe need delete
    Eigen::Matrix<float,3,4> Update();

    //! maybe need delete
    void Detect() {
    };

private:

    float currentTime;

    Vec3<float> allowedWorkSpace = {0.1f,0.1f,0.05f};

    qrRobot* robot;

    qrGroundSurfaceEstimator* groundEstimator;

};

//! maybe need delete
class AnomalyDetection {

public:

    AnomalyDetection(qrRobot* robotIn, qrGaitGenerator* gaitGeneratorIn, qrGroundSurfaceEstimator* groundEstimatorIn);

private:

    qrRobot* robot;

    qrGaitGenerator* gaitGenerator;

    qrGroundSurfaceEstimator* groundEstimator;

    qrContactDetection contactDetection;

    SlipDetection slipDetection;

};

} // Namespace Quadruped

#endif // QR_CONTACT_DETECTION_H
