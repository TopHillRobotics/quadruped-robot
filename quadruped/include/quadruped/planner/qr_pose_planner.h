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

#ifndef QR_POSE_PLANNER_H
#define QR_POSE_PLANNER_H

#include <map>
#include <set>

#include "robots/qr_robot.h"
#include "estimators/qr_state_estimator_container.h"
#include "gait/qr_openloop_gait_generator.h"
#include "utils/qr_se3.h"
#include "utils/qr_geometry.h"


namespace Quadruped {

/**
 * @brief plan the base position and base pose. note that the
 * base point and CoM point is not the same point.
 */
class qrPosePlanner {

public:

    /**
     * @brief Reset time for footStepper.
     */
    float resetTime;

    /**
     * @brief Leg id for each leg in counter clock wise order.
     * leg id:
     * 3---0
     * |   |
     * 1---2
     */
    const int legIdMap2CounterwiseOrder[4] = {0, 3, 1, 2};

    /**
     * @brief Leg id for each leg in clock wise order.
     * leg id:
     * 1---0
     * |   |
     * 3---2
     */
    const int invLegIdMap2CounterwiseOrder[4] = {0, 2, 3, 1};

    /**
     * @brief qrRobot object.
     */
    qrRobot *robot;

    /**
     * @brief RobotEstimator object.
     */
    qrRobotEstimator *robotEstimator;

    /**
     * @brief GroundSurfaceEstimator object.
     */
    qrGroundSurfaceEstimator *groundEstimator;

    /**
     * @brief GaitGenerator object.
     */
    qrGaitGenerator *gaitGenerator;

    /**
     * @brief Hip (offset) position in base frame.
     * note: see [ETH:qrRobot Dynamics Lecture Notes] for
     * learing these representations.
     */
    Eigen::Matrix<float, 3, 4> rBH;

    /**
     * @brief Base center position in World/Ineria frame.
     */
    Vec3<float> rIB;

    /**
     * @brief Project rIB to world XY plane.
     */
    Vec3<float> rIB_;
    /**
     * @brief Foot position in base frame.
     */
    Eigen::Matrix<float,3,4> rBF;

    /**
     * @brief Foot position in world frame.
     */
    Eigen::Matrix<float,3,4> rIF;

    /**
     * @brief CoM position in base frame.
     */
    Vec3<float> rBCOM;

    /**
     * @brief CoM offset position in base frame.
     */
    Vec3<float> rICOMoffset;

    /**
     * @brief Projected com offset position in base frame.
     */
    Vec3<float> rICOMoffset_;

    /**
     * @brief Support polgen center in world frame, including z axis.
     */
    Vec3<float> rSP;

    /**
     * @brief Projected rSP in xy plane.
     */
    Vec3<float> rSP_;

    /**
     * @brief Num of legs which may contact.
     */
    int N = 4;

    /**
     * @brief Valid contact point of leg's Id.
     */
    std::set<int> validContactPointId;

    /**
     * @brief Allowed minimum length of leg.
     */
    float lMin = 0.22;

    /**
     * @brief Allowed maximum length of leg.
     */
    float lMax = 0.35;

    /**
     * @brief Used for gradient computation.
     * Asp 's size maybe (3,3) or (4, 3)
     */
    Eigen::MatrixXf Asp;

    /**
     * @brief Used for gradient computation.
     */
    Eigen::MatrixXf bsp;

    /**
     * @brief Used for QP solve.
     */
    Eigen::MatrixXf G;

    /**
     * @brief Lagrangue factors.
     */
    Eigen::MatrixXf Lambda;

    /**
     * @brief FootPosition in base frame.
     */
    Eigen::Matrix<float, 3, 4> footPosition;

    /**
     * @brief Is the foot contact with ground.
     */
    bool contactK[4];

    /**
     * @brief The number of contacted legs.
     */
    float contactLegNumber;

    /**
     * @brief Is it a swing foot.
     */
    bool swingK[4];

    /**
     * @brief Points of the support polygon in world frame.
     */
    std::vector<Vec3<float>> supportPolygonVertices;

    /**
     * @brief Projected points of the support polygon in world frame.
     */
    std::vector<Vec3<float>> projectedSupportPolygonVertices;

    /**
     * @brief Length of array from foot tip to base.
     */
    std::vector<Vec3<float>> g;

    /**
     * @brief Used for support polygon computation.
     */
    Vec3<float> so3Phi;

    /**
     * @brief Quaternion that express the base orientation.
     */
    Quat<float> quat;

    /**
     * @brief The base row-pitch-yaw in world frame.
     */
    Vec3<float> rpy;

    /**
     * @brief Estimated base position in world frame.
     */
    Vec3<float> rIBSource;

    /**
     * @brief Desired base pose and position.
     */
    Vec6<float> poseDest;

    /**
     * @brief Estimated base pose and position in world frame.
     */
    Vec6<float> poseSource;

    /**
     * @brief Base pose and position in world frame.
     */
    Vec6<float> pose;

    /**
     * @brief Base liner velocity and angular velocity in world frame.
     */
    Vec6<float> twist;

    /**
     * @brief Stores the given data.
     */
    robotics::math::qrSegment<float, Vec6<float>> segment;

    /**
     * @brief Used for gradient cumputation.
     */
    float omega = 0.5;

    /**
     * @brief Used for G cumputation.
     */
    float eps = 0.1;

    /**
     * @brief Initial a1 robot body height.
     */
    const float bodyHight = A1_BODY_HIGHT;

public:

    /**
     * @brief Constructor of PosePlanner class.
     * @param robotIn: the robot for pose planner.
     * @param gaitGeneratorIn: generate desired gait.
     * @param stateEstimators: estimate current kinematic state.
     */
    qrPosePlanner(qrRobot *robotIn, qrGaitGenerator *gaitGeneratorIn, qrStateEstimatorContainer* stateEstimators);

    /**
      * @brief Destructor of PosePlanner class.
      */
    ~qrPosePlanner() = default;

    /**
     * @brief Called during the start of a controller.
     * @param current_time: The wall time in seconds.
     */
    void Reset(float currentTime) {
    };

    /**
     * @brief Getter method of member poseDest.
     */
    inline Vec6<float> GetBasePose() {
        return poseDest;
    };

    /**
     * @brief Reset BasePose planner.
     * @param currentTime: The wall time in seconds.
     */
    void ResetBasePose(float currentTime) {
        resetTime = currentTime;

        Eigen::Matrix<float, 3, 4> footPoseWorld = robot->GetFootPositionsInWorldFrame();
        pose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw(); // poseDest;
        poseDest << footPoseWorld.row(0).mean(), footPoseWorld.row(1).mean(), bodyHight, 0, 0, 0;
        twist << 0,0,0,0,0,0;
        std::cout <<"resetTime = "<<resetTime <<  ", reset poseDest = " << poseDest.transpose() << std::endl;
        segment.Reset(pose, poseDest);
    };

    /**
     * @brief Given the current time, get the corresponding position and pose.
     * @param currentTime: The wall time in seconds.
     * @return corresponding position and pose.
     */
    std::tuple<Vec6<float>, Vec6<float>> GetIntermediateBasePose(float currentTime) {
        float dt = currentTime - resetTime;
        const float auxTime = 5.0;
        float phase = dt / auxTime;
        if (phase > 1.0) {
            phase = 1.0;
        }
        pose = segment.GetPoint(phase);
        std::cout <<"currentTime" << currentTime << ", dt = " << dt <<" phase=" << phase << std::endl;
        return {pose, twist};
    };

    /**
     * @brief Given the current time, get the corresponding position and pose.
     * @param phase: relative time in the gait cycle.
     * @param currentTime: The wall time in seconds.
     * @return corresponding position and pose.
     */
    std::tuple<Vec6<float>, Vec6<float>> GetIntermediateBasePose(float phase, float currentTime) {
        phase *= 1.0;
        if (phase>1.0) {
            phase = 1.0; // todo
        }
        pose = segment.GetPoint(phase);
        // pose[2] = poseDest[2];

        // robotics::math::Spline::Point point;
        // for(int i=0;i<3;++i) {
        //     tarjectoryR3[i].getPoint(currentTime, point);// segment.GetPoint(phase).head(3);
        //     pose[i] = point.x;
        //     twist[i] = point.xd;
        // }
        // Mat3<float> outR;
        // Vec3<float> outwb;
        // tarjectorySo3.GetPoint(currentTime, outR, outwb);
        // pose.tail(3) = robotics::math::rotationMatrixToRPY(outR.transpose());
        // twist.tail(3) = outwb; 
        return {pose, twist};
    };

    /**
     * @brief Caculate the desired pose destination.
     * @param currentTime: The wall time in seconds.
     * @return desired pose destination.
     */
    Vec6<float> Update(float currentTime);

    /**
     * @brief Compute Gradient of F for QP.
     * @return Gradient of F.
     */
    Vec6<float> ComputeGradientF();

    /**
     * @brief Compute Hessian of F for QP.
     * @return Hessian of F.
     */
    Mat6<float> ComputeHessianF();

    /**
     * @brief Compute Gradient of G for QP.
     * @return Gradient of G.
     */
    Eigen::MatrixXf ComputeGradientG();

    /**
     * @brief Compute Hessian of G for QP.
     * @return Hessian of G.
     */
    std::vector<Mat6<float>> ComputeHessianG();
    
    /**
     * @brief Compute G.
     * @return G.
     */
    Eigen::MatrixXf ComputeG();

    /**
     * @brief Compute F.
     * @return F.
     */
    float ComputeF();

    /**
     * @brief Project a vector.
     * @param input vector.
     * @return projected vector.
     */
    Vec3<float> ProjectV(Vec3<float> v);

    /**
     * @brief solve qp problem.
     * @param hessF.
     * @param hessGSum.
     * @param gradientF.
     * @param gradientG.
     * @param GValue.
     * @return desired pose and Lagrange factor.
     */
    std::tuple<Vec6<float>,Eigen::MatrixXf> QpSolver(Mat6<float>& hessF,
                                                     Mat6<float>& hessGSum,
                                                     Vec6<float>& gradientF,
                                                     Eigen::MatrixXf& gradientG,
                                                     Eigen::MatrixXf& GValue);

    /**
     * @brief Permutate matrix cols in counter clock order.
     * @param input&output matrix.
     */
    void ToCounterClockOrder(Eigen::Matrix<float,3,4>& A) {
        Eigen::PermutationMatrix<4, 4> perm;
        perm.indices() = { 0, 2, 3, 1};
        // Permutate cols
        A = A * perm;
    };

    /**
     * @brief Permutate array cols in counter clock order.
     * @param input&output array.
     */
    void ToCounterClockOrder(bool array[4]) {
        bool temp = array[1];
        array[1] = array[2];
        array[2] = array[3];
        array[3] = temp;
    };

};

} // Namespace Quadruped

#endif // QR_POSE_PLANNER_H
