// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:tophill.robotics@gmail.com

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

#ifndef QR_POSE_PLANNER_H_
#define QR_POSE_PLANNER_H_

#include <map>
#include <set>

#include "robots/robot.h"
#include "state_estimator/robot_estimator.h"
#include "state_estimator/ground_estimator.h"
#include "mpc_controller/qr_gait_generator.h"
#include "utils/se3.h"
#include "utils/geometry.h"

namespace Quadruped {
class qrPosePlanner {
public:
    qrPosePlanner(Robot *robotIn, RobotEstimator *robotEstimatorIn, qrGroundSurfaceEstimator *groundEstimatorIn, qrGaitGenerator *gaitGeneratorIn);

    ~qrPosePlanner() = default;

    /**
     * @brief Called during the start of a controller.
     * @param current_time: The wall time in seconds.
     */
    void Reset(float currentTime) {};

    inline Vec6<float> GetBasePose()
    { return poseDest; }

    void ResetBasePose(float currentTime)
    {
        resetTime = currentTime;

        Eigen::Matrix<float, 3, 4> footPoseWorld = robot->state.GetFootPositionsInWorldFrame();
        pose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw(); // poseDest;
        poseDest << footPoseWorld.row(0).mean(), footPoseWorld.row(1).mean(), bodyHight,
                    0,0,0;
        twist << 0,0,0,0,0,0;
        std::cout <<"resetTime = "<<resetTime <<  ", reset poseDest = " << poseDest.transpose() << std::endl;
        segment.Reset(pose, poseDest);
    }

    std::tuple<Vec6<float>, Vec6<float>> GetIntermediateBasePose(float currentTime)
    {
        float dt = currentTime - resetTime;
        const float auxTime = 5.0;
        float phase = dt / auxTime;
        if (phase > 1.0) {
            phase = 1.0;
        }
        pose = segment.GetPoint(phase);
        std::cout <<"currentTime" << currentTime << ", dt = " << dt <<" phase=" << phase << std::endl;
        return {pose, twist};
    }


    std::tuple<Vec6<float>, Vec6<float>> GetIntermediateBasePose(float phase, float currentTime) {
        phase *= 1.0;
        if (phase>1.0) {
            phase = 1.0; // todo
        }

        pose = segment.GetPoint(phase);
        return {pose, twist};
    }

    Vec6<float> Update(float currentTime);

    Vec6<float> ComputeGradientF();

    Mat6<float> ComputeHessianF();

    Eigen::MatrixXf ComputeGradientG();

    std::vector<Mat6<float>> ComputeHessianG();
    
    Eigen::MatrixXf ComputeG();

    float ComputeF();

    Vec3<float> ProjectV(Vec3<float> v);

    std::tuple<Vec6<float>,Eigen::MatrixXf> QpSolver(Mat6<float>& hessF,
                                                    Mat6<float>& hessGSum,
                                                    Vec6<float>& gradientF,
                                                    Eigen::MatrixXf& gradientG,
                                                    Eigen::MatrixXf& GValue);

    void ToCounterClockOrder(Eigen::Matrix<float,3,4>& A) 
    {
        Eigen::PermutationMatrix<4, 4> perm;
        perm.indices() = { 0, 2, 3, 1};
        // Permutate cols
        A = A * perm;
    }

    void ToCounterClockOrder(bool array[4])
    {
        bool temp = array[1];
        array[1] = array[2];
        array[2] = array[3];
        array[3] = temp;
    }


    float resetTime;
    const int legIdMap2CounterwiseOrder[4] = {0, 3, 1, 2};
    const int invLegIdMap2CounterwiseOrder[4] = {0, 2, 3, 1};
    Robot *robot;
    RobotEstimator *robotEstimator;
    qrGroundSurfaceEstimator *groundEstimator;
    qrGaitGenerator *gaitGenerator;

    Eigen::Matrix<float, 3, 4> rBH; // hip (offset) position in base frame;
    Vec3<float> rIB; // base center position in World/Ineria frame
    Vec3<float> rIB_; // project rIB to world XY plane.
    Eigen::Matrix<float,3,4> rBF; // foot position in base frame;
    Eigen::Matrix<float,3,4> rIF; // foot position in world frame;
    Vec3<float> rBCOM; // com (offset) position in base frame; 
    Vec3<float> rICOMoffset;
    Vec3<float> rICOMoffset_;
    Vec3<float> rSP; // support polgen center in world frame, including z axis;
    Vec3<float> rSP_; // projected rSP in xy plane;

    int N = 4; // num of contact legs
    std::set<int> validContactPointId;
    float lMin = 0.22; // allowed minimum length of leg
    float lMax = 0.35; // allowed maximum length of leg
    Eigen::MatrixXf Asp; // Asp 's size maybe (3,3) or (4, 3)
    Eigen::MatrixXf bsp;
    Eigen::MatrixXf G;
    Eigen::MatrixXf Lambda; // lagrangue factors

    Eigen::Matrix<float, 3, 4> footPosition; // in base frame
    bool contactK[4]; // is the foot contact with ground.
    float contactLegNumber;
    bool swingK[4]; // is it a swing foot ?
    std::vector<Vec3<float>> supportPolygonVertices; // in world frame
    std::vector<Vec3<float>> projectedSupportPolygonVertices; // in world frame
    std::vector<Vec3<float>> g; // length of array from foot tip to base

    Vec3<float> so3Phi;
    Quat<float> quat;
    Vec3<float> rpy;
    Vec3<float> rIBSource;
    Vec6<float> poseDest;
    Vec6<float> poseSource;
    Vec6<float> pose;
    Vec6<float> twist;
    robotics::math::CubicSplineInSO3 tarjectorySo3;
    robotics::math::CubicSpline tarjectoryR3[3];
    robotics::math::Segment<float, Vec6<float>> segment;
    float omega = 0.5;
    float eps = 0.1;
    const float bodyHight = A1_BODY_HIGHT;
};
} //namespace Quadruped

#endif //QR_POSE_PLANNER_H_
