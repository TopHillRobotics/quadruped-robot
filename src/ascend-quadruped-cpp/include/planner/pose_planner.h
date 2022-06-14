/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: pose optimatization for walk gait.
* Author: Zhu Yijie & Zang Yaohua
* Create: 2021-20-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_POSE_PLANNER_H_
#define ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_POSE_PLANNER_H_

#include <map>
#include <set>

#include "robots/robot.h"
#include "state_estimator/robot_estimator.h"
#include "state_estimator/ground_estimator.h"
#include "mpc_controller/openloop_gait_generator.h"
#include "utils/se3.h"
#include "utils/geometry.h"

namespace Quadruped {
class PosePlanner {
public:
    float resetTime;
    const int legIdMap2CounterwiseOrder[4] = {0, 3, 1, 2};
    const int invLegIdMap2CounterwiseOrder[4] = {0, 2, 3, 1};
    // friend class FakeComAdjuster;
    Robot *robot;
    RobotEstimator *robotEstimator;
    GroundSurfaceEstimator *groundEstimator;
    OpenloopGaitGenerator *gaitGenerator;

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
public:
    PosePlanner(Robot *robotIn, RobotEstimator *robotEstimatorIn, GroundSurfaceEstimator *groundEstimatorIn, OpenloopGaitGenerator *gaitGeneratorIn);

    ~PosePlanner() = default;

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

        Eigen::Matrix<float, 3, 4> footPoseWorld = robot->GetFootPositionsInWorldFrame();
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
        // pose[2] = poseDest[2];
        // std::cout << "pose= " <<pose << std::endl;

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
};
} //namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_POSE_PLANNER_H_
