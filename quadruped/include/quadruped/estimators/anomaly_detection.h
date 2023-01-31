/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhu Yijie & Zang Yaohua
* Create: 2022-03-15
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_ROBOT_CONTACT_DETECTION_H
#define ASCEND_QUADRUPED_CPP_ROBOT_CONTACT_DETECTION_H

#include "robots/robot.h"
#include "estimators/filter.hpp"
#include "TinyEKF.h"
#include "estimators/ground_estimator.h"
#include "gait/gait.h"
#include "gait/walk_gait_generator.h"
#include "dynamics/physics_transform.h"

namespace Quadruped {
    class ContactDetection {
        public:
            ContactDetection(Robot* robotIn, GaitGenerator* gaitGeneratorIn, GroundSurfaceEstimator* groundEstimatorIn);

            void Update(float currentTime);

            void Reset(float currentTime);

            void UpdateSlip(float currentTime);

            void GMObserver(float currentTime);

            Vec4<float> JointObserver(float currentTime);
        
            Vec4<bool> GetIsContact()
            {
                return isContact;   
            }
        
        private:
            Robot* robot;
            GaitGenerator* gaitGenerator;
            GroundSurfaceEstimator* groundEstimator;
            TinyEKF<4, 12>* filterContact; // dim state=4, dim obs=8
            TinyEKF<4, 4>* filterSlip; // dim state=4, dim obs=4

            float timeSinceReset;
            float lastTime;
            Vec4<float> lastFootVzInControlFrame = Vec4<float>::Zero();
            /// contact ///
            Vec4<bool> isContact = {true, true, true, true}; // contact result for each leg
            double thresold[2] = {0.5,0.4};
            Eigen::Matrix<float, 4, 4> pContact; // probability of contact for each obs factor
            MovingWindowFilter<float, 3> windowFilter[4];   
            // for kalman filter computation
            double fx[4]; // fx = f(x,u)
            double hx[12]; // hx = h(x), it is determined by H matrix.
            double F[4][4]; // F = df/dx
            double H[12][4]; // H = dh/dx
            double z[12]; // z = hx, true observation.
            // param of kalman filter
            float sigmaPhase= 0.1f;
            Vec4<float> externalTorques;
            // for trot
            // Vec4<float> meanTorques = {4.0f,4.0f,4.0f,4.0f};
            // Vec4<float> sigmaTorques = {3.0f,3.0f,3.0f,3.0f};
            // for walk
            Vec4<float> meanTorques = {1.0f,1.0f,1.0f,1.0f};
            Vec4<float> sigmaTorques = {2.0f,2.0f,2.0f,2.0f};
            
            Vec4<float> meanPz = {-A1_BODY_HIGHT,-A1_BODY_HIGHT,-A1_BODY_HIGHT,-A1_BODY_HIGHT};
            Vec4<float> sigmaPz = {0.15f,0.15f,0.15f,0.15f};
            // for leg force computation   
            Eigen::Matrix<float, 3, 4> externalForces;
            std::vector<Mat3<float>> legInertias;
            Eigen::Matrix<float,12,1> lastJointVs;
            
            /// slip ///
            Eigen::Matrix<float, 2, 4> pSlip; // probability of slip for each obs factor
            Vec4<bool> isSlip = {false, false, false, false}; // contact result for each leg
            std::deque<Vec3<float>> vQue;
            Vec3<float> vSum = Vec3<float>::Zero();
            Vec3<float> vAvg = Vec3<float>::Zero();
            double fxSlip[4]; // fx = f(x,u)
            double hxSlip[4]; // hx = h(x), it is determined by H matrix.
            double FSlip[4][4]; // F = df/dx
            double HSlip[4][4]; // H = dh/dx
            double zSlip[4]; // z = hx, true observation.
            //
            long long count=0;
        };


    class SlipDetection {
        public:
            SlipDetection(Robot* robotIn, GaitGenerator* gaitGeneratorIn, GroundSurfaceEstimator* groundEstimatorIn);

            void Update(float currentTime);

        private:
            Robot* robot;
            // RobotEstimator* robotEstimator;

            float timeSinceReset;
            float lastTime;
            bool isContact;
            
    };


    class WorkspaceDetection {
    public:
        WorkspaceDetection(Robot* robotIn, GroundSurfaceEstimator* groundEstimatorIn);

        Eigen::Matrix<float,3,4> Update();

        void Detect();

    private:
        float currentTime;
        Vec3<float> allowedWorkSpace = {0.1f,0.1f,0.05f};
        Robot* robot;
        GroundSurfaceEstimator* groundEstimator;
    };

    class AnomalyDetection {
        public:
            AnomalyDetection(Robot* robotIn, GaitGenerator* gaitGeneratorIn, GroundSurfaceEstimator* groundEstimatorIn);

        private:
            Robot* robot;
            GaitGenerator* gaitGenerator;
            GroundSurfaceEstimator* groundEstimator;
            ContactDetection contactDetection;
            SlipDetection slipDetection;
            
    };
} // namespace Quadruped
#endif // ASCEND_QUADRUPED_CPP_ROBOT_CONTACT_DETECTION_H
