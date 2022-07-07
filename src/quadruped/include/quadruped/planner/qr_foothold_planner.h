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

#ifndef QR_FOOTHOLD_PLANNER_H_
#define QR_FOOTHOLD_PLANNER_H_

#include "planner/qr_foot_stepper.h"
#include "robot/qr_robot.h"

    class qrFootholdPlanner {
    public:
        // TODO:add robot module
        qrFootholdPlanner(qrRobot *robotIn, qrGroundSurfaceEstimator *groundSurfaceEstimatorIn);
        virtual ~qrFootholdPlanner() = default;

        void Reset();
        /**
         * @brief only be called at the moment right before lift up legs.
         */
        void UpdateOnce(Mat3x4<float> currentFootholds, std::vector<int> legIds={});

        /**
         * @brief compute desired foot-end position delta in position mode
         * @param currentFootholds current foot-end position of all the leg
         */
        void ComputeFootholdsOffset(Mat3x4<float> currentFootholds);

        /**
         * @brief compute desired foot-end position in walk mode
         * @param currentFootholds current foot-end position of all the leg
         * @param currentComPose current com postion and pose
         * @param desiredComPose desired com postion and pose
         * @param legIds the order of legs
         */
        void ComputeNextFootholds(Mat3x4<float>& currentFootholds,
                                  Vec6<float>& currentComPose,
                                  Vec6<float>& desiredComPose,
                                  std::vector<int>& legIds);
        
        /**
         * @brief get desired com position and rpy
         */
        inline const Vec6<float> &GetDesiredComPose() const
        {
            return desiredComPose;
        }

        /**
         * @brief get desired foot-end position delta
         * i.e. currentFootholds + desiredFootholdsOffset = desiredFootholds
         */
        inline const Mat3x4<float> &GetFootholdsOffset() const
        {
            return desiredFootholdsOffset;
        }

        /**
         * @brief get desired com position and rpy
         */
        inline Vec6<float> GetComGoal(Vec6<float> currentComPose)
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

        /**
         * @brief the class of robot state
         */
        qrRobot *robot;

        qrRobotState *robotState;

        /**
         * @brief the class of ground info
         */
        qrGroundSurfaceEstimator *groundEstimator;

        /**
         * @brief calculate the location of landing points based on specific terrain
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
        Terrain& terrain;
        
        /**
         * @brief current com position and rpy
         */
        Vec6<float> comPose;

        /**
         * @brief desired com position and rpy
         */
        Vec6<float> desiredComPose;

        /**
         * @brief desired foot-end position delta for position mode
         */
        Mat3x4<float> desiredFootholdsOffset;

        /**
         * @brief desired foot-end position for walk mode 
         */
        Mat3x4<float> desiredFootholds;
    };
#endif // QR_FOOTHOLD_PLANNER_H_
