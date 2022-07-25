
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
namespace Quadruped {
    /**
     * @brief plan the foothold fpr next swing stage.
     */
    class qrFootholdPlanner {
    public:

        qrFootholdPlanner(Robot *robotIn, qrGroundSurfaceEstimator *groundEsitmatorIn);

        ~qrFootholdPlanner() = default;

        void Reset();

        void Update()
        {}
        /**
         * @brief only be called at the moment right before lift up legs.
         */
        void UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds, std::vector<int> legIds={});

        bool Loadterrain( std::string& configPathIn);

        // for postion mode 
        Eigen::Matrix<float, 3, 4> ComputeNextFootholds(Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                        Eigen::Matrix<float, 6, 1>& currentComPose,
                                                        Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                        std::vector<int>& legIds);

        // for walk mode 
        Eigen::Matrix<float, 3, 4> ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                          Eigen::Matrix<float, 6, 1> currentComPose,
                                                          Eigen::Matrix<float, 6, 1> desiredComPose,
                                                          std::vector<int> legIds);

        inline const Eigen::Matrix<float, 6, 1> &GetDesiredComPose() const
        {
            return desiredComPose;
        }

        inline const Eigen::Matrix<float, 3, 4> &GetFootholdsOffset() const
        {
            return desiredFootholdsOffset;
        }

        inline Eigen::Matrix<float, 6, 1> GetComGoal(Eigen::Matrix<float, 6, 1> currentComPose)
        {
            desiredComPose << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;
            return desiredComPose;
        }

        /**
         * @brief for walk mode in world frame
         */
        inline Vec3<float> GetFootholdInWorldFrame(int legId) {
            return desiredFootholds.col(legId);
        }

    public:

        Robot *robot;
        qrGroundSurfaceEstimator *groundEsitmator;
        qrFootStepper *footstepper;

        float resetTime;
        float timeSinceReset;

        YAML::Node footStepperConfig;
        float gapWidth;
        float footHoldOffset;
        std::vector<float> gaps;
        qrTerrain& terrain;
        constexpr static int N = 50; // default map size
        
        long long unsigned stepCount;
        Eigen::Matrix<float, 6, 1> comPose;
        Eigen::Matrix<float, 6, 1> desiredComPose;
        Eigen::Matrix<float, 3, 4> desiredFootholdsOffset;
        Eigen::Matrix<float, 3, 4> desiredFootholds;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_Foothold_PLANNER_H_
