/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: a interface of robot locomotion controller.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_COM_ADJUSTER_H_
#define ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_COM_ADJUSTER_H_

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <cmath>

#include "robots/robot.h"
#include "estimators/robot_estimator.h"
#include "gait/openloop_gait_generator.h"
#include "utils/se3.h"

namespace Quadruped {
    class ComAdjuster {
    private:
        friend class FakeComAdjuster;
        /**
         * @brief the vector index of ADJEST_LEG means the order of the legs,
         *        the value of ADJEST_LEG means adjacent two legs of the indexed leg, 
         *        where cw means clockwise leg, the ccw means counter-clockwise leg.
         */         
        const std::vector<std::map<std::string, int>> ADJEST_LEG{std::map<std::string, int>{{"cw", 2}, {"ccw", 1}},
                                                                 std::map<std::string, int>{{"cw", 0}, {"ccw", 3}},
                                                                 std::map<std::string, int>{{"cw", 3}, {"ccw", 0}},
                                                                 std::map<std::string, int>{{"cw", 1}, {"ccw", 2}}
        };

        Robot *robot;
        GaitGenerator *gaitGenerator;
        RobotEstimator *robotEstimator;

        Eigen::Vector3f basePosition;
        Eigen::Vector4f baseOrientation;
        Eigen::Vector3f inverseTranslation;
        Eigen::Matrix3f inverseRotation;
        Eigen::Matrix<float, 3, 1> comPosInBaseFrame;
        Eigen::Matrix<float, 3, 1> comPosInWorldFrame;
        Eigen::Matrix<int, 4, 1> legState;
        Eigen::Matrix<float, 4, 1> normalizedPhase;
        Eigen::Matrix<float, 3, 4> footPosition; // in base frame
        float contactK[4]; // is the foot contact with ground.
        float swingK[4]; // is it a swing foot ?
        float weightFactor[4]; // weight factors of vertices.
        Eigen::Matrix<float, 3, 4> supportPolygonVertices;
        float delta;

    public:
        ComAdjuster(Robot *robotIn,
                    GaitGenerator *gaitGeneratorIn,
                    RobotEstimator *robotEstimatorIn);

        ~ComAdjuster() = default;

        /**
         * @brief Called during the start of a controller.
         * @param current_time: The wall time in seconds.
         */
        void Reset(float currentTime);

        Eigen::Matrix<float, 3, 1> Update(float currentTime);

        Eigen::Matrix<float, 3, 1> TestUpdate(float currentTime);

        inline Eigen::Matrix<float, 3, 1> &GetComPosInBaseFrame()
        {
            return comPosInBaseFrame;
        }
    };
} //namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_COM_ADJUSTER_H_
