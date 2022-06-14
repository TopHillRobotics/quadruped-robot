/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the ground of quadruped.
* Author: Zhu Yijie
* Create: 2021-11-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_GROUND_ESTIMATOR_H
#define ASCEND_QUADRUPED_CPP_GROUND_ESTIMATOR_H

#include "utils/se3.h"
#include "robots/robot.h"

namespace Quadruped{

    struct Gap {
        float distance; // between COM and center of the gap.
        float width; // of the gap
        Eigen::Matrix<float, 3, 1> startPoint; // the closest point on the gap margin in base frame.

        Gap(float d, float w, Eigen::Matrix<float, 3, 1> p) : distance(d), width(w), startPoint(p)
        {}
    };

    struct Stair {
        float height;
        float width; 
        float length=1.0; 
        Eigen::Matrix<float, 3, 1> startPoint; // the closest point on the gap margin in base frame.
        int k=3;
        Stair() {}
        Stair(float h, float w, float l, Eigen::Matrix<float, 3, 1> p)
            : height(h), width(w), length(l), startPoint(p)
        {}
    };

    struct Terrain {
        TerrainType terrainType;
        float footHoldOffset = 0.1f;
        std::vector<Gap*> gaps;
        // float gapWidth = 0.14f;
        Stair* stair;
        Eigen::MatrixXf costMap;
    };

    /**
     * @brief As descriped in MIT CHEETAH3 paper the 3D plane is z(x,y) = a0+ a1*x +a2*y
     * @param a  Vec3<float>, coefficients for ground surface plane, z= a0+a1*x+a2*y
     */
    class GroundSurfaceEstimator {

    public:
        GroundSurfaceEstimator(Robot *robot, std::string terrainConfigPath, unsigned int windowSize=DEFAULT_WINDOW_SIZE);
    
        void Loadterrain(std::string& terrainConfigPath);
    
        void Reset(float currentTime);

        /**
         * @brief compute the plane equation when four feet are all in contact with ground.
         */
        void Update(float currentTime);

        /** @brief compute or return the normal of ground surface represent in (initial) world frame. */
        Eigen::Matrix<double, 3, 1> GetNormalVector(bool update);
        

        /**
         * @brief control frame is the frame origin at COM, and x axis is algined with body COM frame's X-axis,
         * as well as its z-axis is normal to estimated ground surface and y-axis is parrell
         * with the local surface plane.
         * @return the result is presented in the world frame.
         */
        Eigen::Matrix<double, 4, 4> ComputeControlFrame();

        float GetZ(float x, float y);
        
        /** @brief three direction vectors present in world frame when compute the GRF */
        Eigen::Matrix<float, 3, 3> GetAlignedDirections();

        Quat<float> GetControlFrameOrientation() const
        {
            return controlFrameOrientation.cast<float>();
        }
        
        Vec3<float> GetControlFrameRPY() const
        {
            return controlFrameRPY.cast<float>();
        }

        Vec3<double> controlFrameRPY;
        Quat<double> controlFrameOrientation;
    // private:
        Robot *robot;
        Terrain terrain;
        Vec3<double> a; // coeffcient of plane equation
        Eigen::Matrix<double, 4, 3> W;
        Vec4<double> pZ; // z position
        Vec3<double> n; // normal vector of plane equation

        Mat4<double> controlFrame;
        Eigen::Matrix<bool, 4, 1> lastContactState;
        
        YAML::Node footStepperConfig;
    };
}
#endif // ASCEND_QUADRUPED_CPP_GROUND_ESTIMATOR_H