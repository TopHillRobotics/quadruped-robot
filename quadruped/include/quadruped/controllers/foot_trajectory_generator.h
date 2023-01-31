/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: seing foot tarjectory generator.
* Author: Zhu Yijie
* Create: 2021-11-20
* Notes: this is for swing foot.
* Modify: init the file. @ Zhu Yijie;
*/

#ifndef ASCEND_QUADRUPED_CPP_FOOT_TRAJECTORY_GENERATOR_H
#define ASCEND_QUADRUPED_CPP_FOOT_TRAJECTORY_GENERATOR_H

#include <Eigen/Dense>
#include "utils/geometry.h"
#include "utils/BSpline.h"
#include "utils/tools.h"
#include "config.h"

namespace Quadruped {
    struct StepParameters {
        StepParameters() : duration(0.), height(0.), penetration(0.)
        {}
        StepParameters(float _duration, float _height, float _penetration = 0.)
            : duration(_duration), height(_height), penetration(_penetration)
        {}

        /** @brief Duration of the step */
        float duration;

        /** @brief Height of the step */
        float height;

        /** @brief Distance of penetration of the swing trajectory */
        float penetration;
    };

    enum SplineType {
        XYLinear_ZParabola=0,
        QuadraticPolygon,
        CubicPolygon,
        QuinticPolygon,
        BSpline
        
    };

    /**
     * @brief Descripte the spline infomation  
     * @param degree int, the degree of spline, such as 2, 3, 5
     * @param splineType SplineType, option value: quadratic, cubicPolygon, quinticPolygon, BSpline
     */
    struct SplineInfo {
        int degree=3;
        SplineType splineType = SplineType::CubicPolygon;
        std::vector<glm::vec3> controlPoints;
        std::vector<float> knots;
        SplineInfo () {}
        SplineInfo (std::vector<glm::vec3> &controlPointsIn, std::vector<float> &knotsIn) {
            splineType = SplineType::BSpline;
            controlPoints = controlPointsIn;
            knots = knotsIn;
        }
    };

    class FootSplinePatternGenerator {
    public:
        /** @brief Constructor function */
        FootSplinePatternGenerator(): initial_time_(0.), duration_(0.) {};

        /** @brief Destructor function */
        virtual ~FootSplinePatternGenerator() = default;

        /**
         * @brief Set the parameters for the generation of the foot swing trajectory
         * This methods assumes that there is not an obstacle in the trajectory.
         * @param const double& Initial time
         * @param const Eigen::Vector3d& Initial foot position
         * @param const Eigen::Vector3d& Target foot position
         * @param const StepParameters Step parameters
         */
        virtual void SetParameters(const float initial_time,
                           const Eigen::Vector3f &initial_pos,
                           const Eigen::Vector3f &target_pos,
                           const StepParameters &params) = 0;

        /**
         * @brief Generates the foot-swing trajectory for a given time
         * @param Eigen::Vector3d& Instantaneous foot position
         * @param Eigen::Vector3d& Instantaneous foot velocity
         * @param Eigen::Vector3d& Instantaneous foot acceleration
         * @param const double& Current time
         */
        virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                Eigen::Vector3f &foot_vel,
                                Eigen::Vector3f &foot_acc,
                                float time) = 0;
        
        virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos)
        {
            //
        }

        /*
        bool check_stop_condition(const Eigen::Matrix3d & Jac,
                const Eigen::Vector3d& grforce_base,
                double force_th);
        bool isTimeElapsed(double& t);
        */

    protected:
        /** @brief Initial time of the swing trajectory */
        float initial_time_;
        /** @brief Duration of the swing trajectory */
        float duration_;
        Vec3<float> startPos;
        Vec3<float> endPos;
        Mat3<float> R_theta;
        Vec3<float> Tp;
        std::vector<float> datax, datay1, datay2, datay3;

    private:
        /** @brief Spliners for the different axis of the foot movement */
    };

    class FootParabolaPatternGenerator : public FootSplinePatternGenerator {
    public:
        /** @brief Constructor function */
        FootParabolaPatternGenerator();

        /** @brief Destructor function */
        virtual ~FootParabolaPatternGenerator();

        /**
         * @brief Set the parameters for the generation of the foot swing trajectory
         * This methods assumes that there is not an obstacle in the trajectory.
         * @param const double& Initial time
         * @param const Eigen::Vector3d& Initial foot position
         * @param const Eigen::Vector3d& Target foot position
         * @param const StepParameters Step parameters
         */
        virtual void SetParameters(const float initial_time,
                           const Eigen::Vector3f &initial_pos,
                           const Eigen::Vector3f &target_pos,
                           const StepParameters &params);

        /**
         * @brief Generates the foot-swing trajectory for a given time
         * @param Eigen::Vector3d& Instantaneous foot position
         * @param Eigen::Vector3d& Instantaneous foot velocity
         * @param Eigen::Vector3d& Instantaneous foot acceleration
         * @param const double& Current time
         */
        virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                Eigen::Vector3f &foot_vel,
                                Eigen::Vector3f &foot_acc,
                                float time);
        
        virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos)
        {
            //
        }

    private:
        /** @brief Spliners for the different axis of the foot movement */
        robotics::math::QuadraticSpline foot_spliner_z_;
        StepParameters stepParameters;
    };


    class FootCubicPatternGenerator : public FootSplinePatternGenerator {
    public:
        /** @brief Constructor function */
        FootCubicPatternGenerator();

        /** @brief Destructor function */
        virtual ~FootCubicPatternGenerator();

        /**
         * @brief Set the parameters for the generation of the foot swing trajectory
         * This methods assumes that there is not an obstacle in the trajectory.
         * @param const double& Initial time
         * @param const Eigen::Vector3d& Initial foot position
         * @param const Eigen::Vector3d& Target foot position
         * @param const StepParameters Step parameters
         */
        virtual void SetParameters(const float initial_time,
                           const Eigen::Vector3f &initial_pos,
                           const Eigen::Vector3f &target_pos,
                           const StepParameters &params);

        /**
         * @brief Generates the foot-swing trajectory for a given time
         * @param Eigen::Vector3d& Instantaneous foot position
         * @param Eigen::Vector3d& Instantaneous foot velocity
         * @param Eigen::Vector3d& Instantaneous foot acceleration
         * @param const double& Current time
         */
        virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                Eigen::Vector3f &foot_vel,
                                Eigen::Vector3f &foot_acc,
                                float time);
        
        virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos)
        {
            //
        }

    private:
        /** @brief Spliners for the different axis of the foot movement */
        robotics::math::CubicSpline foot_spliner_x_;
        robotics::math::CubicSpline foot_spliner_y_;
        robotics::math::CubicSpline foot_spliner_up_z_;
        robotics::math::CubicSpline foot_spliner_down_z_;
    };

        
    class FootBSplinePatternGenerator : public FootSplinePatternGenerator {
    public:
        /** @brief Constructor function */
        FootBSplinePatternGenerator(SplineInfo &splineInfo);

        /** @brief Destructor function */
        virtual ~FootBSplinePatternGenerator() = default;

        virtual void SetParameters(const float initial_time,
                            const Eigen::Vector3f &initial_pos,
                            const Eigen::Vector3f &target_pos,
                            const StepParameters &params);
        virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                        Eigen::Vector3f &foot_vel,
                                        Eigen::Vector3f &foot_acc,
                                        float time);
        virtual void UpdateSpline(float initial_time, float duration, const Eigen::Vector3f &initial_pos, float target_appex,const Eigen::Vector3f &target_pos);

    private:
        tinynurbs::Curve3f crv;
        std::vector<glm::vec3> controlPointsTemplate;
    };
    
    /**************************************************/
    /**************************************************/
    class SwingFootTrajectory {
    public:
        /** brief init func
         * @param Vec3<float> startPosIn
         * @param Vec3<float> endPosIn
         * @param float duration, default value=1.f
         * @param float maxClearance, default value = 0.1f
         */
        SwingFootTrajectory() {};

        SwingFootTrajectory(SplineInfo splineInfoIn,
                            Vec3<float> startPosIn = {0.f, 0.f, 0.f},
                            Vec3<float> endPosIn = {0.f, 0.f, 0.f},
                            float duration = 1.f,
                            float maxClearance = 0.1f
                            );

        SwingFootTrajectory(const SwingFootTrajectory &item);

        virtual ~SwingFootTrajectory() = default;

        /** @brief Call it every time you need a tarjectory point to control. 
         * @param Vec3<float>& foot position;
         * @param Vec3<float>& foot linear velocity;
         * @param Vec3<float>& foot acceleration;
         * @param float time phase, default at range [0,1];
         * @param bool whether phase needs to be moduled, default -<em> false </em>.
         * @return bool flag that indicates whether the process/result/input is correct.
         */
        bool GenerateTrajectoryPoint(Vec3<float> &footPos,
                                     Vec3<float> &footV,
                                     Vec3<float> &footA,
                                     float t,
                                     bool phaseModule=false);

        /** @brief a new cycle begining of the swing foot. */
        void ResetFootTrajectory(float duration, const Vec3<float> &initialPos, const Vec3<float> &targetPos, float height=0.15f)
        {
            stepParams.duration = duration;
            stepParams.height = height;
            footTarjGen->SetParameters(0., initialPos, targetPos, stepParams);
        };

        /** @brief corrupted in the mid air, adjust the behaviar. 
         * todo
         */
        void ResetFootTrajectory(float duration, float currentTime, const Vec3<float> &targetPos)
        {
            stepParams.duration = duration;
            footTarjGen->SetParameters(currentTime, startPos, targetPos, stepParams);
        };

        void Update()
        {
            // footTarjGen->UpdateSpline();
        };

        // private:
        float mid;
        Vec3<float> startPos;
        Vec3<float> endPos;
        StepParameters stepParams;
        FootSplinePatternGenerator *footTarjGen;
        SplineInfo splineInfo;
        std::vector<float> datax, datay1, datay2, datay3;

    };
} // Quadruped

#endif  // ASCEND_QUADRUPED_CPP_FOOT_TRAJECTORY_GENERATOR_H
