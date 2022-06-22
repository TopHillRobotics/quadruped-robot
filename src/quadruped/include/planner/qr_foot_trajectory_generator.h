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

#ifndef QR_FOOT_TRAJECTORY_GENERATOR_H
#define QR_FOOT_TRAJECTORY_GENERATOR_H

#include <Eigen/Dense>

#include "common/geometry.h"
#include "common/BSpline.h"
#include "config.h"

/**
 * @brief 
 */
struct qrStepParameters{

    /**
     * @brief Default constructor of qrStepParameters.
     */
    qrStepParameters():duration(0.f),height(0.f),penetration(0.f)
    {}

    /**
     * @brief Constructor of qrStepParameters with parameters.
     */
    qrStepParameters(float duration, float height, float penetration = 0.f)
        : duration(duration), height(height), penetration(penetration)
    {}

    /**
     * @brief Duration of the step.
     */
    float duration;

    /**
     * @brief Height of the step.
     */
    float height;

    /**
     * @brief Distance of penetration of the swing trajectory.
     */
    float penetration;
};

/**
 * @brief Spline information.
 */
struct qrSplineInfo {
    
    /**
     * @brief Default constructor of qrSplineInfo.
     */
    qrSplineInfo() {}

    /**
     * @brief Constructor of qrSplineInfo with parameters.
     * @param controlPointsIn 
     * @param knosIn 
     */
    qrSplineInfo(std::vector<glm::vec3> &controlPointsIn, std::vector<float> &knosIn) 
    {}

    /**
     * @brief 
     */
    int degree = 3;

    /**
     * @brief The type of spline.
     * e.g. "cubicPolygon", "BSpline","Normal"
     */
    std::string splineTye = "cubicPolygon";

    /**
     * @brief params of Bspline
     */
    std::vector<glm::vec3> controlPoints;

    /**
     * @brief 
     */
    std::vector<float> knots;
};

/**
 * @brief qrFootSplinePatternGenerator is a base class of foot spline generator
 */
class qrFootSplinePatternGenerator {
public:

    /** 
     * @brief Constructor function 
     */
    qrFootSplinePatternGenerator();

    /** 
     * @brief Destructor function 
     */
    virtual ~qrFootSplinePatternGenerator();

    /**
     * @brief Set the parameters for the generation of the foot swing trajectory
     * This methods assumes that there is not an obstacle in the trajectory.
     * @param initial_time Initial time.
     * @param initial_pos Initial foot position.
     * @param target_pos Target foot position.
     * @param params Step parameters.
     */
    virtual void SetParameters(const float initial_time,
                        const Eigen::Vector3f &initial_pos,
                        const Eigen::Vector3f &target_pos,
                        const StepParameters &params);

    /**
     * @brief Generates the foot-swing trajectory for a given time
     * @param foot_pos Instantaneous foot position.
     * @param foot_vel Instantaneous foot velocity.
     * @param foot_acc Instantaneous foot acceleration.
     * @param time Current time.
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                            Eigen::Vector3f &foot_vel,
                            Eigen::Vector3f &foot_acc,
                            float time);
    
    virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos)
    {
        //
    }

protected:
    /** 
     * @brief Initial time of the swing trajectory 
     */
    float initial_time;

    /** 
     * @brief Duration of the swing trajectory 
     */
    float duration;

    /** 
     * @brief Duration of the swing trajectory 
     */
    Vec3<float> startPos;

    /** 
     * @brief Duration of the swing trajectory 
     */
    Vec3<float> endPos;

private:
    /** 
     * @brief Spliners for the different axis of the foot movement 
     */
    robotics::math::CubicSpline foot_spliner_x_;
    robotics::math::CubicSpline foot_spliner_y_;
    robotics::math::CubicSpline foot_spliner_up_z_;
    robotics::math::CubicSpline foot_spliner_down_z_;
};

class qrFootBSplinePatternGenerator : public qrFootSplinePatternGenerator {
public:
    /** 
     * @brief Constructor function of qrFootBSplinePatternGenerator.
     */
    qrFootBSplinePatternGenerator();

    /** 
     * @brief Destructor function.
     */
    virtual ~qrFootBSplinePatternGenerator() = default;

    /** 
     * @brief Destructor function.
     * @param initial_time The initial time.
     * @param initial_pos The initial position of the foot.
     * @param target_pos The target position of the foot
     * @param params StepParameters object.
     */
    virtual void SetParameters(const float initial_time,
                               const Eigen::Vector3f &initial_pos,
                               const Eigen::Vector3f &target_pos,
                               const StepParameters &params);

    /** 
     * @brief Destructor function.
     * @param foot_pos The foot position.
     * @param foot_vel The foot velocity.
     * @param foot_acc The foot acceleration.
     * @param time Current time.
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                    Eigen::Vector3f &foot_vel,
                                    Eigen::Vector3f &foot_acc,
                                    float time);

    /** 
     * @brief Destructor function.
     * @param initial_time The initial time.
     * @param duration Duration of the swing trajectory.
     * @param initial_pos The initial position of the foot.
     * @param target_appex
     * @param target_pos The target position of the foot.
     */
    virtual void UpdateSpline(float initial_time, 
                              float duration, 
                              const Eigen::Vector3f &initial_pos, 
                              float target_appex,
                              const Eigen::Vector3f &target_pos);

private:
    /** 
     * @brief curve information
     */
    tinynurbs::Curve3f crv;
};

class qrFootNoramalSplinePatternGenerator : public qrFootSplinePatternGenerator {

public:

    /**
     * @brief Default qrFootNoramalSplinePatternGenerator constructor.
     */
    qrFootNoramalSplinePatternGenerator();

    virtual ~qrFootNoramalSplinePatternGenerator() = default;

    /**
     * @brief Quadratic interpolation function, used to generate polygon curve.
     * @param phase
     * @param start
     * @param mid
     * @param end
     * @return a float value with phase
     */
    float GenParabola(float phase, float start, float mid, float end);

    /**
     * @brief Generating the trajectory of the swing leg
     * @param inputPhase
     * @param startPos
     * @param endPos
     * @return foot position like (x,y,z)
     */
    Matrix<float, 3, 1> GenerateTrajectory(float inputPhase,
                                           Matrix<float, 3, 1> startPos,
                                           Matrix<float, 3, 1> endPos);
};

class qrSwingFootTrajectory {
public:
    /**
     * @brief Default constructor of qrSwingFootTrajectory.
     */
    qrSwingFootTrajectory() {};

    /** 
     * @brief init func
     * @param splineInfoIn
     * @param startPosIn
     * @param endPosIn
     * @param duration Duration of the swing trajectory. default value=1.f
     * @param maxClearance Max clearance of the trajectory. default value = 0.1f
     */
    qrSwingFootTrajectory(SplineInfo splineInfoIn,
                        Vec3<float> startPosIn = {0.f, 0.f, 0.f},
                        Vec3<float> endPosIn = {0.f, 0.f, 0.f},
                        float duration = 1.f,
                        float maxClearance = 0.1f);

    /**
     * @brief Copy constructor of qrSwingFootTrajectory.
     * @param item qrSwingFootTrajectory object.
     */
    qrSwingFootTrajectory(const qrSwingFootTrajectory &item);

    virtual ~qrSwingFootTrajectory() = default;

    /** 
     * @brief Call it every time you need a tarjectory point to control. 
     * @param footPos foot position;
     * @param footV foot linear velocity;
     * @param footA foot acceleration;
     * @param t time phase, default at range [0,1];
     * @param phaseModule whether phase needs to be moduled, default -<em> false </em>.
     * @return bool flag that indicates whether the process/result/input is correct.
     */
    bool GenerateTrajectoryPoint(Vec3<float> &footPos,
                                    Vec3<float> &footV,
                                    Vec3<float> &footA,
                                    float t,
                                    bool phaseModule = false);

    /** 
     * @brief a new cycle begining of the swing foot. 
     * @param duration Duration of the swing trajectory
     * @param initialPos The initial position of the foot.
     * @param targetPos The target position of the foot.
     */
    void ResetFootTrajectory(float duration, const Vec3<float> &initialPos, const Vec3<float> &targetPos)
    {
        stepParams.duration = duration;
        footTarjGen->SetParameters(0., initialPos, targetPos, stepParams);
    };

    /** @brief corrupted in the mid air, adjust the behaviar. 
     * //TODO
     * @param duration Duration of the swing trajectory
     * @param currentTime Current time.
     * @param targetPos The target position of the foot.
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

private:

    /** 
     * @brief Destructor function.
     */
    float mid;

    /** 
     * @brief Foot start position.
     */
    Vec3<float> startPos;

    /** 
     * @brief Foot end position.
     */
    Vec3<float> endPos;

    /** 
     * @brief qrStepParameters object pointer.
     */
    qrStepParameters stepParams;

    /** 
     * @brief qrFootSplinePatternGenerator object pointer.
     */
    qrFootSplinePatternGenerator *footTarjGen;

    /** 
     * @brief qrFootNoramalSplinePatternGenerator object pointer.
     */
    qrFootNoramalSplinePatternGenerator *footNormalTrajGen;

    /** 
     * @brief SplineInfo object pointer.
     */
    qrSplineInfo splineInfo;    
};
#endif // QR_FOOT_TRAJECTORY_GENERATOR_H