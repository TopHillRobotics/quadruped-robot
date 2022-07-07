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

#include "common/qr_geometry.h"
#include <tinynurbs/tinynurbs.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

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
     * e.g. "cubicPolynomial", "BSpline", "bizer","Parabola"
     */
    std::string splineType = "cubicPolygon";

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
    virtual ~qrFootSplinePatternGenerator(){}

    /**
     * @brief Set the parameters for the generation of the foot swing trajectory
     * This methods assumes that there is not an obstacle in the trajectory.
     * @param initial_time Initial time.
     * @param initial_pos Initial foot position.
     * @param target_pos Target foot position.
     * @param params Step parameters.
     */
    virtual void SetParameters(const float initial_time,
                        const Vec3<float> &initial_pos,
                        const Vec3<float> &target_pos,
                        const qrStepParameters &params);

    /**
     * @brief Generates the foot-swing trajectory for a given time
     * @param foot_pos Instantaneous foot position.
     * @param foot_vel Instantaneous foot velocity.
     * @param foot_acc Instantaneous foot acceleration.
     * @param time Current time.
     */
    virtual bool GenerateTrajectory(Vec3<float> &foot_pos,
                            Vec3<float> &foot_vel,
                            Vec3<float> &foot_acc,
                            float time);
    
    virtual void UpdateSpline(float initial_time, float duration, Vec3<float> &initial_pos, float target_appex, Vec3<float> &target_pos)
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
     * @brief  
     */
    Vec3<float> startPos;

    /** 
     * @brief  
     */
    Vec3<float> endPos;

private:
    /** 
     * @brief Spliners for the different axis of the foot movement 
     */
    math::qrCubicSpline foot_spliner_x_;
    math::qrCubicSpline foot_spliner_y_;
    math::qrCubicSpline foot_spliner_up_z_;
    math::qrCubicSpline foot_spliner_down_z_;
};

class qrFootBSplinePatternGenerator : public qrFootSplinePatternGenerator {
public:
    /** 
     * @brief Constructor function of qrFootBSplinePatternGenerator.
     */
    qrFootBSplinePatternGenerator();

    /**
     * @brief Constructor function of qrFootBSplinePatternGenerator.
     * @param splineInfo: structure of spline information
     */
    qrFootBSplinePatternGenerator(qrSplineInfo &splineInfo);

    /** 
     * @brief Destructor function.
     */
    virtual ~qrFootBSplinePatternGenerator() = default;

    /** 
     * @brief Destructor function.
     * @param initial_time The initial time.
     * @param initial_pos The initial position of the foot.
     * @param target_pos The target position of the foot
     * @param params qrStepParameters object.
     */
    virtual void SetParameters(const float initial_time,
                               const Vec3<float> &initial_pos,
                               const Vec3<float> &target_pos,
                               const qrStepParameters &params);

    /** 
     * @brief Destructor function.
     * @param foot_pos The foot position.
     * @param foot_vel The foot velocity.
     * @param foot_acc The foot acceleration.
     * @param time Current time.
     */
    virtual bool GenerateTrajectory(Vec3<float> &foot_pos,
                                    Vec3<float> &foot_vel,
                                    Vec3<float> &foot_acc,
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
                              const Vec3<float> &initial_pos, 
                              float target_appex,
                              const Vec3<float> &target_pos);

private:
    /** 
     * @brief curve information
     */
    tinynurbs::Curve3f crv;
};

// class qrFootParabolaSplinePatternGenerator : public qrFootSplinePatternGenerator {

// public:

//     /**
//      * @brief Default qrFootParabolaSplinePatternGenerator constructor.
//      */
//     qrFootParabolaSplinePatternGenerator();

//     virtual ~qrFootParabolaSplinePatternGenerator() = default;

//     /**
//      * @brief Generate a parabola curve using three given points: (0, y0), (0.5, ym) and (1, y1).
//      * y = ax^2 + bx + c
//      * @param x specifies the given x coordiate (phase) to compute the parabola value. x is normalized to [0,1].
//      * @param y0 specifies the y coordinate of the start point at t=0.
//      * @param ym specifies the y coordinate of the middle point at x=0.5. 
//      * @param y1 specifies the y coordinate of the end point at x=1.
//      * @return the y coordinate of the parabola curve for a given x (phase).
//      */
//     float GenerateParabola(float x, float y0, float ym, float y1);

//     /**
//      * @brief Generate the 3D trajectory of the swing leg
//      * @param phase specifies the given phase in [0, 1] to compute the trajectory.
//      * @param startPos specifies the foot's position at the beginning of swing cycle.
//      * @param endPos specifies the foot's desired position at the end of swing cycle.
//      * @param clearance specifies the height over the ground.
//      * @return the desired foot position (x,y,z) at the current swing phase. 
//      */
//     Vec3<float> GenerateSwingFootTrajectory(float phase,
//                                                            Vec3<float> startPos,
//                                                            Vec3<float> endPos,
//                                                            float clearance=0.1);
// };

class qrSwingFootTrajectory {
public:
    /**
     * @brief Default constructor of qrSwingFootTrajectory.
     */
    qrSwingFootTrajectory() {};

    /** 
     * @brief init func
     * @param splineInfoIn spline information.
     * @param startPosIn foot start position.
     * @param endPosIn foot end position.
     * @param duration Duration of the swing trajectory. default value=1.f
     * @param maxClearance Max clearance of the trajectory. default value = 0.1f
     */
    qrSwingFootTrajectory(qrSplineInfo splineInfoIn,
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
     * @brief SplineInfo object pointer.
     */
    qrSplineInfo splineInfo;    
};
#endif // QR_FOOT_TRAJECTORY_GENERATOR_H
