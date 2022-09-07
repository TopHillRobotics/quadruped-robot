// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Normal University
// Contact:  tophill.robotics@gmail.com

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

#include "tinynurbs/tinynurbs.h"
#include "common/qr_geometry.h"
#include "common/qr_BSpline.h"
#include "config.h"

struct qrStepParameters {
    /**
     * @brief Default constructor of qrStepParameters.
     */
    qrStepParameters() : duration(0.), height(0.), penetration(0.)
    {}
    
    /**
     * @brief Constructor of qrStepParameters with parameters.
     * @param _duration Duration of the step
     * @param _height Height of the step
     * @param _penetration Distance of penetration of the swing trajectory
     */
    qrStepParameters(float _duration, float _height, float _penetration = 0.)
        : duration(_duration), height(_height), penetration(_penetration)
    {}

    /** 
     * @brief Duration of the step 
     */
    float duration;

    /** 
     * @brief Height of the step 
     */
    float height;

    /** 
     * @brief Distance of penetration of the swing trajectory 
     */
    float penetration;
};

/**
 * @brief Descripte the spline infomation  
 */
struct qrSplineInfo {
    /**
     * @brief Default constructor of qrSplineInfo.
     */
    qrSplineInfo () {}

    /**
     * @brief Constructor of qrSplineInfo with parameters.
     * @param controlPointsIn 
     * @param knosIn 
     */
    qrSplineInfo (std::vector<glm::vec3> &controlPointsIn, std::vector<float> &knotsIn) {
        splineType = "BSpline";
        controlPoints = controlPointsIn;
        knots = knotsIn;
    }

    /**
     * @brief int, the degree of spline, such as 2, 3, 5
     */
    int degree=3;

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
                        const qrStepParameters &params);

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
    float initial_time_;

    /** 
     * @brief Duration of the swing trajectory 
     */
    float duration_;

    /** 
     * @brief The start position of the swing trajectory
     */
    Vec3<float> startPos;

    /** 
     * @brief  The end position of the swing trajectory
     */
    Vec3<float> endPos;

private:
    /** 
     * @brief Spliners for the different axis of the foot movement 
     */
    math::CubicSpline foot_spliner_x_;
    math::CubicSpline foot_spliner_y_;
    math::CubicSpline foot_spliner_up_z_;
    math::CubicSpline foot_spliner_down_z_;
};


class qrFootBSplinePatternGenerator : public qrFootSplinePatternGenerator {
public:
    /** 
     * @brief Constructor function 
     * @param splineInfo: structure of spline information
     */
    qrFootBSplinePatternGenerator(qrSplineInfo &qrSplineInfo);

    /** 
     * @brief Destructor function 
     */
    virtual ~qrFootBSplinePatternGenerator() = default;

    /** 
     * @brief Set parameters of spline.
     * @param initial_time The initial time.
     * @param initial_pos The initial position of the foot.
     * @param target_pos The target position of the foot
     * @param params qrStepParameters object.
     */
    virtual void SetParameters(const float initial_time,
                            const Eigen::Vector3f &initial_pos,
                            const Eigen::Vector3f &target_pos,
                            const qrStepParameters &params);

    /** 
     * @brief Generate trajectory via spline.
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
     * @brief Update the information of the spline.
     * @param initial_time The initial time.
     * @param duration Duration of the swing trajectory.
     * @param initial_pos The initial position of the foot.
     * @param target_appex
     * @param target_pos The target position of the foot.
     */
    virtual void UpdateSpline(float initial_time, float duration, const Eigen::Vector3f &initial_pos, float target_appex,const Eigen::Vector3f &target_pos);

private:
    /** 
     * @brief curve information
     */
    tinynurbs::Curve3f crv;
};

class qrSwingFootTrajectory {
public:
    /**
     * @brief Default constructor of qrSwingFootTrajectory.
     */
    qrSwingFootTrajectory() {};

    /** 
     * @brief Constructor function.
     * @param splineInfoIn spline information.
     * @param startPosIn foot start position.
     * @param endPosIn foot end position.
     * @param duration Duration of the swing trajectory. default value=1.f
     * @param maxClearance Max clearance of the trajectory. default value = 0.1f
     */
    qrSwingFootTrajectory(qrSplineInfo qrSplineInfoIn,
                        Vec3<float> startPosIn = {0.f, 0.f, 0.f},
                        Vec3<float> endPosIn = {0.f, 0.f, 0.f},
                        float duration = 1.f,
                        float maxClearance = 0.1f
                        );

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
     * @brief A new cycle begining of the swing foot. 
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

    // private:
    /** 
     * @brief Mid position between the startPos and the endPos.
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

#endif  // QR_FOOT_TRAJECTORY_GENERATOR_H
