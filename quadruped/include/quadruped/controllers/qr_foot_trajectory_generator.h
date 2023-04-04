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

#include <glm/glm.hpp>
#include <tinynurbs/tinynurbs.h>
#include <Eigen/Dense>

#include "utils/qr_geometry.h"
#include "utils/qr_tools.h"
#include "config.h"


namespace Quadruped {

enum SplineType {
    XYLinear_ZParabola = 0,
    QuadraticPolygon,
    CubicPolygon,
    QuinticPolygon,
    BSpline
};


struct qrStepParameters {

    qrStepParameters() = default;

    qrStepParameters(float _duration, float _height, float _penetration = 0.):
        duration(_duration), height(_height), penetration(_penetration) {
    }

    /**
     * @brief Duration of the step.
     */
    float duration = 0.0f;

    /**
     * @brief Height of the step.
     */
    float height = 0.0f;

    /**
     * @brief Distance of penetration of the swing trajectory.
     */
    float penetration = 0.0f;
};

struct qrSplineInfo {

    qrSplineInfo () = default;

    /**
     * @brief Constructor of class qrSplineInfo.
     * @param control_points: control points of the splines.
     * @param knots: knots of the splines.
     */
    qrSplineInfo (std::vector<glm::vec3> &control_points, std::vector<float> &knots) {
        this->splineType = SplineType::BSpline;
        this->controlPoints = control_points;
        this->knots = knots;
    }

    /**
     * @brief The degree of spline, such as 2, 3, 5.
     */
    int degree = 3;

    /**
     * @brief The spline type. Option value: quadratic, cubicPolygon, quinticPolygon, BSpline.
     */
    SplineType splineType = SplineType::CubicPolygon;

    std::vector<glm::vec3> controlPoints;

    std::vector<float> knots;

};

class qrFootSplinePatternGenerator {

public:

    qrFootSplinePatternGenerator() = default;

    virtual ~qrFootSplinePatternGenerator() = default;

    /**
     * @brief Set the parameters for the generation of the foot swing trajectory.
     * This method assumes that there is not an obstacle in the trajectory.
     * This method is a pure virtual function. Any user-defined Spline should implement this function.
     * @param initial_time: the start time of the spline.
     * @param initial_pos: the initial position of the footstep.
     * @param target_pos: the desired footstep of this gait period.
     * @param params: other parameters of the spline, @see Quadruped::StepParameters.
     */
    virtual void SetParameters(const float initial_time,
                               const Eigen::Vector3f &initial_pos,
                               const Eigen::Vector3f &target_pos,
                               const qrStepParameters &params) = 0;

    /**
     * @brief Generates the foot-swing trajectory for a given time.
     * @param foot_pos: instantaneous foot position.
     * @param foot_vel: instantaneous foot velocity.
     * @param foot_acc: instantaneous foot acceleration.
     * @param time: current time.
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                    Eigen::Vector3f &foot_vel,
                                    Eigen::Vector3f &foot_acc,
                                    float time) = 0;

    /**
     * @brief Update the spline every control loop.
     * @param initial_time: initial time when update the spline.
     * @param duration: duration of the spline.
     * @param initial_pos: initial foothold position when update the spline.
     * @param target_appex
     * @param target_pos: target position of the end of spline.
     */
    virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos) {
    }

protected:

    /**
     * @brief Initial time of the swing trajectory.
     */
    float initialTime = 0.0f;

    /**
     * @brief Duration of the swing trajectory.
     */
    float duration = 0.0f;

    /**
     * @brief Start position of the spline.
     */
    Vec3<float> startPos;

    /**
     * @brief End position of the spline.
     */
    Vec3<float> endPos;

    /**
     * @brief Rotation matrix conver world frame into canonical frame.
     */
    Mat3<float> RTheta;

    /**
     * @brief Initial position of the spline.
     */
    Vec3<float> Tp;

};

class qrFootParabolaPatternGenerator : public qrFootSplinePatternGenerator {

public:

    qrFootParabolaPatternGenerator() = default;

    virtual ~qrFootParabolaPatternGenerator() = default;

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual void SetParameters(const float initial_time,
                               const Eigen::Vector3f &initial_pos,
                               const Eigen::Vector3f &target_pos,
                               const qrStepParameters &params);

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                    Eigen::Vector3f &foot_vel,
                                    Eigen::Vector3f &foot_acc,
                                    float time);

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos) {
    }

private:

    /**
     * @brief Spliners for the different axis of the foot movement
     */
    robotics::math::qrQuadraticSpline footSplinerZ;

    /**
     * @brief Step parameters for spline update.
     */
    qrStepParameters stepParameters;

};

class qrFootCubicPatternGenerator : public qrFootSplinePatternGenerator {

public:

    qrFootCubicPatternGenerator() = default;

    virtual ~qrFootCubicPatternGenerator() = default;

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual void SetParameters(const float initial_time,
                               const Eigen::Vector3f &initial_pos,
                               const Eigen::Vector3f &target_pos,
                               const qrStepParameters &params);

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                            Eigen::Vector3f &foot_vel,
                            Eigen::Vector3f &foot_acc,
                            float time);

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual void UpdateSpline(float initial_time, float duration, Eigen::Vector3f &initial_pos, float target_appex, Eigen::Vector3f &target_pos) {
    }

private:

    /**
     * @brief Spliners for x axis of the foot movement.
     */
    robotics::math::qrCubicSpline footSplinerX;

    /**
     * @brief Spliners for y axis of the foot movement.
     */
    robotics::math::qrCubicSpline footSplinerY;

    /**
     * @brief Spliners for z-up axis of the foot movement.
     */
    robotics::math::qrCubicSpline footSplinerUpZ;

    /**
     * @brief Spliners for z-down axis of the foot movement.
     */
    robotics::math::qrCubicSpline footSplinerDownZ;

};

class qrFootBSplinePatternGenerator : public qrFootSplinePatternGenerator {

public:

    /**
     * @brief Constructor method of class qrFootBSplinePatternGenerator.
     * @param splinInfo: BSpline information.
     */
    qrFootBSplinePatternGenerator(qrSplineInfo &splineInfo);

    virtual ~qrFootBSplinePatternGenerator() = default;

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual void SetParameters(const float initial_time,
                               const Eigen::Vector3f &initial_pos,
                               const Eigen::Vector3f &target_pos,
                               const qrStepParameters &params);

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual bool GenerateTrajectory(Eigen::Vector3f &foot_pos,
                                    Eigen::Vector3f &foot_vel,
                                    Eigen::Vector3f &foot_acc,
                                    float time);

    /**
     * @see Quadruped::FootSplinePatternGenerator
     */
    virtual void UpdateSpline(float initial_time,
                              float duration,
                              const Eigen::Vector3f &initial_pos,
                              float target_appex,
                              const Eigen::Vector3f &target_pos);

private:

    tinynurbs::Curve3f crv;

    std::vector<glm::vec3> controlPointsTemplate;

};


class SwingFootTrajectory {

public:

    SwingFootTrajectory() = default;

    /**
     * @brief Constructor method of class qrSwingFootTrajectory
     * @param splineInfoIn: BSplineInfo if needed.
     * @param startPosIn: start position of the footstep of the swingleg.
     * @param endPosIn: end position of the footstep of the swingleg.
     * @param duration: duration of the trajectory, generally the swing duration.
     * @param maxClearance: the highest point of the trajectory.
     */
    SwingFootTrajectory(qrSplineInfo splineInfoIn,
                          Vec3<float> startPosIn = {0.f, 0.f, 0.f},
                          Vec3<float> endPosIn = {0.f, 0.f, 0.f},
                          float duration = 1.f,
                          float maxClearance = 0.1f);

    SwingFootTrajectory(const SwingFootTrajectory &item);

    virtual ~SwingFootTrajectory() = default;

    /**
     * @brief Call it every time you need a tarjectory point to control.
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

    /**
     * @brief A new cycle begining of the swing foot.
     */
    void ResetFootTrajectory(float duration, const Vec3<float> &initialPos, const Vec3<float> &targetPos, float height=0.15f) {
        stepParams.duration = duration;
        stepParams.height = height;
        footTarjGen->SetParameters(0., initialPos, targetPos, stepParams);
    };

    /**
     * @brief Corrupted in the mid air, adjust the behaviar.
     */
    void ResetFootTrajectory(float duration, float currentTime, const Vec3<float> &targetPos) {
        stepParams.duration = duration;
        footTarjGen->SetParameters(currentTime, startPos, targetPos, stepParams);
    };

    float mid;

    Vec3<float> startPos;

    Vec3<float> endPos;

    qrStepParameters stepParams;

    qrFootSplinePatternGenerator *footTarjGen;

    qrSplineInfo splineInfo;

};

} // Quadruped

#endif  // QR_FOOT_TRAJECTORY_GENERATOR_H
