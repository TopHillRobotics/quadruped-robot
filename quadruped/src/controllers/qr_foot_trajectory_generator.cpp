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

#include "controllers/qr_foot_trajectory_generator.h"


namespace Quadruped {

qrFootBSplinePatternGenerator::qrFootBSplinePatternGenerator(qrSplineInfo &splineInfo)
{
    controlPointsTemplate = {
        glm::vec3(-10  , 0, 0),
        glm::vec3(-10.3, 0, 0.2),
        glm::vec3(-13  , 0, 2),
        glm::vec3(-15  , 0, 7),
        glm::vec3(0    , 0, 7.8),
        glm::vec3(11   , 0, 8),
        glm::vec3(10.5 , 0, 4),
        glm::vec3(10.2 , 0, 1),
        glm::vec3(10   , 0, 0)};

    crv.control_points = controlPointsTemplate;

    crv.knots = {0.,    0.,    0.,    0.,
                 0.3/6, 1.3/6, 2.5/6, 3.0/6, 4.0/6,
                 1,     1,     1,     1};
    crv.degree = 3;
    glm::vec3 pt1 = tinynurbs::curvePoint(crv, 0.f);
}


void qrFootBSplinePatternGenerator::SetParameters(
    const float initial_time,
    const Eigen::Vector3f &initial_pos,
    const Eigen::Vector3f &target_pos,
    const qrStepParameters &params)
{
    /* Setting the initial time and duration of the swing movements. */
    initialTime = initial_time;
    this->duration = params.duration;

    /* Computing the appex of the swing movement. */
    Eigen::Vector3f step_delta = target_pos - initial_pos;
    float dx = step_delta[0];
    float dy = step_delta[1];

    RTheta = robotics::math::coordinateRotation(robotics::math::CoordinateAxis::Z, atan2(dy, dx));

    Tp = initial_pos;

    float height_dist = fabs((float)step_delta(2));
    float step2d_dist = fabs(step_delta.head<2>().norm());

    float step_theta;
    if (step2d_dist < 1e-3) {
        step_theta = 0.f;
    } else {
        step_theta = atan(height_dist / step2d_dist);
    }
    float target_appex = params.height;

    /* Setting the spline boundaries. */
    UpdateSpline(initial_time, params.duration, initial_pos, target_appex, step_delta);
}


void qrFootBSplinePatternGenerator::UpdateSpline(
    float initial_time,
    float duration,
    const Eigen::Vector3f &initial_pos,
    float target_appex,
    const Eigen::Vector3f &target_pos)
{
    std::vector<glm::vec3> &controlPoints =  crv.control_points;
    std::vector<float> &knots = crv.knots;
    target_appex *= 100.f;
    startPos << 0.f, 0.f, 0.f;
    endPos = RTheta* (target_pos * 100.f);
    float xRatio = abs(endPos[0] - startPos[0]) / 20.f;
    if (endPos[2] >= startPos[2]) {
        /* Walk up. */
        float z_length_left = target_appex;
        float z_length_right = target_appex - (endPos[2] - startPos[2]);
        float zRatio = abs(z_length_left) / 8.f;
        float x_mid = (endPos[0] + startPos[0])/2;
        float z_offset = startPos[2];
        for(int i=0; i<controlPointsTemplate.size();++i) {
            controlPoints[i].x = controlPointsTemplate[i].x * xRatio + x_mid;
            controlPoints[i].z = controlPointsTemplate[i].z * zRatio + z_offset;

        }

        controlPoints[8].z = endPos[2];
        controlPoints[7].z = controlPoints[8].z + controlPointsTemplate[7].z/8 * z_length_right;
        controlPoints[6].z = controlPoints[8].z + controlPointsTemplate[6].z/8 * z_length_right;
        controlPoints[5].z = controlPoints[8].z + controlPointsTemplate[5].z/8 * z_length_right;

    } else {
        /* Walk down. */
        float z_length_left = target_appex - (startPos[2] - endPos[2]); // - endPos[2]
        float z_length_right = target_appex;
        float zRatio = abs(z_length_right) / 8.f;
        float x_mid = (endPos[0] + startPos[0])/2;
        float z_offset = endPos[2];
        for(int i=0; i<controlPointsTemplate.size();++i) {
            controlPoints[i].x = controlPointsTemplate[i].x * xRatio + x_mid;
            controlPoints[i].z = controlPointsTemplate[i].z * zRatio + z_offset;
        }
        controlPoints[0].z = startPos[2];
        controlPoints[1].z = controlPoints[0].z + 0.2 /8 *z_length_left;
        controlPoints[2].z = controlPoints[0].z + 2.0/8 *z_length_left;
        controlPoints[3].z = controlPoints[0].z + 7.0/8 *z_length_left;
    }
}


bool qrFootBSplinePatternGenerator::GenerateTrajectory(
    Vec3<float> &foot_pos,
    Vec3<float> &foot_vel,
    Vec3<float> &foot_acc,
    float time)
{
    float dt = time - initialTime;

    /* The float number is not exactly representation, so add 1e-3. */
    if (dt < - 1e-3 || dt >= duration + 1e-3)
        return false;

    auto pv = tinynurbs::curveDerivatives(crv, 1, dt);

    /* convert cm to m. */
    foot_pos[0] = pv[0].x / 100;
    foot_pos[1] = pv[0].y / 100;
    foot_pos[2] = pv[0].z / 100;
    foot_vel[0] = pv[1].x / 100;
    foot_vel[1] = pv[1].y / 100;
    foot_vel[2] = pv[1].z / 100;

    foot_pos = RTheta.transpose() * foot_pos + Tp;
    foot_vel = RTheta.transpose() * foot_vel;
    return true;
}


void qrFootParabolaPatternGenerator::SetParameters(
    const float initial_time,
    const Eigen::Vector3f &initial_pos,
    const Eigen::Vector3f &target_pos,
    const qrStepParameters &params)
{
    /* Setting the initial time and duration of the swing movements. */
    initialTime = initial_time;
    this->duration = params.duration;
    stepParameters = params;
    startPos = initial_pos;
    endPos = target_pos;

    /* Setting the spline boundaries. */
    footSplinerZ.setBoundary(initial_time,
                                params.duration,
                                initial_pos(2),
                                target_pos(2));
}


bool qrFootParabolaPatternGenerator::GenerateTrajectory(
    Vec3<float> &foot_pos,
    Vec3<float> &foot_vel,
    Vec3<float> &foot_acc,
    float phase)
{
    if (phase < initialTime - 1e-3)
        return false;

    if (phase >= initialTime + duration + 1e-3)
        return false;

    robotics::math::qrSpline::Point swing_traj_x, swing_traj_y, swing_traj_z;

    float maxClearance, mid;
    swing_traj_x.x = (1 - phase) * startPos[0] + phase * endPos[0];
    swing_traj_y.x = (1 - phase) * startPos[1] + phase * endPos[1];
    maxClearance = stepParameters.height;

    mid = std::max(endPos(2, 0), startPos(2, 0)) + maxClearance;

    footSplinerZ.getPoint(phase, mid, swing_traj_z);

    /* Setting the foot state. */
    foot_pos << swing_traj_x.x, swing_traj_y.x, swing_traj_z.x;
    foot_vel << swing_traj_x.xd, swing_traj_y.xd, swing_traj_z.xd;
    foot_acc << swing_traj_x.xdd, swing_traj_y.xdd, swing_traj_z.xdd;
    return true;
}


void qrFootCubicPatternGenerator::SetParameters(
    const float initial_time,
    const Eigen::Vector3f &initial_pos,
    const Eigen::Vector3f &target_pos,
    const qrStepParameters &params)
{
    /* Setting the initial time and duration of the swing movements. */
    initialTime = initial_time;
    this->duration = params.duration;

    /* Computing the appex of the swing movement. */
    Eigen::Vector3f step_delta = target_pos - initial_pos;
    float height_dist = fabs((float)step_delta(2));
    float step2d_dist = fabs(step_delta.head<2>().norm());
    float step_theta;
    if (step2d_dist < 1e-3) {
        step_theta = 0.f;
    } else {
        step_theta = atan(height_dist / step2d_dist);
    }
    float target_appex;
    if (target_pos(2) >= initial_pos(2)) {
        target_appex = target_pos(2) + params.height * cos(step_theta);
    } else {
        target_appex = initial_pos(2) + params.height * cos(step_theta);
    }
    /* Setting the spline boundaries. */
    footSplinerX.setBoundary(initial_time, params.duration, initial_pos(0), target_pos(0));
    footSplinerY.setBoundary(initial_time, params.duration, initial_pos(1), target_pos(1));
    footSplinerUpZ.setBoundary(initial_time, params.duration / 2.0f, initial_pos(2), target_appex);
    footSplinerDownZ.setBoundary(initial_time + params.duration / 2.0f, params.duration / 2.0f, target_appex, target_pos(2) - params.penetration);
}


bool qrFootCubicPatternGenerator::GenerateTrajectory(
    Vec3<float> &foot_pos,
    Vec3<float> &foot_vel,
    Vec3<float> &foot_acc,
    float time)
{
    if (time < initialTime - 1e-3)
        return false;
    robotics::math::qrSpline::Point swing_traj_x, swing_traj_y, swing_traj_z;
    float dt = time - initialTime;
    footSplinerX.getPoint(time, swing_traj_x);
    footSplinerY.getPoint(time, swing_traj_y);

    if (dt <= (duration / 2.0f))
        footSplinerUpZ.getPoint(time, swing_traj_z);
    else
        footSplinerDownZ.getPoint(time, swing_traj_z);

    foot_pos << swing_traj_x.x, swing_traj_y.x, swing_traj_z.x;
    foot_vel << swing_traj_x.xd, swing_traj_y.xd, swing_traj_z.xd;
    foot_acc << swing_traj_x.xdd, swing_traj_y.xdd, swing_traj_z.xdd;

    if (time >= initialTime + duration + 1e-3)
        return false;

    return true;
}


SwingFootTrajectory::SwingFootTrajectory(
    qrSplineInfo splineInfoIn,
    Vec3<float> startPosIn,
    Vec3<float> endPosIn,
    float duration,
    float maxClearance):

    splineInfo(splineInfoIn),
    startPos(startPosIn),
    endPos(endPosIn),
    stepParams(duration, 0., 0.)
{
    mid = std::max(endPos[2], startPos[2]) + maxClearance;

    switch (splineInfo.splineType)
    {
    case SplineType::BSpline:
        stepParams.height = std::min(0.2f, std::max(0.1f, maxClearance + abs(endPos[2] - startPos[2])));
        footTarjGen = new qrFootBSplinePatternGenerator(splineInfo);
        break;
    case SplineType::CubicPolygon:
        stepParams = qrStepParameters(duration, mid, 0.);
        footTarjGen = new qrFootCubicPatternGenerator();
        break;
    default:
        stepParams.height = maxClearance;
        footTarjGen = new qrFootParabolaPatternGenerator();
        break;
    }

    footTarjGen->SetParameters(0., startPos, endPos, stepParams);
}


SwingFootTrajectory::SwingFootTrajectory(const SwingFootTrajectory &item)
{
    mid = item.mid;
    stepParams = item.stepParams;
    footTarjGen->SetParameters(0., item.startPos, item.endPos, stepParams);
}

bool SwingFootTrajectory::GenerateTrajectoryPoint(
    Vec3<float> &footPos,
    Vec3<float> &footV,
    Vec3<float> &footA,
    float t,
    bool phaseModule)
{
    float inputPhase = t;
    float phase;
    if (phaseModule) {
        if (inputPhase <= 0.5) {
            phase = 0.8 * std::sin(inputPhase * M_PI);
        } else {
            phase = 0.8 + (inputPhase - 0.5) * 0.4;
        }
    } else {
        phase = inputPhase;
    }
    bool flag;
    flag = footTarjGen->GenerateTrajectory(footPos, footV, footA, phase);
    return flag;
}

} // Namespace Quadruped
