// The MIT License

// Copyright (c) 2022 
// Robot Motion and Vision Laboratory at East China Parabola University
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

#include "planner/qr_foot_trajectory_generator.h"

qrFootSplinePatternGenerator::qrFootSplinePatternGenerator() : initial_time(0.f), duartion(0.f)
{

}

void qrFootSplinePatternGenerator::SetParameters(const float initial_time,
                                                 const Eigen::Vector3f &initial_pos,
                                                 const Eigen::Vector3f &target_pos,
                                                 const StepParameters &params)
{
    // Setting the initial time and duration of the swing movements
    this->initial_time = initial_time;
    this->duration = params.duration;

    // Computing the appex of the swing movement
    Eigen::Vector3f step_delta = target_pos - initial_pos;
    float height_dist = fabs((float)step_delta(2));
    float step2d_dist = fabs(step_delta.head<2>().norm());
    float step_theta;
    if (step2d_dist < 1e-3) {
        printf("[warning] no xy-plane movement, set foot hight default value!\n");
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
    // Setting the spline boundaries
    this->foot_spliner_x_.setBoundary(initial_time,
                                      params.duration,
                                      initial_pos(0),
                                      target_pos(0));
    this->foot_spliner_y_.setBoundary(initial_time,
                                      params.duration,
                                      initial_pos(1),
                                      target_pos(1));
    this->foot_spliner_up_z_.setBoundary(initial_time,
                                          params.duration / 2.0f,
                                          initial_pos(2),
                                          target_appex);
    this->foot_spliner_down_z_.setBoundary(initial_time + params.duration / 2.0f,
                                           params.duration / 2.0f,
                                           target_appex,
                                           target_pos(2) - params.penetration);
}

bool qrFootSplinePatternGenerator::GenerateTrajectory(Vec3<float> &foot_pos,
                                                      Vec3<float> &foot_vel,
                                                      Vec3<float> &foot_acc,
                                                      float time)
{
    if (time < initial_time_ - 1e-3)  // the float number is not exactly representation, so add 1e-3.
        return false; // duration it's always positive, and makes sense when
    // is bigger than the sample time
    // Computing the time that allows us to discriminate the swing-up or swing-down phase
    robotics::math::qrSpline::qrPoint swing_traj_x, swing_traj_y, swing_traj_z;
    float dt = time - initial_time_;
    foot_spliner_x_.getPoint(time, swing_traj_x);
    foot_spliner_y_.getPoint(time, swing_traj_y);

    if (dt <= (duration_ / 2.0f))
        foot_spliner_up_z_.getPoint(time, swing_traj_z);
    else
        foot_spliner_down_z_.getPoint(time, swing_traj_z);

    // Setting the foot state
    foot_pos << swing_traj_x.x, swing_traj_y.x, swing_traj_z.x;
    foot_vel << swing_traj_x.xd, swing_traj_y.xd, swing_traj_z.xd;
    foot_acc << swing_traj_x.xdd, swing_traj_y.xdd, swing_traj_z.xdd;

    if (time >= initial_time_ + duration_ + 1e-3)
        return false;

    return true;
}


qrFootBSplinePatternGenerator::qrFootBSplinePatternGenerator()
{
    // points in regular frame
    crv.control_points = {glm::vec3(-10, 0, 0),
                          glm::vec3(-11, 0, 0.2),
                          glm::vec3(-14, 0, 2),
                          glm::vec3(-15, 0, 7),
                          glm::vec3(0, 0, 8),
                          glm::vec3(15, 0, 7),
                          glm::vec3(14, 0, 2),
                          glm::vec3(11, 0, 0.2),
                          glm::vec3(10, 0, 0)};
    crv.knots = {0., 0., 0., 0.,
                1./6, 2./6, 3./6, 4./6, 5./6,
                1, 1, 1, 1};
    crv.degree = 3;
    glm::vec3 pt1 = tinynurbs::curvePoint(crv, 0.f);
}

void qrFootBSplinePatternGenerator::SetParameters(const float initial_time,
                                                  const Eigen::Vector3f &initial_pos,
                                                  const Eigen::Vector3f &target_pos,
                                                  const qrStepParameters &params)
{
    // Setting the initial time and duration of the swing movements
    this->initial_time = initial_time;
    this->duration = params.duration;

    // Computing the appex of the swing movement
    Eigen::Vector3f step_delta = target_pos - initial_pos;
    float height_dist = fabs((float)step_delta(2));
    float step2d_dist = fabs(step_delta.head<2>().norm());
    float step_theta;
    if (step2d_dist < 1e-3) {
        printf("[warning] no xy-plane movement, set foot hight default value!\n");
        step_theta = 0.f;
    } else {
        step_theta = atan(height_dist / step2d_dist);
    }
    float target_appex = params.height;

    // Setting the spline boundaries
    UpdateSpline(initial_time, params.duration, initial_pos, target_appex, target_pos);
}

void qrFootBSplinePatternGenerator::UpdateSpline(float initial_time, 
                                                 float duration, 
                                                 const Eigen::Vector3f &initial_pos, 
                                                 float target_appex, 
                                                 const Eigen::Vector3f &target_pos)
{
    std::vector<glm::vec3> &controlPoints =  crv.control_points;
    std::vector<float> &knots = crv.knots;
    target_appex *= 100.f;
    this->startPos = initial_pos * 100.f;
    this->endPos = target_pos * 100.f;
    float xRatio = abs(this->endPos[0] - this->startPos[0]) / 20.f;
    if (this->endPos[2] >= this->startPos[2]) { // walk up
        float z_length_left = target_appex;// - endPos[2]; // > 0
        float z_length_right = target_appex - (this->endPos[2] - this->startPos[2]);
        float zRatio = abs(z_length_left) / 8.f;
        float x_mid = (this->endPos[0] + this->startPos[0]) / 2;
        float z_offset = this->startPos[2];
        for(auto &point : controlPoints) {
            point.x = point.x * xRatio + x_mid;
            // point.y = point.y * yRatio;
            point.z = point.z * zRatio + z_offset;

        }

        controlPoints[8].z = this->endPos[2];
        controlPoints[7].z = controlPoints[8].z + 0.2 / 8 *z_length_right;
        controlPoints[6].z = controlPoints[8].z + 2.0 / 8 *z_length_right;
        controlPoints[5].z = controlPoints[8].z + 7.0 / 8 *z_length_right;
    
    } else { // walk down
        float z_length_left = target_appex - (this->startPos[2] - this->endPos[2]);
        float z_length_right = target_appex ;
        float zRatio = abs(z_length_right) / 8.f;
        float x_mid = (this->endPos[0] + this->startPos[0]) / 2;
        float z_offset = this->endPos[2];
        for(auto &point : controlPoints) {
            point.x = point.x * xRatio + x_mid;
            // point.y = point.y * yRatio;
            point.z = point.z * zRatio + z_offset;

        }
        controlPoints[0].z = this->startPos[2];
        controlPoints[1].z = controlPoints[0].z + 0.2 / 8 *z_length_left;
        controlPoints[2].z = controlPoints[0].z + 2.0 / 8 *z_length_left;
        controlPoints[3].z = controlPoints[0].z + 7.0 / 8 *z_length_left; 
    }
}

bool qrFootBSplinePatternGenerator::GenerateTrajectory(Vec3<float> &foot_pos,
                                                       Vec3<float> &foot_vel,
                                                       Vec3<float> &foot_acc,
                                                       float time)
{
    float dt = time - initial_time_;
    if (dt < - 1e-3 || dt >= duration_ + 1e-3){  // the float number is not exactly representation, so add 1e-3.
        return false; // duration it's always positive, and makes sense when
    }
    // Setting the foot state
    glm::vec3 pt1 = tinynurbs::curvePoint(crv, dt);
    foot_pos[0] = pt1.x/100; // cm --> m
    foot_pos[1] = pt1.y/100;
    foot_pos[2] = pt1.z/100;
    // from regular frame -> world frame
    foot_pos[1] = (1.f - dt) * startPos(1, 0)/100 + dt * endPos(1, 0)/100;
    return true;
}

float qrFootParabolaSplinePatternGenerator::GenParabola(float phase, float start, float mid, float end)
{
    float a;
    float b;
    float c;
   
    a = 2*(y0 - 2*ym + y1);
    b = -3*y0 + 4*ym - y1;
    c = y0;
    
    return a * x * x + b * x + c;
}

Eigen::Matrix<float, 3, 1> qrFootParabolaSplinePatternGenerator::GenerateSwingFootTrajectory(float phase,
                                                                                             Eigen::Matrix<float, 3, 1> startPos,
                                                                                             Eigen::Matrix<float, 3, 1> endPos,
                                                                                             float clearance=0.1)
{
    // refer to google's motion_imitation code (Python)
    // For the first half of the swing cycle, the swing leg moves faster and finishes 
    // 80% of the full swing trajectory. The rest 20% of trajectory takes another half swing cycle. 
    // Intuitely, we want to move the swing foot quickly to the target landing location and 
    // stay above the ground. In this way the control is more robust to perturbations to the body
    // that may cause the swing foot to drop onto the ground earlier than expected.
    // This is a common practice similar to the MIT cheetah and Marc Raibert's original controllers.
    float phase;
    float x;
    float y;
    float z;
    float mid;
    float clearance;

    phase = inputPhase;

    if (inputPhase <= 0.5) {
        phase = 0.8 * sin(inputPhase * M_PI);
    } else {
        phase = 0.8 + (inputPhase - 0.5) * 0.4;
    }
    
    clearance = 0.1;
    
    x = (1 - phase) * startPos(0, 0) + phase * endPos(0, 0);
    y = (1 - phase) * startPos(1, 0) + phase * endPos(1, 0);
    mid = max(endPos(2, 0), startPos(2, 0)) + maxClearance;
    z = GenerateParabola(phase, startPos(2, 0), mid, endPos(2, 0));

    return Eigen::Matrix<float, 3, 1>(x, y, z);
}

qrSwingFootTrajectory::qrSwingFootTrajectory(qrSplineInfo splineInfoIn,
                                             Vec3<float> startPosIn,
                                             Vec3<float> endPosIn,
                                             float duration,
                                             float maxClearance)
    : splineInfo(splineInfoIn), startPos(startPosIn), endPos(endPosIn), stepParams(duration, 0., 0.)
{
    this->mid = std::max(this->endPos[2], this->startPos[2]) + maxClearance;
    if (this->splineInfo.splineType=="BSpline") {
        std::cout << "swing BSpline\n";
        this->stepParams.height = std::min(0.2f, std::max(0.1f, maxClearance + abs(this->endPos[2] - this->startPos[2])));
        this->footTarjGen = new qrFootBSplinePatternGenerator(splineInfo);
    } else if(this->splineInfo.splineType=="cubicPolygon") {
        std::cout << "swing CubicPolygon\n";
        // stepParams.height = maxClearance;  // todo
        this->stepParams = qrStepParameters(duration, mid, 0.);
        this->footTarjGen = new qrFootSplinePatternGenerator();
    } 
    // TODO : add Parabola spline generator. - gk
    // else if(this->splineInfo.splineType=="Parabola") {
    //     std::cout << "swing Parabola\n";
    //     this->footTarjGen = new qrFootNoramalSplinePatternGenerator();
    // }
    this->footTarjGen->SetParameters(0., this->startPos, this->endPos, this->stepParams);
}

qrSwingFootTrajectory::qrSwingFootTrajectory(const qrSwingFootTrajectory &item)
{
    mid = item.mid;
    this->stepParams = item.stepParams;
    footTarjGen->SetParameters(0., item.startPos, item.endPos, this->stepParams);
}

bool qrSwingFootTrajectory::GenerateTrajectoryPoint(Vec3<float> &footPos,
                                                    Vec3<float> &footV,
                                                    Vec3<float> &footA,
                                                    float t,
                                                    bool phaseModule)
{
    float inputPhase = t;  // 0<=t<=1
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
    // TODO : add Parabola spline process - gk.
    // if(this->splineInfo.splineType == "Parabola"){

    // }
    flag = this->footTarjGen->GenerateTrajectory(footPos, footV, footA, phase);
    return flag; // return # p,v,a
}