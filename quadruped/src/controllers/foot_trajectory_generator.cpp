/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: seing foot tarjectory generator.
* Author: Zhu Yijie
* Create: 2021-11-20
* Notes: this is for swing foot.
* Modify: init the file. @ Zhu Yijie;
*/

#include "controllers/foot_trajectory_generator.h"

namespace Quadruped {

    FootBSplinePatternGenerator::FootBSplinePatternGenerator(SplineInfo &splineInfo)
    {
        // points in regular frame
        // controlPointsTemplate = {glm::vec3(-10, 0, 0),
        //                         glm::vec3(-11, 0, 0.2),
        //                         glm::vec3(-14, 0, 2),
        //                         glm::vec3(-15, 0, 7),
        //                         glm::vec3(0, 0, 8),
        //                         glm::vec3(13, 0, 7),
        //                         glm::vec3(12, 0, 2),
        //                         glm::vec3(11, 0, 0.2),
        //                         glm::vec3(10, 0, 0)
        //                         }; // old
        controlPointsTemplate = {glm::vec3(-10, 0, 0),
                                glm::vec3(-10.3, 0, 0.2),
                                glm::vec3(-13, 0, 2),
                                glm::vec3(-15, 0, 7),
                                glm::vec3(0, 0, 7.8),
                                glm::vec3(11, 0, 8),
                                glm::vec3(10.5, 0, 4),
                                glm::vec3(10.2, 0, 1),
                                glm::vec3(10, 0, 0)
                                };
        // controlPointsTemplate = {glm::vec3(-10, 0, 0),
        //                         glm::vec3(-10.3, 0, 0.2),
        //                         glm::vec3(-13, 0, 2),
        //                         glm::vec3(-15, 0, 7),
        //                         glm::vec3(0, 0, 7.8),
        //                         glm::vec3(10.5, 0, 8),
        //                         glm::vec3(10.2, 0, 4),
        //                         glm::vec3(10.1, 0, 1),
        //                         glm::vec3(10, 0, 0)
        //                         };

        crv.control_points = controlPointsTemplate;
        crv.knots = {0., 0., 0., 0.,
                    // 1./6, 2./6, 3./6, 4./6, 5./6, // old
                    0.3/6, 1.3/6, 2.5/6, 3.0/6, 4.0/6, // this
                    // 0.5/6, 1.0/6, 2.0/6, 3.0/6, 4.5/6,
                    1, 1, 1, 1
                    };
        crv.degree = 3;
        glm::vec3 pt1 = tinynurbs::curvePoint(crv, 0.f);
    }
    
    void FootBSplinePatternGenerator::SetParameters(const float initial_time,
                                                   const Eigen::Vector3f &initial_pos,
                                                   const Eigen::Vector3f &target_pos,
                                                   const StepParameters &params)
    {
        // Setting the initial time and duration of the swing movements
        initial_time_ = initial_time;
        duration_ = params.duration;

        // Computing the appex of the swing movement
        Eigen::Vector3f step_delta = target_pos - initial_pos;
        float dx = step_delta[0];
        float dy = step_delta[1];
        
        R_theta = robotics::math::coordinateRotation(robotics::math::CoordinateAxis::Z, atan2(dy, dx));

        Tp = initial_pos;

        float height_dist = fabs((float)step_delta(2));
        float step2d_dist = fabs(step_delta.head<2>().norm());
        
        float step_theta;
        if (step2d_dist < 1e-3) {
            // printf("[warning] no xy-plane movement, set foot hight default value!\n");
            step_theta = 0.f;
        } else {
            step_theta = atan(height_dist / step2d_dist);
        }
        float target_appex = params.height;

        // Setting the spline boundaries
        UpdateSpline(initial_time, params.duration, initial_pos, target_appex, step_delta);  // target_pos
    }

    void FootBSplinePatternGenerator::UpdateSpline(float initial_time, float duration, const Eigen::Vector3f &initial_pos, float target_appex, const Eigen::Vector3f &target_pos)
    {
        std::vector<glm::vec3> &controlPoints =  crv.control_points;  
        std::vector<float> &knots = crv.knots;
        target_appex *= 100.f;
        startPos << 0.f, 0.f, 0.f; //  initial_pos * 100.f;
        endPos = R_theta* (target_pos * 100.f); // w' frame --> canonical frame
        float xRatio = abs(endPos[0] - startPos[0]) / 20.f;
        // std::cout << "start pos=" <<startPos.transpose() << std::endl;
        // std::cout << "end pos=" <<endPos.transpose() << std::endl;
        // std::cout << "xRatio= "<< xRatio << std::endl;
        if (endPos[2] >= startPos[2]) { // walk up
            float z_length_left = target_appex;// - endPos[2]; // > 0
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
        
        } else { // walk down
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
        
        // int i=0;
        // for (auto &point : controlPoints) {
        //      std::cout << "controlPoint ["<< i<< "] = "<< point.x <<", "<< point.y <<", "<< point.z << std::endl;
        //     i+=1;
        // }
    }

    bool FootBSplinePatternGenerator::GenerateTrajectory(Vec3<float> &foot_pos,
                                                        Vec3<float> &foot_vel,
                                                        Vec3<float> &foot_acc,
                                                        float time)
    { 
        float dt = time - initial_time_;
        
        if (dt < - 1e-3 || dt >= duration_ + 1e-3)  // the float number is not exactly representation, so add 1e-3.
            return false; // duration it's always positive, and makes sense when
        
        // Setting the foot state
        // foot_pos << swing_traj_x.x, swing_traj_y.x, swing_traj_z.x;
        // foot_vel << swing_traj_x.xd, swing_traj_y.xd, swing_traj_z.xd;
        // foot_acc << swing_traj_x.xdd, swing_traj_y.xdd, swing_traj_z.xdd;
        // glm::vec3 pt1 = tinynurbs::curvePoint(crv, dt);
        auto pv = tinynurbs::curveDerivatives(crv, 1, dt);
        // glm::vec2 tgt1 = tinynurbs::curveTangent(crv, 0.5f);
        // REQUIRE(tgt1.x == Approx(1));
        // REQUIRE(tgt1.y == Approx(0));
        foot_pos[0] = pv[0].x / 100; // cm --> m
        foot_pos[1] = pv[0].y / 100;
        foot_pos[2] = pv[0].z / 100;
        foot_vel[0] = pv[1].x / 100;
        foot_vel[1] = pv[1].y / 100;
        foot_vel[2] = pv[1].z / 100;
        
        // CASE 1. linear interpolation for Y-axis
        // foot_pos[1] = (1.f - dt) * startPos(1, 0)/100 + dt * endPos(1, 0)/100;
        
        // CASE 2. from regular frame -> canonical frame
        foot_pos = R_theta.transpose() * foot_pos + Tp;
        foot_vel = R_theta.transpose() * foot_vel;
        return true;
    }

    FootParabolaPatternGenerator::FootParabolaPatternGenerator()
    {}

    FootParabolaPatternGenerator::~FootParabolaPatternGenerator()
    {}

    void FootParabolaPatternGenerator::SetParameters(const float initial_time,
                                                   const Eigen::Vector3f &initial_pos,
                                                   const Eigen::Vector3f &target_pos,
                                                   const StepParameters &params)
    {
        // Setting the initial time and duration of the swing movements
        initial_time_ = initial_time;
        duration_ = params.duration;
        stepParameters = params;
        startPos = initial_pos;
        endPos = target_pos;
        // Setting the spline boundaries
        foot_spliner_z_.setBoundary(initial_time,
                                    params.duration,
                                    initial_pos(2),
                                    target_pos(2));
    }

    bool FootParabolaPatternGenerator::GenerateTrajectory(Vec3<float> &foot_pos,
                                                        Vec3<float> &foot_vel,
                                                        Vec3<float> &foot_acc,
                                                        float phase)
    {
        if (phase < initial_time_ - 1e-3)  // the float number is not exactly representation, so add 1e-3.
            return false; // duration it's always positive, and makes sense when

        if (phase >= initial_time_ + duration_ + 1e-3)
            return false;
        // is bigger than the sample time
        // Computing the time that allows us to discriminate the swing-up or swing-down phase
        robotics::math::Spline::Point swing_traj_x, swing_traj_y, swing_traj_z;
        
        float maxClearance, mid;
        swing_traj_x.x = (1 - phase) * startPos[0] + phase * endPos[0];
        swing_traj_y.x = (1 - phase) * startPos[1] + phase * endPos[1];
        maxClearance = stepParameters.height;// 0.15; // todo
        
        mid = std::max(endPos(2, 0), startPos(2, 0)) + maxClearance;
        
        foot_spliner_z_.getPoint(phase, mid, swing_traj_z);
        
        // Setting the foot state
        foot_pos << swing_traj_x.x, swing_traj_y.x, swing_traj_z.x;
        foot_vel << swing_traj_x.xd, swing_traj_y.xd, swing_traj_z.xd;
        foot_acc << swing_traj_x.xdd, swing_traj_y.xdd, swing_traj_z.xdd;
        return true;
    }

    
    FootCubicPatternGenerator::FootCubicPatternGenerator()
    {}

    FootCubicPatternGenerator::~FootCubicPatternGenerator()
    {}

    void FootCubicPatternGenerator::SetParameters(const float initial_time,
                                                   const Eigen::Vector3f &initial_pos,
                                                   const Eigen::Vector3f &target_pos,
                                                   const StepParameters &params)
    {
        // Setting the initial time and duration of the swing movements
        initial_time_ = initial_time;
        duration_ = params.duration;

        // Computing the appex of the swing movement
        Eigen::Vector3f step_delta = target_pos - initial_pos;
        float height_dist = fabs((float)step_delta(2));
        float step2d_dist = fabs(step_delta.head<2>().norm());
        float step_theta;
        if (step2d_dist < 1e-3) {
            // printf("[warning] no xy-plane movement, set foot hight default value!\n");
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
        foot_spliner_x_.setBoundary(initial_time,
                                    params.duration,
                                    initial_pos(0),
                                    target_pos(0));
        foot_spliner_y_.setBoundary(initial_time,
                                    params.duration,
                                    initial_pos(1),
                                    target_pos(1));
        foot_spliner_up_z_.setBoundary(initial_time,
                                       params.duration / 2.0f,
                                       initial_pos(2),
                                       target_appex);
        foot_spliner_down_z_.setBoundary(initial_time + params.duration / 2.0f,
                                         params.duration / 2.0f,
                                         target_appex,
                                         target_pos(2) - params.penetration);
    }

    bool FootCubicPatternGenerator::GenerateTrajectory(Vec3<float> &foot_pos,
                                                        Vec3<float> &foot_vel,
                                                        Vec3<float> &foot_acc,
                                                        float time)
    {
        if (time < initial_time_ - 1e-3)  // the float number is not exactly representation, so add 1e-3.
            return false; // duration it's always positive, and makes sense when
        // is bigger than the sample time
        // Computing the time that allows us to discriminate the swing-up or swing-down phase
        robotics::math::Spline::Point swing_traj_x, swing_traj_y, swing_traj_z;
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

    SwingFootTrajectory::SwingFootTrajectory(SplineInfo splineInfoIn,
                                            Vec3<float> startPosIn,
                                             Vec3<float> endPosIn,
                                             float duration,
                                             float maxClearance
                                             )
        : splineInfo(splineInfoIn), startPos(startPosIn), endPos(endPosIn), stepParams(duration, 0., 0.)
    {
        mid = std::max(endPos[2], startPos[2]) + maxClearance;
        
        switch (splineInfo.splineType)
        {
        case SplineType::BSpline:
            // std::cout << "swing BSpline\n";
            stepParams.height = std::min(0.2f, std::max(0.1f, maxClearance + abs(endPos[2] - startPos[2])));
            footTarjGen = new FootBSplinePatternGenerator(splineInfo);
            break;
        case SplineType::CubicPolygon:
            // std::cout << "swing CubicPolygon\n";
            // stepParams.height = maxClearance;  // todo
            stepParams = StepParameters(duration, mid, 0.);
            footTarjGen = new FootCubicPatternGenerator();
            break;
        default: // SplineType::XYLinear_ZParabola
            // std::cout << "swing XYLinear_ZParabola\n";
            stepParams.height = maxClearance; //std::min(0.2f, std::max(0.1f, maxClearance + abs(endPos[2] - startPos[2])));
            footTarjGen = new FootParabolaPatternGenerator();
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

    // SwingFootTrajectory::~SwingFootTrajectory()
    // {
        // delete  footTarjGen;
    // }

    bool SwingFootTrajectory::GenerateTrajectoryPoint(Vec3<float> &footPos,
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
        // std::cout << footPos << "\n" << footV << "\n"<< footA << std::endl;
        // std::cout << "swing gen 266\n";
        bool flag;
        flag = footTarjGen->GenerateTrajectory(footPos, footV, footA, phase);
        // if (endPos[2] < -0.05) {
        //     for (float i = 0.f; i<1.f; i=i+0.01) {
        //         flag = footTarjGen->GenerateTrajectory(footPos, footV, footA, i);
        //         datax.push_back(footPos[0]);
        //         // std::cout << "swing gen "<< footPos << std::endl;
        //         datay1.push_back(footPos[2]);
        //     }
        //     std::cout << "swing gen 273\n";
        //     plt::figure();
        //     std::map<std::string,std::string> map_;
        //     map_["cmap"] = "viridis";
        //     map_["alpha"] = "0.5";
        //     plt::scatter(datax, datay1, (2.0), map_);
        //     plt::grid(true);
        //     plt::show();
        //     exit(0);
        // }

        // if (phaseModule) {
        //     if (phase<=0.5) {
        //         footV *= 1.2;
        //     } else {
        //     footV *= 0.8;
        //     }
        // }
        return flag; // return p,v,a;
    }

} // Quadruped
