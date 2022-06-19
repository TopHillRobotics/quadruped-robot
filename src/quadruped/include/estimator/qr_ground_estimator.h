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

#ifndef QR_GROUND_ESTIMATOR_H
#define QR_GROUND_ESTIMATOR_H

#include "utils/se3.h"
#include "robots/robot.h"

/**
 * @brief Describe a gap in the map.
 */
struct Gap {
    /**
     * @brief The distance between COM and center of the gap.
     */
    float distance;
    /**
     * @brief The gap's width which is the rectangle's width.
     */
    float width;
    /**
     * @brief The cloest point on the gap margin in base frame.
     */
    Eigen::Matrix<float, 3, 1> startPoint;
    
    /**
     * @brief Construct a Gap object by using given distance d, width w and start point p.
     */
    Gap(float d, float w, Eigen::Matrix<float, 3, 1> p):distance(d),width(w),startPoint(p)
    {}
};

/**
 * @brief Descirbe a continuous stairs in the map.
 */
struct Stair {
    /**
     * @brief The height of a stair.
     */
    float height;
    /**
     * @brief The width of a stair.
     */
    float width;
    /**
     * @brief The distance between this edge and the next stair's edge.
     */
    float length = 1.0;
    /**
     * @brief The cloest point on the gap margin in base frame.
     */
    Eigen::Matrix<float, 3, 1> startPoint;
    /**
     * @brief 
     */
    int k = 3;
    
    /**
     * @brief Default constructor of Stair.
     */
    Stair() {}
    
    /**
     * @brief Constructor of Stair with parameters.
     * @param h The height of a stair.
     * @param width The width of a stair.
     * @param p The cloest point on the gap margin in base frame.
     */
    Stair(float h, float w, float l, Eigen::Matrix<float, 3, 1> p)
        :height(h),width(w),length(l),startPoint(p)
    {}
};

/**
 * @brief Describe terrain information of the map.
 */
struct Terrain {
    /**
     * @brief The type of the terrain.(Gaps or stair)
     */
    TerrainType terrainType;
    /**
     * @brief The offset of foothold each step.
     */
    float footHoldOffset = 0.1f;
    /**
     * @brief The gaps' information in the map.
     */
    std::vector<Gap*> gaps;
    /**
     * @brief The stair information in the map. (Default only one stair in the map)
     */
    Stair* stair;
    /**
     * @brief Costmap information.
     */
    Eigen::MatrixXf costMap;
};

/**
 * @brief The qrGroundSurfaceEstimator is to estimate the ground surface that the robot's walk
 *        on. It will help robot to adjust its body to keep balance.
 */
class qrGroundSurfaceEstimator {
public:
    /**
     * @brief Constructor of qrGroundSurfaceEstimator. 
     * @param robot The robot object pointer.
     * @param terrainConfigPath The terrain config file path.
     */
    qrGroundSurfaceEstimator(qrRobot* robot, std::string terrainConfigPath);

    /**
     * @brief Load terrain from a given terrain config file path.
     * @param terrainConfigPath terrain config file path.
     */
    void LoadTerrain(std::string& terrainConfigPath);

    /**
     * @brief The Reset function of qrGroundSurfaceEstimator.
     */
    void Reset();
    
    /**
     * @brief Compute the plane equation when four feet are all in contact with ground.
     */
    void Update();

    /**
     * @brief Compute or return the normal of ground surface represent in (initial) world frame.
     * @param update If need to update the normal vector. True is yes and False is no.
     * @return The normal vector of plane equation.
     */
    Eigen::Matrix<double, 3, 1> GetNormalVector(bool update);

    /**
     * @brief control frame is the frame origin at COM, and x axis is algined with body COM frame's X-axis,
     * as well as its z-axis is normal to estimated ground surface and y-axis is parrell
     * with the local surface plane.
     * @return the result is presented in the world frame.
     */
    Eigen::Matrix<double, 4, 4> ComputeControlFrame();

    /**
     * @brief Get the height of the map in point (x,y).
     * @return The height.
     */
    float GetZ(float x, float y);

    /**
     * @brief three direction vectors present in world frame when compute the GRF
     * @return A 3*3 matrix of three (x,y,z) direction vectors.
     */
    Eigen::Matrix<float, 3, 3> GetAlignedDirections();

    /**
     * @brief Get the orientation in control frame.
     * @return A 4 * 1 Quad vector which descirbes the orientation in control frame.
     */
    Quat<float> GetControlFrameOrientation()const
    {
        return controlFrameOrientation.cast<float>();
    }

    /**
     * @brief Get the roll, pitch and yaw in control frame.
     * @return A 3*1 vector which contains roll, pitch and yaw.
     */
    Vec3<float> GetControlFrameRPY() const
    {
        return controlFrameRPY.cast<float>();
    }

private:

    /**
     * @brief The qrRobot object pointer.
     */
    qrRobot *robot;
    /**
     * @brief The Terrain object.
     */
    Terrain terrain;
    /**
     * @brief coeffcient of plane equation
     */
    Vec3<double> a;
    /**
     * @brief 
     */
    Eigen::Matrix<double, 4, 3> W;
    /**
     * @brief The z(height) information of the foot.
     */
    Vec4<double> pZ;
    /**
     * @brief Normal vector of plane equation.
     */
    Vec3<double> n;
    /**
     * @brief Control Frame;
     */
    Mat4<double> controlFrame;
    /**
     * @brief The vector which contains roll, pitch and yaw.
     */
    Vec3<double> controlFrameRPY;
    /**
     * @brief Quad vector which descirbes the orientation in control frame.
     */
    Quad<double> controlFrameOrientation;
    /**
     * @brief Last Contact states of each leg. 0 represents not contact and 1 represents contact.
     */
    Eigen::Matrix<bool, 4, 1> lastContactStates;
    /**
     * @brief YAML node of terrain config.
     */
    YAML::Node terrainConfig;
};

#endif // QR_GROUND_ESTIMATOR_H