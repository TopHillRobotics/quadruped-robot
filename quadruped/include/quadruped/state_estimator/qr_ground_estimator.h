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

#include "common/qr_se3.h"
#include "robots/qr_robot.h"


struct qrGap {
    /**
     * @brief The distance between COM and center of the gap.
     */
    float distance;

    /**
     * @brief The width of the gap.
     */
    float width;

    /**
     * @brief The closest point on the gap margin in base frame.
     */
    Eigen::Matrix<float, 3, 1> startPoint;

    /**
    * @brief Constructor of qrGap.
    * @param d The distance between COM and center of the gap.
    * @param w The width of the gap.
    * @param p The closest point on the gap margin in base frame.
    */
    qrGap(float d, float w, Eigen::Matrix<float, 3, 1> p) : distance(d), width(w), startPoint(p)
    {}
};

struct qrStair {
    /**
     * @brief The height of one stair.
     */
    float height;

    /**
     * @brief The width of one stair.
     */
    float width; 

    /**
     * @brief The length of one stair.
     */
    float length=1.0; 

    /**
     * @brief The closest point on the stairs margin in base frame.
     */
    Eigen::Matrix<float, 3, 1> startPoint;

    /**
     * @brief 
     */
    int k=3;

    /**
    * @brief The default constructor of qrStair.
    */
    qrStair() {}

    /**
    * @brief Constructor of qrStair.
    * @param h The height of one stair.
    * @param w The width of one stair.
    * @param l The length of one stair.
    * @param p The closest point on the stairs margin in base frame.
    */
    qrStair(float h, float w, float l, Eigen::Matrix<float, 3, 1> p)
        : height(h), width(w), length(l), startPoint(p)
    {}
};

struct qrTerrain {
    /**
    * @brief The terrain type.
    */
    TerrainType terrainType;

    /**
    * @brief The foot hold offset.
    */
    float footHoldOffset = 0.1f;

    /**
    * @brief The gap terrain.
    */
    std::vector<qrGap*> gaps;

    /**
    * @brief The stair terrain.
    */
    qrStair* stair;

    /**
    * @brief The costMap detect by robot.
    */
    Eigen::MatrixXf costMap;
};

/**
 * @brief As descriped in MIT CHEETAH3 paper the 3D plane is z(x,y) = a0+ a1*x +a2*y
 * @param a  Vec3<float>, coefficients for ground surface plane, z= a0+a1*x+a2*y
 */
class qrGroundSurfaceEstimator {

public:

    /**
    * @brief Constructor of qrGroundSurfaceEstimator.
    * @param robot The robot object pointer.
    * @param terrainConfigPath The terrain yaml file path.
    * @param windowSize The size of window filter.
    */
    qrGroundSurfaceEstimator(qrRobot *robot, std::string terrainConfigPath, unsigned int windowSize=DEFAULT_WINDOW_SIZE);

    /**
    * @brief Load terrain from terrain yaml file.
    * @param terrainConfigPath The terrain yaml file path.
    */
    void Loadterrain(std::string& terrainConfigPath);

    /**
    * @brief Reset the estimator.
    * @param currentTime Current time.
    */
    void Reset(float currentTime);

    /**
     * @brief Compute the plane equation when four feet are all in contact with ground.
     */
    void Update(float currentTime);

    /** 
     * @brief Compute or return the normal of ground surface represent in (initial) world frame. 
     * @param update If need to update.
     */
    Eigen::Matrix<double, 3, 1> GetNormalVector(bool update);
    

    /**
     * @brief Control frame is the frame origin at COM, and x axis is algined with body COM frame's X-axis,
     * as well as its z-axis is normal to estimated ground surface and y-axis is parrell
     * with the local surface plane.
     * @return the result is presented in the world frame.
     */
    Eigen::Matrix<double, 4, 4> ComputeControlFrame();

    /** 
     * @brief Get the distance of Z axis from point (x,y) 
     * @param x The x position.
     * @param y The y position.
     */
    float GetZ(float x, float y);
    
    /** 
     * @brief Get three direction vectors present in world frame when compute the GRF
     * @return Three direction vectors present in world frame
     */
    Eigen::Matrix<float, 3, 3> GetAlignedDirections();

    /** 
     * @brief Get the orientation in control frame.
     * @return The orientation in control frame
     */
    Quat<float> GetControlFrameOrientation() const
    {
        return controlFrameOrientation.cast<float>();
    }
    
    /** 
     * @brief Get rpy in control frame.
     * @return The rpy in control frame
     */
    Vec3<float> GetControlFrameRPY() const
    {
        return controlFrameRPY.cast<float>();
    }

    /**
     * @brief The rpy in control frame
     */
    Vec3<double> controlFrameRPY;
    
    /**
     * @brief The orientation in control frame
     */
    Quat<double> controlFrameOrientation;

// private:
    /**
     * @brief The robot object pointer.
     */
    qrRobot *robot;
    
    /**
     * @brief The qrTerrain object.
     */
    qrTerrain terrain;
    
    /**
     * @brief The coeffcient of plane equation
     */
    Vec3<double> a;
    
    /**
     * @brief Compute the plane equation when four feet are all in contact with ground.
     */
    Eigen::Matrix<double, 4, 3> W;
    
    /**
     * @brief The z position.
     */
    Vec4<double> pZ;
    
    /**
     * @brief The normal vector of plane equation
     */
    Vec3<double> n;

    /**
     * @brief The control frame.
     */
    Mat4<double> controlFrame;

    /**
     * @brief The last legs contact state.
     */
    Eigen::Matrix<bool, 4, 1> lastContactState;
    
    /**
     * @brief The configuration in terrain yaml file.
     */
    YAML::Node footStepperConfig;

private:

    /**
     * @brief whether the plane should update
     * @param contactState: contact state of foot
     * @return whether to update
     */
    bool ShouldUpdate(const Eigen::Matrix<bool, 4, 1>& contactState);
};

#endif // QR_GROUND_ESTIMATOR_H
