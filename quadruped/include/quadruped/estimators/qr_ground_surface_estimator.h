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

#include "utils/qr_se3.h"
#include "robots/qr_robot.h"
#include "estimators/qr_base_state_estimator.h"


namespace Quadruped {

/**
 * @brief One kind of terrian:   ground----| gap |----ground
 */
struct qrGap {

    /**
     * @brief Distance between COM and center of the gap.
     */
    float distance;
    /**
     * @brief Width of the gap.
     */
    float width;

    /**
     * @brief The closest point on the gap margin in base frame.
     */
    Eigen::Matrix<float, 3, 1> startPoint;

    /**
     * @brief Constructor of gap struct.
     * @param d: distance between COM and center of the gap
     * @param w: width of the gap
     * @param p: the closest point on the gap margin in base frame.
     */
    inline qrGap(float d, float w, Eigen::Matrix<float, 3, 1> p) : distance(d), width(w), startPoint(p)
    { }

};

/**
 * @brief One kind of terrian.
 */
struct qrStair {

    /**
     * @brief Height of a step.
     */
    float height;

    /**
     * @brief Width of a step.
     */
    float width; 

    /**
     * @brief Length of a step.
     */
    float length=1.0; 

    /**
     * @brief The closest point on the gap margin in base frame.
     */
    Eigen::Matrix<float, 3, 1> startPoint; // the closest point on the gap margin in base frame.

    /**
     * @brief Num of the steps.
     */
    int k=3;

    /**
     * @brief Constructor of struct Stair.
     */
    inline qrStair() = default;

    /**
     * @brief constructor of stair.
     * @param h: height of a step.
     * @param w: width of a step.
     * @param l: length of a step.
     * @param p: the closest point on the gap margin in base frame.
     */
    inline qrStair(float h, float w, float l, Eigen::Matrix<float, 3, 1> p) : height(h), width(w), length(l), startPoint(p) {
    };

};

/**
 * @brief Struct of terrian.
 */
struct qrTerrain {

    /**
     * @brief Type of terrian.
     */
    TerrainType terrainType;

    /**
     * @brief The foothold contact offset to the foot joint.
     */
    float footHoldOffset = 0.1f;

    /**
     * @brief Vector of gaps.
     */
    std::vector<qrGap*> gaps;

    /**
     * @brief Point to stair
     */
    qrStair* stair;

    /**
     * @brief Matrix of cost map for foothold planner.
     */
    Eigen::MatrixXf costMap;

};

/**
 * @brief As descriped in MIT CHEETAH3 paper the 3D plane is z(x,y) = a0+ a1*x +a2*y
 * @param a  Vec3<float>, coefficients for ground surface plane, z= a0+a1*x+a2*y
 */
class qrGroundSurfaceEstimator : public qrBaseStateEstimator {

public:

    /**
     * @brief Constructor of ground surface estimator.
     * @param Robot: the robot class for ground estimation.
     * @param terrainConfigPath: the file path to the terrian config.
     */
    qrGroundSurfaceEstimator(qrRobot *robot, std::string terrainConfigPath);

    /**
     * @brief Load the terrain config file.
     * @param terrainConfigPath: the file path to the terrian config.
     */
    void Loadterrain(std::string& terrainConfigPath);

    /**
     * @brief Reset the estimator.
     * @param currentTime: time since the timer started.
     */
    virtual void Reset(float currentTime);

    /**
     * @brief Compute the plane equation when four feet are all in contact with ground.
     * @param currentTime: time since the timer started.
     */
    virtual void Update(float currentTime);

    /**
     * @brief Compute or return the normal of ground surface represent in (initial) base frame.
     * @param update: if normalize the normal vector or not.
     * @return the normal of ground surface represent in (initial) base frame.
    */
    Eigen::Matrix<double, 3, 1> GetNormalVector(bool update);

    /**
     * @brief Control frame is the frame origin at COM, and x axis is algined with body COM frame's X-axis,
     * as well as its z-axis is normal to estimated ground surface and y-axis is parrell
     * with the local surface plane.
     * @return homogenous transformation Matrix of control frame w.r.t world frame.
     */
    Eigen::Matrix<double, 4, 4> ComputeControlFrame();

    /**
     * @brief Get z value at a point of ground surface with respect to base frame.
     * @param x: the given x in base frame.
     * @param y: the given y in base frame.
     * @return the z value.
     */
    float GetZInBaseFrame(float x, float y);

    /**
     * @brief Get a z value in current control frame by given x and y in last control frame.
     * @param x: the given x in last control frame.
     * @param y: the given y in last control frame.
     * @return the z value in current control frame.
     */
    float GetZInControlFrame(float x, float y);

    /**
     * @brief Get three direction vectors present in world frame when compute the GRF.
     * @return the rotation matrix of control frame with respect to world frame.
    */
    Eigen::Matrix<float, 3, 3> GetAlignedDirections();

    /**
     * @brief Get orientation quaterion present in world frame.
     * @return orientation quaternion of control frame with respect to world frame.
    */
    Quat<float> GetControlFrameOrientation() const {
        return controlFrameOrientation.cast<float>();
    };
    
    /**
     * @brief Get row-pitch-yaw of control frame present in world frame.
     * @return row-pitch-yaw of control frame present in world frame.
    */
    Vec3<float> GetControlFrameRPY() const {
        return controlFrameRPY.cast<float>();
    };

    /**
     * @brief Row-pitch-yaw of control frame present in world frame.
    */
    Vec3<double> controlFrameRPY;

    /**
     * @brief Orientation quaternion of control frame with respect to world frame.
    */
    Quat<double> controlFrameOrientation;

    /**
     * @brief The robot class for ground estimation.
    */
    qrRobot *robot;

    /**
     * @brief Terrian for locomotion.
    */
    qrTerrain terrain;

    /**
     * @brief Coeffcient of plane equation.
    */
    Vec3<double> a;

    /**
     * @brief W contains the position of four footholds,
     * which used to caculate the coeffcient of plane equation.
     * W = [1, p_x, p_y]_4*3, p contains data for each leg, p_x = [p_x1, p_x2, p_x3, p_x4]
    */
    Eigen::Matrix<double, 4, 3> W;

    /**
     * @brief Z position of 4 footholds in base frame.
    */
    Vec4<double> pZ;

    /**
     * @brief Normal vector of plane equation in base frame.
    */
    Vec3<double> n;

    /**
     * @brief Robot body position in world frame.
    */
    Vec3<float> bodyPositionInWorldFrame;

    /**
     * @brief Homogenous transformation Matrix of control frame w.r.t world frame.
    */
    Mat4<double> controlFrame;

    /**
     * @brief Contact state of for legs at last control loop.
    */
    Eigen::Matrix<bool, 4, 1> lastContactState;
    
    /**
     * @brief Yaml node of the foothold planner config file.
    */
    YAML::Node footStepperConfig;

};

} // Namespace Quadruped

#endif // QR_GROUND_ESTIMATOR_H
