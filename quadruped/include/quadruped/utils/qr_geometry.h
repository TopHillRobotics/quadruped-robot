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

#ifndef QR_GEOMETRY_H
#define QR_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <type_traits>

#include "utils/qr_algebra.h"
#include "utils/qr_cpptypes.h"
#include "utils/qr_se3.h"


namespace robotics {

namespace math {

/** 
 * @brief Linear interpolate between two point in state space.
 * @example 
 *  robotics::math::Segment<float, Vec6<float>> segment;
 *  Vec6<float> s = Vec6<float>::Zero();
 *  Vec6<float> t;
 *  t<< 1, 1, 1, 1, 0, 0;
 *  segment.Reset(s,t);
 *  Vec6<float> midpoint = segment.GetPoint(0.9);
 *  cout << midpoint << endl;
 * 
 */
template <class t=float, class T=Vec6<t>>
class qrSegment {

public:

    /**
     * @brief Contructor of class qrSegment.
     */
    qrSegment() = default;

    qrSegment(T source_, T dest_) {
        source = source_;
        dest = dest_;
    }

    /**
     * @brief Destructor of class Segment.
     */
    ~qrSegment() = default;

    T GetPoint(float phase) {
        if (phase > 1.0) {
            phase = 1.0;
        } else if (phase < 0.0) {
            phase =0.0;
        }
        T point = phase*dest + (1.0-phase)*source;
        return point;
    }

    void Reset(T source_, T dest_) {
        source = source_;
        dest = dest_;
    }

private:

    T source;

    T dest;

};


/**
 * @ref https://math.stackexchange.com/questions/2217654/interpolation-in-so3-different-approaches
 * @ref https://math.stackexchange.com/questions/1832019/what-is-the-angular-velocity-in-an-inertial-frame-given-the-angular-velocity-in
 * 
 */
class qrCubicSplineInSO3 {

public:

    /**
     * @brief Contructor of class qrCubicSplineInSO3.
     */
    qrCubicSplineInSO3() = default;

    qrCubicSplineInSO3(Mat3<float> R1, Mat3<float>R2, float t0, float duration, Mat3<float> inertial);

    /**
     * @brief Destructor of class CubicSplineInSO3.
     */
    ~qrCubicSplineInSO3() = default;

    void GetPoint(float currentTime, Mat3<float>& outR, Vec3<float>& outwb);

private:

    float t0;

    float dt;

    float lastTime;

    Mat3<float> R1;

    Mat3<float> lastR;

    Mat3<float> R2;

    Mat3<float> dR1;
    
    Mat3<float> dR2;

    Mat3<float> M0;

    Mat3<float> M1;

    Mat3<float> M2;

    Mat3<float> M3;

    Mat3<float> W;

};

/**
 * @brief Spline class defines an abstract class for different spline interpolations
 */
class qrSpline {

public:

    /**
     * @brief record the spatial information, including the time dervative.
     */
    struct Point {

        /**
         * @brief Contructor of struct Point.
         */
        Point() : x(0.0), xd(0.0), xdd(0.0) {
        }

        Point(float p, float v = 0.0, float a = 0.0) : x(p), xd(v), xdd(a) {
        }

        void setZero() {
            x = 0.0;
            xd = 0.0;
            xdd = 0.0;
        }

        friend std::ostream &operator <<(std::ostream& cout, Point& p);

        float x;
        float xd;
        float xdd;
    };

    qrSpline::Point operator=(const qrSpline::Point &rhs) {
        qrSpline::Point out;
        out.x = rhs.x;
        out.xd = rhs.xd;
        out.xdd = rhs.xdd;
        return out;
    }

    /**
     * @brief Contructor of class Spline.
     */
    qrSpline() : initial_time_(0.), duration_(0.) {
    }

    /**
     * @brief Contructor of class Spline.
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start: Start point
     * @param end: End point
     */
    qrSpline(const float &initial_time,
           const float &duration,
           const Point &start,
           const Point &end);
    
    /**
     * @brief Contructor of class Spline.
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start_p: Start point
     * @param end_p: End point
     */
    qrSpline(const float &initial_time,
           const float &duration,
           float start_p,
           float end_p);

    /**
     * @brief Destructor of class Spline.
     */
    virtual ~qrSpline() = default;

    /**
     * @brief Sets the boundary of the spline
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start_p: Start point
     * @param end_p: End point
     */
    void setBoundary(const float &initial_time,
                     const float &duration,
                     const Point &start_p,
                     const Point &end_p);

    /**
     * @brief Sets the boundary of the spline
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start_p: Start point
     * @param end_p: End point
     */
    void setBoundary(const float &initial_time,
                     const float &duration,
                     const float &start_p,
                     const float &end_p);

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param p: Point value
     */
    virtual bool getPoint(const float &current_time,
                          Point &p) = 0;

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param p: Point value
     */
    virtual bool getPoint(const float &current_time,
                          float &p) = 0;

    bool isTimeElapsed(float &time);

protected:

    /** @brief Initial time of the spline */
    float initial_time_;

    /** @brief Duration of the spline */
    float duration_;

    /** Start point of the spline */
    Point start_;

    /** @brief End point of the spline */
    Point end_;

};

inline bool qrSpline::isTimeElapsed(float &t) {
    /* This makes sense only without the time interval */
    if ((t - initial_time_) > duration_)
        return true;
    else
        return false;
}

class qrQuadraticSpline : public qrSpline {

public:

    /**
     * @brief Contructor of class qrQuadraticSpline.
     */
    qrQuadraticSpline() = default;

    /**
     * @brief Contructor of class QuadraticSpline.
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start: Start point
     * @param end: End point
     */
    qrQuadraticSpline(const float &initial_time,
                    const float &duration,
                    const Point &start,
                    const Point &end) : qrSpline(initial_time, duration, start, end) {
    }

    /**
     * @brief Contructor of class QuadraticSpline.
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start: Start point
     * @param end: End point
     */
    qrQuadraticSpline(const float &initial_time,
                    const float &duration,
                    float start,
                    float end) : qrSpline(initial_time, duration, start, end) {
    }

    /**
     * @brief Destructor of class QuadraticSpline.
     */
    ~qrQuadraticSpline() = default;

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param mid: Mid point value
     * @param p: Point value
     */
    bool getPoint(const float &current_time,
                  float mid,
                  Point &p);

    virtual bool getPoint(const float &current_time, Point &p);

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param mid: Mid point value
     * @param p: Point value
     */
    bool getPoint(const float &current_time,
                  float mid,
                  float &p);

    virtual bool getPoint(const float &current_time, float &p);

};

/**
 * @brief CubicSpline class defines a cubic spline interpolation
 */
class qrCubicSpline : public qrSpline {

public:

    /**
     * @brief Contructor of class CubicSpline.
     */
    qrCubicSpline() = default;

    /**
     * @brief Contructor of class CubicSpline.
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start: Start point
     * @param end: End point
     */
    qrCubicSpline(const float &initial_time,
                const float &duration,
                const Point &start,
                const Point &end) : qrSpline(initial_time, duration, start, end) {
    }

    /**
     * @brief Contructor of class CubicSpline.
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start: Start point
     * @param end: End point
     */
    qrCubicSpline(const float &initial_time,
                const float &duration,
                float start,
                float end) : qrSpline(initial_time, duration, start, end) {
    }

    /**
     * @brief Destructor of class CubicSpline.
     */
    ~qrCubicSpline() = default;

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param p: Point value
     */
    bool getPoint(const float &current_time,
                  Point &p);

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param p: Point value
     */
    bool getPoint(const float &current_time,
                  float &p);

};

/**
 * @brief FifthOrderPolySpline class defines a 5-order spline interpolation
 */
class qrFifthOrderPolySpline : public qrSpline {

public:

    /**
     * @brief Contructor of class FifthOrderPolySpline.
     */
    qrFifthOrderPolySpline() = default;

    /**
     * @brief Contructor of class FifthOrderPolySpline.
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start: Start point
     * @param end: End point
     */
    qrFifthOrderPolySpline(const float &initial_time,
                         const float &duration,
                         const Point &start,
                         const Point &end) : qrSpline(initial_time, duration, start, end) {
    }

    /**
     * @brief Destructor of class FifthOrderPolySpline.
     */
    ~qrFifthOrderPolySpline() = default;

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param p: Point value
     */
    bool getPoint(const float &current_time,
                  Point &p);

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param p: Point value
     */
    bool getPoint(const float &current_time,
                  float &p);

};

/**
 * @brief LinearSpline class defines a linear spline interpolation.
 */
class qrLinearSpline : public qrSpline {

public:

    /**
     * @brief Contructor of class LinearSpline.
     */
    qrLinearSpline() = default;

    /**
     * @brief Contructor of class LinearSpline.
     * @param initial_time: Initial time
     * @param duration: Duration of the spline
     * @param start: Start point
     * @param end: End point
     */
    qrLinearSpline(const float &initial_time,
                 const float &duration,
                 const Point &start,
                 const Point &end) : qrSpline(initial_time, duration, start, end) {
    }

    /**
     * @brief Destructor of class LinearSpline.
     */
    ~qrLinearSpline() = default;

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param p: Point value
     */
    bool getPoint(const float &current_time, Point &p);

    /**
     * @brief Gets the value of the point according to the spline interpolation.
     * @param current_time: Current time
     * @param p: Point value
     */
    bool getPoint(const float &current_time, float &p);

};

/**
 * @brief Linear interpolation between y0 and yf.  x is between 0 and 1.
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x) {
    static_assert(std::is_floating_point<x_t>::value,
                    "must use floating point value");
    assert(x >= 0 && x <= 1);
    return y0 + (yf - y0) * x;
}

/**
 * @brief Cubic bezier interpolation between y0 and yf.  x is between 0 and 1.
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x) {
    static_assert(std::is_floating_point<x_t>::value,
                    "must use floating point value");
    assert(x >= 0 && x <= 1);
    y_t yDiff = yf - y0;
    x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
    return y0 + bezier * yDiff;
}

/**
 * @brief Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1.
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
    static_assert(std::is_floating_point<x_t>::value,
                    "must use floating point value");
    assert(x >= 0 && x <= 1);
    y_t yDiff = yf - y0;
    x_t bezier = x_t(6) * x * (x_t(1) - x);
    return bezier * yDiff;
}

/**
 * @brief Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1.
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
    static_assert(std::is_floating_point<x_t>::value,
                    "must use floating point value");
    assert(x >= 0 && x <= 1);
    y_t yDiff = yf - y0;
    x_t bezier = x_t(6) - x_t(12) * x;
    return bezier * yDiff;
}

} // Namespace math

} // Namespace robotics

#endif // QR_GEOMETRY_H
