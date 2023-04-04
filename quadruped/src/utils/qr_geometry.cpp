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

#include <Eigen/SVD>

#include "utils/qr_geometry.h"

namespace robotics {

namespace math {

std::ostream &operator <<(std::ostream& cout, qrSpline::Point& p)
{
    cout << "x = "<< p.x << ", dx = "<< p.xd <<", ddx = "<< p.xdd;
    return cout;
}


qrCubicSplineInSO3::qrCubicSplineInSO3(Mat3<float> R1, Mat3<float>R2, float t0, float duration, Mat3<float> inertial):
    t0(t0),
    lastTime(t0),
    dt(duration),
    R1(R1),
    R2(R2),
    lastR(R1)
{
    auto dx = R2 - R1;
    dR1 = Mat3<float>::Zero();
    dR2 = Mat3<float>::Zero();
    auto dv = dR2 - dR1;
    M0 = R1;
    M1 = dR1;
    auto temp = (dx - dR1*duration) / pow(duration,2);
    M2 = 3*temp - dv / duration;
    M3 = (temp - M2) / duration;
    
    W = 0.5 * inertial.trace()*Eigen::Matrix<float,3,3>::Identity() - inertial;
}


void qrCubicSplineInSO3::GetPoint(float currentTime, Mat3<float>& outR, Vec3<float>& outwb)
{
    currentTime = currentTime + 1e-3;
    float t = currentTime - t0;
    if (t > dt) {
        t = dt;
    }
    float t2 = t*t;
    float t3 = t2*t;
    Mat3<float> Mt = M3*t3 + M2*t2 + M1*t + M0;
    Eigen::JacobiSVD<Mat3<float>> svd( (Mt*W), Eigen::ComputeFullV | Eigen::ComputeFullU );
    Mat3<float> U = svd.matrixU();
    Mat3<float> V = svd.matrixV();
    outR = U*V.transpose();
    Mat3<float> dR = (outR - lastR)/(currentTime - lastTime);

    Mat3<float> wb_ = outR.transpose()*dR;
    outwb = matToSkewVec(wb_);
    lastTime = currentTime;
    lastR = outR;
}


qrSpline::qrSpline(const float &initial_time,
               const float &duration,
               const Point &start_p,
               const Point &end_p):
    initial_time_(initial_time),
    duration_(duration),
    start_(start_p),
    end_(end_p)
{
    if (duration <= 0.) {
        throw std::invalid_argument("Cannot create a Spliner with zero or negative duration");
    }
}


qrSpline::qrSpline(const float &initial_time,
               const float &duration,
               float start_p,
               float end_p):
    initial_time_(initial_time),
    duration_(duration),
    start_(start_p),
    end_(end_p)
{
    if (duration <= 0.) {
        throw std::invalid_argument("Cannot create a Spliner with zero or negative duration");
    }
}


void qrSpline::setBoundary(const float &initial_time,
                         const float &duration,
                         const Point &start_p,
                         const Point &end_p)
{
    initial_time_ = initial_time;
    duration_ = duration;
    start_ = start_p;
    end_ = end_p;
}


void qrSpline::setBoundary(const float &initial_time,
                         const float &duration,
                         const float &start_p,
                         const float &end_p)
{
    initial_time_ = initial_time;
    duration_ = duration;

    start_.x = start_p;
    start_.xd = 0.0;
    start_.xdd = 0.0;

    end_.x = end_p;
    end_.xd = 0.0;
    end_.xdd = 0.0;
}


bool qrQuadraticSpline::getPoint(const float &current_time, Point &p)
{
    return false;
}


bool qrQuadraticSpline::getPoint(const float &current_time, float &p)
{
    return false;
}


bool qrQuadraticSpline::getPoint(const float &current_time, float mid, Point &out)
{
    /* Sanity check: no interpolation is required if the duration is zero. */
    if (duration_ == 0.) {
        out = start_;
        return true;
    }

    float dt = current_time - initial_time_;
    if (dt > duration_) {
        dt = duration_;
    }

    /* Sanity checks. */
    if (dt < 0.) {
        return false;
    }

    const float mid_phase = 0.5;
    float deltaOne, deltaTwo, deltaThree, coefa, coefb, coefc;
    deltaOne = mid - start_.x;
    deltaTwo = end_.x - start_.x;
    deltaThree = pow(mid_phase, 2) - mid_phase;
    coefa = (deltaOne - deltaTwo * mid_phase) / deltaThree;
    coefb = (deltaTwo * pow(mid_phase, 2) - deltaOne) / deltaThree;
    coefc = start_.x;
    
    /* Interpolated point. */
    out.x = coefa * pow(current_time, 2) + coefb * current_time + coefc;
    out.xd = 0;
    out.xdd = 0;

    return true;
}


bool qrQuadraticSpline::getPoint(const float &current_time, float mid, float &pos)
{
    Point out;
    qrQuadraticSpline::getPoint(current_time, mid, out);
    pos = out.x;

    return true;
}


bool qrCubicSpline::getPoint(const float &current_time, Point &out)
{
    /* Sanity check: no interpolation is required if the duration is zero. */
    if (duration_ == 0.) {
        out = start_;
        return true;
    }

    float dt = current_time - initial_time_;
    if (dt > duration_) {
        dt = duration_;
    }

    /* Sanity checks. */
    if (dt < 0.) {
        return false;
    }

    float a0, a1, a2, a3;
    float T1, T2, T3;
    float dt2, dt3;

    /* Powers of the duration. */
    T1 = duration_;
    T2 = duration_ * T1;
    T3 = duration_ * T2;

    /* Powers of dt. */
    dt2 = dt * dt;
    dt3 = dt * dt2;

    /* Spline coefficients. */
    /* Make four conditions : x(0)=x0, x(1)=xf, x'(0)=v0, x'(1)=vf. */
    a0 = start_.x;
    a1 = start_.xd;
    a2 = -((3 * a0) - (3 * end_.x) + (2 * T1 * a1) + (T1 * end_.xd)) / T2;
    a3 = ((2 * a0) - (2 * end_.x) + T1 * (a1 + end_.xd)) / T3;

    /* Interpolated point. */
    out.x = a0 + a1 * dt + a2 * dt2 + a3 * dt3;
    out.xd = a1 + 2 * a2 * dt + 3 * a3 * dt2;
    out.xdd = 2 * a2 + 6 * a3 * dt;

    return true;
}


bool qrCubicSpline::getPoint(const float &current_time, float &pos)
{
    Point out;
    qrCubicSpline::getPoint(current_time, out);
    pos = out.x;

    return true;
}


bool qrFifthOrderPolySpline::getPoint(const float &current_time, Point &out)
{
    /* Sanity check: no interpolation is required if the duration is zero. */
    if (almostEqual(duration_, 0.f, 1e-3f)) {
        out = start_;
        return true;
    }

    float dt = current_time - initial_time_;
    if (dt > duration_) {
        dt = duration_;
    }

    /* Sanity checks. */
    if (dt < 0) {
        return false;
    }

    float a0, a1, a2, a3, a4, a5;
    float T1, T2, T3, T4, T5;
    float dt2, dt3, dt4, dt5;

    /* Powers of duration. */
    T1 = duration_;
    T2 = duration_ * duration_;
    T3 = duration_ * T2;
    T4 = duration_ * T3;
    T5 = duration_ * T4;

    dt2 = dt * dt;
    dt3 = dt * dt2;
    dt4 = dt * dt3;
    dt5 = dt * dt4;

    Eigen::Matrix<float, 6, 6> A;
    A << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0 ,0,
            1, T1, T2, T3, T4, T5,
            0, 1, 2*T1, 3*T2, 4*T3, 5*T4,
            0, 0, 2, 6*T1, 12*T2, 20*T3;
    Eigen::Matrix<float, 6, 1> b;
    b << start_.x, start_.xd, start_.xdd, end_.x, end_.xd, end_.xdd;
    Eigen::Matrix<float, 6, 1>  a = A.colPivHouseholderQr().solve(b);
    a0 = a[0];
    a1 = a[1];
    a2 = a[2];
    a3 = a[3];
    a4 = a[4];
    a5 = a[5];


    /* Spline coefficients. */
    // a0 = start_.x;
    // a1 = start_.xd;
    // a2 = start_.xdd / 2;
    
    // a3 = (-20 * a0 + 20 * end_.x +
    //     T1 * (-3 * a2 / 2 * T1 + end_.xdd * T1 - 12 * a1 - 8 * end_.xd)) / (2 * T3);
    // a4 = (30 * a0 - 30 * end_.x +
    //     T1 * (3 * start_.xdd * T1 - 2 * end_.xdd * T1 + 16 * start_.xd + 14 * end_.xd)) / (2 * T4);
    // a5 = -(12 * start_.x - 12 * end_.x +
    //     T1 * (start_.xdd * T1 - end_.xdd * T1 + 6 * (start_.xd + end_.xd))) / (2 * T5);
    
    // a3 = (20 * ( end_.x - a0) -
    //         T1 * (14*T1 * a2 - 10*T4 +  end_.xdd * T1 + 18 * a1 + 2 * end_.xd)) / (8 * T3);
    // a4 = (60 *(a0 - end_.x) -
    //         T1 * (70*T4 - 46*a1 - 26*T1*a2 - 3*T1 * end_.xdd - 14 * end_.xd)) / (32 * T4);
    // a5 = (30*T5 - 2 *T2*a2 + T2*end_.xdd - 6*T1*a1- 6*T1* end_.xd - 12*a1+ 12*end_.x) / (32 * T5);

    out.x = a0 + a1 * dt + a2 * dt2 + a3 * dt3 + a4 * dt4 + a5 * dt5;
    out.xd = a1 + 2 * a2 * dt + 3 * a3 * dt2 + 4 * a4 * dt3 + 5 * a5 * dt4;
    out.xdd = 2 * a2 + 6 * a3 * dt + 12 * a4 * dt2 + 20 * a5 * dt3;
    std::cout << a0 << " a1: "<< a1 << "a2 " << a2 << "a3 " << a3 << "a4 " << a4 <<"a5 " << a5 <<std::endl;
    return true;
}


bool qrFifthOrderPolySpline::getPoint(const float &current_time, float &pos)
{
    Point out;
    qrFifthOrderPolySpline::getPoint(current_time, out);
    pos = out.x;

    return true;
}


bool qrLinearSpline::getPoint(const float &current_time, Point &out)
{
    /* Sanity check: no interpolation is required if the duration is zero. */
    if (almostEqual(duration_, 0.f, 1e-3f)) {
        out = start_;
        return true;
    }

    float dt = current_time - initial_time_;
    if (dt > duration_) {
        dt = duration_;
    }

    /* Sanity checks. */
    if (dt < 0.0) {
        return false;
    }

    out.xd = (end_.x - start_.x) / duration_;
    out.x = start_.x + dt * out.xd;
    out.xdd = 0.f;

    return true;
}


bool qrLinearSpline::getPoint(const float &current_time, float &pos)
{
    Point out;
    qrLinearSpline::getPoint(current_time, out);
    pos = out.x;
    return true;
}

} // namespace math

} // namespace robotics
