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

/**
 * @file spatial.h
 * @brief Utility functions for manipulating spatial quantities
 *
 * This file contains functions for working with spatial vectors and
 * transformation matrices.
 */

#ifndef QR_SPATIAL_H
#define QR_SPATIAL_H

#include <iostream>
#include <type_traits>

#include "common/qr_se3.h"

using namespace math;
enum class JointType { Prismatic,
                        Revolute,
                        FloatingBase,
                        Nothing };

/**
* @brief Calculate the spatial coordinate transform from A to B where B is rotate by
* theta about axis.
*/
template<typename T>
SXform<T> spatialRotation(CoordinateAxis axis, T theta)
{
    static_assert(std::is_floating_point<T>::value,
                    "must use floating point value");
    RotMat<T> R = coordinateRotation(axis, theta);
    SXform<T> X = SXform<T>::Zero();
    X.template topLeftCorner<3, 3>() = R;
    X.template bottomRightCorner<3, 3>() = R;
    return X;
}

/**
* @brief Compute the spatial motion cross product matrix.
* Prefer motionCrossProduct when possible.
*/
template<typename T>
Mat6<typename T::Scalar> motionCrossMatrix(const Eigen::MatrixBase<T> &v)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
    Mat6<typename T::Scalar> m;
    m << 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0), 0, 0, 0, -v(1), v(0), 0, 0, 0,
        0,

        0, -v(5), v(4), 0, -v(2), v(1), v(5), 0, -v(3), v(2), 0, -v(0), -v(4),
        v(3), 0, -v(1), v(0), 0;
    return m;
}

/**
* @brief Compute spatial force cross product matrix.
* Prefer forceCrossProduct when possible
*/
template<typename T>
Mat6<typename T::Scalar> forceCrossMatrix(const Eigen::MatrixBase<T> &v)
{
    Mat6<typename T::Scalar> f;
    f << 0, -v(2), v(1), 0, -v(5), v(4), v(2), 0, -v(0), v(5), 0, -v(3), -v(1),
        v(0), 0, -v(4), v(3), 0, 0, 0, 0, 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0),
        0, 0, 0, -v(1), v(0), 0;
    return f;
}

/**
* @brief Compute spatial motion cross product.  Faster than the matrix multiplication
* version
*/
template<typename T>
SVec<typename T::Scalar> motionCrossProduct(const Eigen::MatrixBase<T> &a,
                                            const Eigen::MatrixBase<T> &b)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
    SVec<typename T::Scalar> mv;
    mv << a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2),
        a(0) * b(1) - a(1) * b(0),
        a(1) * b(5) - a(2) * b(4) + a(4) * b(2) - a(5) * b(1),
        a(2) * b(3) - a(0) * b(5) - a(3) * b(2) + a(5) * b(0),
        a(0) * b(4) - a(1) * b(3) + a(3) * b(1) - a(4) * b(0);
    return mv;
}

/**
* @brief Compute spatial force cross product.  Faster than the matrix multiplication
* version
*/
template<typename T>
SVec<typename T::Scalar> forceCrossProduct(const Eigen::MatrixBase<T> &a,
                                            const Eigen::MatrixBase<T> &b)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
    SVec<typename T::Scalar> mv;
    mv << b(2) * a(1) - b(1) * a(2) - b(4) * a(5) + b(5) * a(4),
        b(0) * a(2) - b(2) * a(0) + b(3) * a(5) - b(5) * a(3),
        b(1) * a(0) - b(0) * a(1) - b(3) * a(4) + b(4) * a(3),
        b(5) * a(1) - b(4) * a(2), b(3) * a(2) - b(5) * a(0),
        b(4) * a(0) - b(3) * a(1);
    return mv;
}

/**
* @brief Convert a spatial transform to a homogeneous coordinate transformation
*/
template<typename T>
Mat4<typename T::Scalar> sxformToHomogeneous(const Eigen::MatrixBase<T> &X)
{
    static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 matrix");
    Mat4<typename T::Scalar> H = Mat4<typename T::Scalar>::Zero();
    RotMat<typename T::Scalar> R = X.template topLeftCorner<3, 3>();
    Mat3<typename T::Scalar> skewR = X.template bottomLeftCorner<3, 3>();
    H.template topLeftCorner<3, 3>() = R;
    H.template topRightCorner<3, 1>() = matToSkewVec(skewR * R.transpose());
    H(3, 3) = 1;
    return H;
}

/**
* @brief Convert a homogeneous coordinate transformation to a spatial one
*/
template<typename T>
Mat6<typename T::Scalar> homogeneousToSXform(const Eigen::MatrixBase<T> &H)
{
    static_assert(T::ColsAtCompileTime == 4 && T::RowsAtCompileTime == 4,
                    "Must have 4x4 matrix");
    Mat3<typename T::Scalar> R = H.template topLeftCorner<3, 3>();
    Vec3<typename T::Scalar> translate = H.template topRightCorner<3, 1>();
    Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
    X.template topLeftCorner<3, 3>() = R;
    X.template bottomLeftCorner<3, 3>() = vectorToSkewMat(translate) * R;
    X.template bottomRightCorner<3, 3>() = R;
    return X;
}

/**
* @brief Create spatial coordinate transformation from rotation and translation
*/
template<typename T, typename T2>
Mat6<typename T::Scalar> createSXform(const Eigen::MatrixBase<T> &R,
                                        const Eigen::MatrixBase<T2> &r)
{
    static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                    "Must have 3x3 matrix");
    static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 matrix");
    Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
    X.template topLeftCorner<3, 3>() = R;
    X.template bottomRightCorner<3, 3>() = R;
    X.template bottomLeftCorner<3, 3>() = -R * vectorToSkewMat(r);
    return X;
}

/**
* @brief Get rotation matrix from spatial transformation
*/
template<typename T>
RotMat<typename T::Scalar> rotationFromSXform(const Eigen::MatrixBase<T> &X)
{
    static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 matrix");
    RotMat<typename T::Scalar> R = X.template topLeftCorner<3, 3>();
    return R;
}

/**
* @brief Get translation vector from spatial transformation
*/
template<typename T>
Vec3<typename T::Scalar> translationFromSXform(const Eigen::MatrixBase<T> &X)
{
    static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 matrix");
    RotMat<typename T::Scalar> R = rotationFromSXform(X);
    Vec3<typename T::Scalar> r =
        -matToSkewVec(R.transpose() * X.template bottomLeftCorner<3, 3>());
    return r;
}

/**
* @brief Invert a spatial transformation (much faster than matrix inverse)
*/
template<typename T>
SXform<typename T::Scalar> invertSXform(const Eigen::MatrixBase<T> &X)
{
    static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 matrix");
    RotMat<typename T::Scalar> R = rotationFromSXform(X);
    Vec3<typename T::Scalar> r =
        -matToSkewVec(R.transpose() * X.template bottomLeftCorner<3, 3>());
    SXform<typename T::Scalar> Xinv = createSXform(R.transpose(), -R * r);
    return Xinv;
}

/**
* @brief Compute joint motion subspace vector
*/
template<typename T>
SVec<T> jointMotionSubspace(JointType joint, CoordinateAxis axis)
{
    Vec3<T> v(0, 0, 0);
    SVec<T> phi = SVec<T>::Zero();
    if (axis == CoordinateAxis::X)
        v(0) = 1;
    else if (axis == CoordinateAxis::Y)
        v(1) = 1;
    else
        v(2) = 1;

    if (joint == JointType::Prismatic)
        phi.template bottomLeftCorner<3, 1>() = v;
    else if (joint == JointType::Revolute)
        phi.template topLeftCorner<3, 1>() = v;
    else
        throw std::runtime_error("Unknown motion subspace");

    return phi;
}

/**
* @brief Compute joint transformation
*/
template<typename T>
Mat6<T> jointXform(JointType joint, CoordinateAxis axis, T q)
{
    Mat6<T> X = Mat6<T>::Zero();
    if (joint == JointType::Revolute) {
        X = spatialRotation(axis, q);
    } else if (joint == JointType::Prismatic) {
        Vec3<T> v(0, 0, 0);
        if (axis == CoordinateAxis::X)
            v(0) = q;
        else if (axis == CoordinateAxis::Y)
            v(1) = q;
        else if (axis == CoordinateAxis::Z)
            v(2) = q;

        X = createSXform(RotMat<T>::Identity(), v);
    } else {
        throw std::runtime_error("Unknown joint xform\n");
    }
    return X;
}

/**
* @brief Construct the rotational inertia of a uniform density box with a given mass.
* @param mass Mass of the box
* @param dims Dimensions of the box
*/
template<typename T>
Mat3<typename T::Scalar> rotInertiaOfBox(typename T::Scalar mass,
                                            const Eigen::MatrixBase<T> &dims)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");
    Mat3<typename T::Scalar> I =
        Mat3<typename T::Scalar>::Identity() * dims.norm() * dims.norm();
    for (int i = 0; i < 3; i++) I(i, i) -= dims(i) * dims(i);
    I = I * mass / 12;
    return I;
}

/**
* @brief Convert from spatial velocity to linear velocity.
* Uses spatial velocity at the given point.
*/
template<typename T, typename T2>
Vec3<typename T::Scalar> spatialToLinearVelocity(const Eigen::MatrixBase<T> &v,
                                                    const Eigen::MatrixBase<T2> &x)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
    static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");
    Vec3<typename T::Scalar> vsAng = v.template topLeftCorner<3, 1>();
    Vec3<typename T::Scalar> vsLin = v.template bottomLeftCorner<3, 1>();
    Vec3<typename T::Scalar> vLinear = vsLin + vsAng.cross(x);
    return vLinear;
}

/**
* @brief Convert from spatial velocity to angular velocity.
*/
template<typename T>
Vec3<typename T::Scalar> spatialToAngularVelocity(const Eigen::MatrixBase<T> &v)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
    Vec3<typename T::Scalar> vsAng = v.template topLeftCorner<3, 1>();
    return vsAng;
}

/**
* @brief Compute the classical lienear accleeration of a frame given its spatial
* acceleration and velocity
*/
template<typename T, typename T2>
Vec3<typename T::Scalar> spatialToLinearAcceleration(const Eigen::MatrixBase<T> &a,
                                                        const Eigen::MatrixBase<T2> &v)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
    static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");

    Vec3<typename T::Scalar> acc;
    // classical accleration = spatial linear acc + omega x v
    acc = a.template tail<3>() + v.template head<3>().cross(v.template tail<3>());
    return acc;
}

/**
* @brief Compute the classical lienear acceleration of a frame given its spatial
* acceleration and velocity
*/
template<typename T, typename T2, typename T3>
Vec3<typename T::Scalar> spatialToLinearAcceleration(const Eigen::MatrixBase<T> &a,
                                                        const Eigen::MatrixBase<T2> &v,
                                                        const Eigen::MatrixBase<T3> &x)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
    static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 6,
                    "Must have 6x1 vector");
    static_assert(T3::ColsAtCompileTime == 1 && T3::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");

    Vec3<typename T::Scalar> alin_x = spatialToLinearVelocity(a, x);
    Vec3<typename T::Scalar> vlin_x = spatialToLinearVelocity(v, x);

    // classical accleration = spatial linear acc + omega x v
    Vec3<typename T::Scalar> acc = alin_x + v.template head<3>().cross(vlin_x);
    return acc;
}

/**
* @brief Apply spatial transformation to a point.
*/
template<typename T, typename T2>
Vec3<typename T::Scalar> sXFormPoint(const Eigen::MatrixBase<T> &X,
                                        const Eigen::MatrixBase<T2> &p)
{
    static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                    "Must have 6x6 vector");
    static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");

    Mat3<typename T::Scalar> R = rotationFromSXform(X);
    Vec3<typename T::Scalar> r = translationFromSXform(X);
    Vec3<typename T::Scalar> Xp = R * (p - r);
    return Xp;
}

/**
* @brief Convert a force at a point to a spatial force
* @param f : force
* @param p : point
*/
template<typename T, typename T2>
SVec<typename T::Scalar> forceToSpatialForce(const Eigen::MatrixBase<T> &f,
                                                const Eigen::MatrixBase<T2> &p)
{
    static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");
    static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                    "Must have 3x1 vector");
    SVec<typename T::Scalar> fs;
    fs.template topLeftCorner<3, 1>() = p.cross(f);
    fs.template bottomLeftCorner<3, 1>() = f;
    return fs;
}

/**
* @brief Representation of Rigid Body Inertia as a 6x6 Spatial Inertia Tensor
*/
template<typename T>
class SpatialInertia {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
    * @brief Construct spatial inertia from mass, center of mass, and 3x3 rotational
    * inertia
    */
    SpatialInertia(T mass, const Vec3<T> &com, const Mat3<T> &inertia)
    {
        Mat3<T> cSkew = vectorToSkewMat(com);
        _inertia.template topLeftCorner<3, 3>() =
            inertia + mass * cSkew * cSkew.transpose();
        _inertia.template topRightCorner<3, 3>() = mass * cSkew;
        _inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
        _inertia.template bottomRightCorner<3, 3>() = mass * Mat3<T>::Identity();
    }

    /**
    * @brief Construct spatial inertia from 6x6 matrix
    */
    explicit SpatialInertia(const Mat6<T> &inertia) { _inertia = inertia; }

    /**
    * @brief If no argument is given, zero.
    */
    SpatialInertia() { _inertia = Mat6<T>::Zero(); }

    /**
    * @brief Construct spatial inertia from mass property vector
    */
    explicit SpatialInertia(const MassProperties<T> &a)
    {
        _inertia(0, 0) = a(4);
        _inertia(0, 1) = a(9);
        _inertia(0, 2) = a(8);
        _inertia(1, 0) = a(9);
        _inertia(1, 1) = a(5);
        _inertia(1, 2) = a(7);
        _inertia(2, 0) = a(8);
        _inertia(2, 1) = a(7);
        _inertia(2, 2) = a(6);
        Mat3<T> cSkew = vectorToSkewMat(Vec3<T>(a(1), a(2), a(3)));
        _inertia.template topRightCorner<3, 3>() = cSkew;
        _inertia.template bottomLeftCorner<3, 3>() = cSkew.transpose();
        _inertia.template bottomRightCorner<3, 3>() = a(0) * Mat3<T>::Identity();
    }

    /**
    * @brief Construct spatial inertia from pseudo-inertia. This is described in
    *   Linear Matrix Inequalities for Physically Consistent Inertial Parameter
    *   Identification: A Statistical Perspective on the Mass Distribution, by
    *   Wensing, Kim, Slotine
    * @param P
    */
    explicit SpatialInertia(const Mat4<T> &P)
    {
        Mat6<T> I;
        T m = P(3, 3);
        Vec3<T> h = P.template topRightCorner<3, 1>();
        Mat3<T> E = P.template topLeftCorner<3, 3>();
        Mat3<T> Ibar = E.trace() * Mat3<T>::Identity() - E;
        I.template topLeftCorner<3, 3>() = Ibar;
        I.template topRightCorner<3, 3>() = vectorToSkewMat(h);
        I.template bottomLeftCorner<3, 3>() = vectorToSkewMat(h).transpose();
        I.template bottomRightCorner<3, 3>() = m * Mat3<T>::Identity();
        _inertia = I;
    }

    /**
    * @brief Convert spatial inertia to mass property vector
    */
    MassProperties<T> asMassPropertyVector()
    {
        MassProperties<T> a;
        Vec3<T> h = matToSkewVec(_inertia.template topRightCorner<3, 3>());
        a << _inertia(5, 5), h(0), h(1), h(2), _inertia(0, 0), _inertia(1, 1),
            _inertia(2, 2), _inertia(2, 1), _inertia(2, 0), _inertia(1, 0);
        return a;
    }

    /**
    * @brief Get 6x6 spatial inertia
    */
    const Mat6<T> &getMatrix() const { return _inertia; }

    void setMatrix(const Mat6<T> &mat) { _inertia = mat; }

    void addMatrix(const Mat6<T> &mat) { _inertia += mat; }

    /**
    * @brief Get mass
    */
    T getMass() { return _inertia(5, 5); }

    /**
    * @brief Get center of mass location
    */
    Vec3<T> getCOM()
    {
        T m = getMass();
        Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
        Vec3<T> com = matToSkewVec(mcSkew) / m;
        return com;
    }

    /**
    * @brief Get 3x3 rotational inertia
    */
    Mat3<T> getInertiaTensor()
    {
        T m = getMass();
        Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
        Mat3<T> I_rot = _inertia.template topLeftCorner<3, 3>() - mcSkew * mcSkew.transpose() / m;
        return I_rot;
    }

    /**
    * @brief Convert to 4x4 pseudo-inertia matrix.  This is described in
    *   Linear Matrix Inequalities for Physically Consistent Inertial Parameter
    *   Identification: A Statistical Perspective on the Mass Distribution, by
    *   Wensing, Kim, Slotine
    */
    Mat4<T> getPseudoInertia()
    {
        Vec3<T> h = matToSkewVec(_inertia.template topRightCorner<3, 3>());
        Mat3<T> Ibar = _inertia.template topLeftCorner<3, 3>();
        T m = _inertia(5, 5);
        Mat4<T> P;
        P.template topLeftCorner<3, 3>() =
            0.5 * Ibar.trace() * Mat3<T>::Identity() - Ibar;
        P.template topRightCorner<3, 1>() = h;
        P.template bottomLeftCorner<1, 3>() = h.transpose();
        P(3, 3) = m;
        return P;
    }

    /**
    * @brief Flip inertia matrix around an axis.  This isn't efficient, but it works!
    */
    SpatialInertia flipAlongAxis(CoordinateAxis axis)
    {
        Mat4<T> P = getPseudoInertia();
        Mat4<T> X = Mat4<T>::Identity();
        if (axis == CoordinateAxis::X)
            X(0, 0) = -1;
        else if (axis == CoordinateAxis::Y)
            X(1, 1) = -1;
        else if (axis == CoordinateAxis::Z)
            X(2, 2) = -1;
        P = X * P * X;
        return SpatialInertia(P);
    }

private:
    Mat6<T> _inertia;
};

#endif //QR_SPATIAL_H