#ifndef QR_PHYSICS_TRANSFORM_H
#define QR_PHYSICS_TRANSFORM_H

#include "common/qr_cTypes.h"
#include "common/qr_se3.h"

/** 
 * @brief transform rigid body rotaion inertia.
 * @param inertia interia in intric body frame, that is origin at COM
 * @param mass body mass
 * @param p the reference point reletive to COM where interia is represented.
 * @param R the orientation of new frame relative to Body frame, where inertia is represented.
 * @result Mat3<float> : new inertia matrix
 */
Mat3<float> transformInertia(Mat3<float> inertia, float mass, Vec3<float> p, Mat3<float> R=Mat3<float>::Identity());

#endif //QR_PHYSICS_TRANSFORM_H