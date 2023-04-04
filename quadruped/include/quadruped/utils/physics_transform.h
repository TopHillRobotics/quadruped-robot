#ifndef QR_PHYSICS_TRANSFORM_H
#define QR_PHYSICS_TRANSFORM_H

#include "utils/qr_cpptypes.h"
#include "utils/qr_se3.h"


namespace Quadruped {

/** 
 * @brief Transform rigid body rotaion inertia.
 * @param inertia: interia in intric body frame, that is origin at COM.
 * @param mass: body mass.
 * @param p: the reference point reletive to COM where interia is represented.
 * @param R: the orientation of new frame relative to Body frame, where inertia is represented.
 * @return new inertia matrix.
 */
Mat3<float> transformInertia(Mat3<float> inertia, float mass, Vec3<float> p, Mat3<float> R=Mat3<float>::Identity());

} // Namespace Quadruped

#endif // QR_PHYSICS_TRANSFORM_H
