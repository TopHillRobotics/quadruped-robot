#include "utils/physics_transform.h"


namespace Quadruped {

Mat3<float> transformInertia(Mat3<float> inertia, float mass, Vec3<float> p, Mat3<float> R)
{
    Mat3<float> newInertia = Mat3<float>::Identity();
    Mat3<float> pxp = p*p.transpose();
    newInertia = inertia + mass*(p.dot(p)*Mat3<float>::Identity()-pxp);
    return newInertia;
}

} // namespace Quadruped
