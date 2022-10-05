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
 *  @file qr_floating_base_model.hpp
 *  @brief Implementation of Rigid Body Floating Base model data structure
 *
 * This class stores the kinematic tree described in "Rigid Body Dynamics
 * Algorithms" by Featherstone (download from
 * https://www.springer.com/us/book/9780387743141 on MIT internet)
 *
 * The tree includes an additional "rotor" body for each body.  This rotor is
 * fixed to the parent body and has a gearing constraint.  This is efficiently
 * included using a technique similar to what is described in Chapter 12 of
 * "Robot and Multibody Dynamics" by Jain.  Note that this implementation is
 * highly specific to the case of a single rotating rotor per rigid body. Rotors
 * have the same joint type as their body, but with an additional gear ratio
 * multiplier applied to the motion subspace. The rotors associated with the
 * floating base don't do anything.
 */

#ifndef QR_FLOATING_BASE_MODEL_HPP
#define QR_FLOATING_BASE_MODEL_HPP

#include <eigen3/Eigen/StdVector>

#include "dynamics/qr_spatial.hpp"

// using namespace spatial;

/**
 * @brief The state of a floating base model (base and joints)
 */
template <typename T>
struct FBModelState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quat<T> bodyOrientation;
    Vec3<T> bodyPosition;
    SVec<T> bodyVelocity;  // body coordinates
    DVec<T> q;
    DVec<T> qd;

    /**
     * @brief Print the position of the body
     */
    void print() const {
    printf("position: %.3f %.3f %.3f\n", bodyPosition[0], bodyPosition[1],
            bodyPosition[2]);
    }
};

/**
 * @brief The result of running the articulated body algorithm on a rigid-body floating
 * base model
 */
template <typename T>
struct FBModelStateDerivative {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3<T> dBodyPosition;
    SVec<T> dBodyVelocity;
    DVec<T> qdd;
};

/**
 * @brief Class FloatingBaseModel to represent a floating base rigid body model with rotors and ground
 *        contacts. No concept of state.
 */
template <typename T>
class FloatingBaseModel {

    public:
    /**
     * @brief Initialize a floating base model with default gravity
     */
    FloatingBaseModel() : _gravity(0, 0, -9.81) {}
    ~FloatingBaseModel() {}

    /**
     * @brief Create the floating body
     * @param inertia Spatial inertia of the floating body
     */
    void addBase(const SpatialInertia<T>& inertia);

    /**
     * @brief Create the floating body
     * @param mass Mass of the floating body
     * @param com  Center of mass of the floating body
     * @param I    Rotational inertia of the floating body
     */
    void addBase(T mass, const Vec3<T>& com, const Mat3<T>& I);

    /**
     * @brief Add a ground contact point to a model
     * @param bodyID The ID of the body containing the contact point
     * @param location The location (in body coordinate) of the contact point
     * @param isFoot True if foot or not.
     * @return The ID of the ground contact point
     */
    int addGroundContactPoint(int bodyID, const Vec3<T>& location,
                            bool isFoot = false);

    /**
     * @brief Add the bounding points of a box to the contact model. Assumes the box is
     *        centered around the origin of the body coordinate system and is axis aligned.
     */
    void addGroundContactBoxPoints(int bodyId, const Vec3<T>& dims);

    /**
     * @brief Add a body
     * @param inertia The inertia of the body
     * @param rotorInertia The inertia of the rotor the body is connected to
     * @param gearRatio The gear ratio between the body and the rotor
     * @param parent The parent body, which is also assumed to be the body the rotor
     *               is connected to
     * @param jointType The type of joint (prismatic or revolute)
     * @param jointAxis The joint axis (X,Y,Z), in the parent's frame
     * @param Xtree  The coordinate transformation from parent to this body
     * @param Xrot  The coordinate transformation from parent to this body's rotor
     * @return The body's ID (can be used as the parent)
     */
    int addBody(const SpatialInertia<T>& inertia,
                const SpatialInertia<T>& rotorInertia, T gearRatio, int parent,
                JointType jointType, CoordinateAxis jointAxis,
                const Mat6<T>& Xtree, const Mat6<T>& Xrot);

    /**
     * @brief Add a body
     * @param inertia The inertia of the body
     * @param rotorInertia The inertia of the rotor the body is connected to
     * @param gearRatio The gear ratio between the body and the rotor
     * @param parent The parent body, which is also assumed to be the body the rotor
     *               is connected to
     * @param jointType The type of joint (prismatic or revolute)
     * @param jointAxis The joint axis (X,Y,Z), in the parent's frame
     * @param Xtree  The coordinate transformation from parent to this body
     * @param Xrot  The coordinate transformation from parent to this body's rotor
     * @return The body's ID (can be used as the parent)
     */
    int addBody(const MassProperties<T>& inertia,
                const MassProperties<T>& rotorInertia, T gearRatio, int parent,
                JointType jointType, CoordinateAxis jointAxis,
                const Mat6<T>& Xtree, const Mat6<T>& Xrot);

    /**
     * @brief
     */
    void check();

    /**
     * @brief Compute the total mass of bodies which are not rotors
     * @return
     */
    T totalRotorMass();

    /**
     * @brief Compute the total mass of bodies which are not rotors.
     * @return
     */
    T totalNonRotorMass();

    /**
     * @brief Get vector of parents, where parents[i] is the parent body of body i
     * @return Vector of parents
     */
    const std::vector<int>& getParentVector() { return _parents; }

    /**
     * @brief Get vector of body spatial inertias
     * @return Vector of body spatial inertias
     */
    const std::vector<SpatialInertia<T>,
                    Eigen::aligned_allocator<SpatialInertia<T>>>&
    getBodyInertiaVector() {
    return _Ibody;
    }

    /**
     * @brief Get vector of rotor spatial inertias
     * @return Vector of rotor spatial inertias
     */
    const std::vector<SpatialInertia<T>,
                    Eigen::aligned_allocator<SpatialInertia<T>>>&
    getRotorInertiaVector() {
    return _Irot;
    }

    /**
     * @brief Set the gravity
     */
    void setGravity(Vec3<T>& g) { _gravity = g; }

    /**
     * @brief Set the flag to enable computing contact info for a given contact point
     * @param gc_index : index of contact point
     * @param flag : enable/disable contact calculation
     */
    void setContactComputeFlag(size_t gc_index, bool flag) {
    _compute_contact_info[gc_index] = flag;
    }


    /**
     * @brief Compute the inverse of the contact inertia matrix (mxm)
     * @param force_directions (6xm) each column denotes a direction of interest
     *        col = [ moment in i.c.s., force in i.c.s.]
     *        e.g. if you want the cartesian inv. contact inertia
     *             force_directions = [ 0_{3x3} I_{3x3}]^T
     *             if you only want the cartesian inv. contact inertia in one
     * direction then use the overloaded version.
     * @return the mxm inverse contact inertia J H^{-1} J^T
     */
    DMat<T> invContactInertia(const int gc_index,
                            const D6Mat<T>& force_directions);

    /**
     * @brief Compute the inverse of the contact inertia matrix (mxm)
     * @param force_ics_at_contact (3x1)
     *        e.g. if you want the cartesian inv. contact inertia in the z_ics
     *             force_ics_at_contact = [0 0 1]^T
     * @return the 1x1 inverse contact inertia J H^{-1} J^T
     */
    T invContactInertia(const int gc_index, const Vec3<T>& force_ics_at_contact);

    /**
     * @brief Apply a unit test force at a contact. Returns the inv contact inertia  in
     * that direction and computes the resultant qdd
     * @param gc_index index of the contact
     * @param force_ics_at_contact unit test forcoe
     * @param dstate - Output paramter of resulting accelerations
     * @return the 1x1 inverse contact inertia J H^{-1} J^T
     */
    T applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact,
                    FBModelStateDerivative<T>& dstate_out);

    /**
     * @brief  Apply a unit test force at a contact. Returns the inv contact inertia  in
     *         that direction and computes the resultant qdd
     * @param gc_index index of the contact
     * @param force_ics_at_contact unit test force expressed in inertial coordinates
     * @param dstate - Output paramter of resulting accelerations
     * @return the 1x1 inverse contact inertia J H^{-1} J^T
     */
    T applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact,
                    DVec<T>& dstate_out);

    /**
     * @brief Populate member variables when bodies are added
     * @param count (6 for fb, 1 for joint)
     */
    void addDynamicsVars(int count);

    /**
     * @brief Updates the size of H, C, Cqd, G, and Js when bodies are added
     */
    void resizeSystemMatricies();

    /**
     * @brief Update the state of the simulator, invalidating previous results
     * @param state : the new state
     */
    void setState(const FBModelState<T>& state) {
        _state = state;

        _biasAccelerationsUpToDate = false;
        _compositeInertiasUpToDate = false;

        resetCalculationFlags();
    }

    /**
     * @brief Mark all previously calculated values as invalid
     */
    void resetCalculationFlags() {
        _articulatedBodiesUpToDate = false;
        _kinematicsUpToDate = false;
        _forcePropagatorsUpToDate = false;
        _qddEffectsUpToDate = false;
        _accelerationsUpToDate = false;
    }

    /**
     * @brief Update the state derivative of the simulator, invalidating previous results.
     * @param dState : the new state derivative
     */
    void setDState(const FBModelStateDerivative<T>& dState) {
        _dState = dState;
        _accelerationsUpToDate = false;
    }

    Vec3<T> getPosition(const int link_idx, const Vec3<T> & local_pos);
    Vec3<T> getPosition(const int link_idx);


    Mat3<T> getOrientation(const int link_idx);
    Vec3<T> getLinearVelocity(const int link_idx, const Vec3<T>& point);
    Vec3<T> getLinearVelocity(const int link_idx);

    Vec3<T> getLinearAcceleration(const int link_idx, const Vec3<T>& point);
    Vec3<T> getLinearAcceleration(const int link_idx);

    Vec3<T> getAngularVelocity(const int link_idx);
    Vec3<T> getAngularAcceleration(const int link_idx);

    /**
     * @brief Forward kinematics of all bodies.  Computes _Xup (from up the tree) and _Xa
     * (from absolute) Also computes _S (motion subspace), _v (spatial velocity in
     * link coordinates), and _c (coriolis acceleration in link coordinates)
     */
    void forwardKinematics();

    /**
     * @brief (Support Function) Computes velocity product accelerations of
     * each link and rotor _avp, and _avprot
     */
    void biasAccelerations();

    /**
     * @brief (Support Function) Computes the composite rigid body inertia
     * of each subtree _IC[i] contains body i, and the body/rotor
     * inertias of all successors of body i.
     * (key note: _IC[i] does not contain rotor i)
     */
    void compositeInertias();


    void forwardAccelerationKinematics();

    /**
     * @brief Compute the contact Jacobians (3xn matrices) for the velocity
     * of each contact point expressed in absolute coordinates
     */
    void contactJacobians();

    /**
     * @brief Computes the generalized gravitational force (G) in the inverse dynamics
     * @return G (_nDof x 1 vector)
     */
    DVec<T> generalizedGravityForce();

    /**
     * @brief Computes the generalized coriolis forces (Cqd) in the inverse dynamics
     * @return Cqd (_nDof x 1 vector)
     */
    DVec<T> generalizedCoriolisForce();

    /**
     * @brief Computes the Mass Matrix (H) in the inverse dynamics formulation
     * @return H (_nDof x _nDof matrix)
     */
    DMat<T> massMatrix();
    
    /**
     * @brief Computes the inverse dynamics of the system
     * @return an _nDof x 1 vector. The first six entries
     * give the external wrengh on the base, with the remaining giving the
     * joint torques
     */
    DVec<T> inverseDynamics(const FBModelStateDerivative<T>& dState);
    void runABA(const DVec<T>& tau, FBModelStateDerivative<T>& dstate);

    size_t _nDof = 0;
    Vec3<T> _gravity;
    std::vector<int> _parents;
    std::vector<T> _gearRatios;
    std::vector<T> _d, _u;

    std::vector<JointType> _jointTypes;
    std::vector<CoordinateAxis> _jointAxes;
    std::vector<Mat6<T>, Eigen::aligned_allocator<Mat6<T>>> _Xtree, _Xrot;
    std::vector<SpatialInertia<T>, Eigen::aligned_allocator<SpatialInertia<T>>> _Ibody,
        _Irot;
    std::vector<std::string> _bodyNames;

    size_t _nGroundContact = 0;
    std::vector<size_t> _gcParent;
    std::vector<Vec3<T>> _gcLocation;
    std::vector<uint64_t> _footIndicesGC;

    std::vector<Vec3<T>> _pGC;
    std::vector<Vec3<T>> _vGC;

    std::vector<bool> _compute_contact_info;

    /**
     * @brief Get the mass matrix for the system
     */
    const DMat<T>& getMassMatrix() const { return _H; }

    /**
     * @brief Get the gravity term (generalized forces)
     */
    const DVec<T>& getGravityForce() const { return _G; }

    /**
     * @brief Get the coriolis term (generalized forces)
     */
    const DVec<T>& getCoriolisForce() const { return _Cqd; }


    /// BEGIN ALGORITHM SUPPORT VARIABLES
    FBModelState<T> _state;
    FBModelStateDerivative<T> _dState;

    vectorAligned<SVec<T>> _v, _vrot, _a, _arot, _avp, _avprot, _c, _crot, _S,
        _Srot, _fvp, _fvprot, _ag, _agrot, _f, _frot;

    vectorAligned<SVec<T>> _U, _Urot, _Utot, _pA, _pArot;
    vectorAligned<SVec<T>> _externalForces;

    vectorAligned<SpatialInertia<T>> _IC;
    vectorAligned<Mat6<T>> _Xup, _Xa, _Xuprot, _IA, _ChiUp;

    DMat<T> _H, _C;
    DVec<T> _Cqd, _G;

    vectorAligned<D6Mat<T>> _J;
    vectorAligned<SVec<T>> _Jdqd;

    vectorAligned<D3Mat<T>> _Jc;
    vectorAligned<Vec3<T>> _Jcdqd;

    bool _kinematicsUpToDate = false;
    bool _biasAccelerationsUpToDate = false;
    bool _accelerationsUpToDate = false;

    bool _compositeInertiasUpToDate = false;

    /**
     * @brief Support function for the ABA
     */
    void updateArticulatedBodies();

    /**
     * @brief Support function for contact inertia algorithms
     *        Comptues force propagators across each joint
     */
    void updateForcePropagators();

    /**
     * @brief Support function for contact inertia algorithms
     *        Computes the qdd arising from "subqdd" components
     *        If you are familiar with Featherstone's sparse Op sp
     *        or jain's innovations factorization:
     *        H = L * D * L^T
     *        These subqdd components represnt the space in the middle
     *        i.e. if H^{-1} = L^{-T} * D^{-1} * L^{1}
     *        then what I am calling subqdd = L^{-1} * tau
     *        This is an awful explanation. It needs latex.
     */
    void udpateQddEffects();

    /**
     * @brief Set all external forces to zero
     */
    void resetExternalForces() {
    for (size_t i = 0; i < _nDof; i++) {
        _externalForces[i] = SVec<T>::Zero();
    }
    }

    /**
     * @brief if has updated the articulated bodies.
     */
    bool _articulatedBodiesUpToDate = false;

    /**
     * @brief if has updated the force propagator.
     */
    bool _forcePropagatorsUpToDate = false;

    /**
     * @brief if has updated the qdd effects.
     */
    bool _qddEffectsUpToDate = false;

    DMat<T> _qdd_from_base_accel;
    DMat<T> _qdd_from_subqdd;
    Eigen::ColPivHouseholderQR<Mat6<T>> _invIA5;
};

#endif // QR_FLOATING_BASE_MODEL_HPP