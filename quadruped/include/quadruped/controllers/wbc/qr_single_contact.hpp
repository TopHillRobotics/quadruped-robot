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

#ifndef QR_SINGLE_CONTACT_H
#define QR_SINGLE_CONTACT_H

#include "dynamics/floating_base_model.hpp"


template<typename T>
class qrSingleContact {

public:

    /**
     * @brief Constructor of the class qrSingleContact.
     * @param robot: Class to represent a floating base rigid body model with rotors and ground contacts. No concept of state.
     * @param contact_pt: the number of contact points.
     */
    qrSingleContact(FloatingBaseModel<T> *robot, int contact_pt);

    virtual ~qrSingleContact() = default;

    /**
     * @brief Update contact jacobian and inequivalent constraint vector.
     * @return true if update has finished.
     */
    bool UpdateContactSpec();

    /**
     * @brief Get the row count/dimension of member Uf.
     */
    size_t GetDimUf() const {
        return Uf.rows();
    }

    /**
     * @brief Getter method of member dimContact.
     */
    size_t GetDimContact() const {
        return dimContact;
    }

    /**
     * @brief Getter method of member Jc.
     */
    void GetJc(DMat<T>& Jc) const {
        Jc = this->Jc;
    }

    /**
     * @brief Getter method of member JcDotQDot.
     */
    void GetJcDotQdot(DVec<T>& JcDotQdot) const {
        JcDotQdot = this->JcDotQdot;
    }

    /**
     * @brief Getter method of member Uf.
     */
    void GetUf(DMat<T>& Uf) const {
        Uf = this->Uf;
    }

    /**
     * @brief Getter method of member ineqVec.
     */
    void GetIneqVec(DVec<T>& ineqVec) const {
        ineqVec = this->ineqVec;
    }

    /**
     * @brief Getter method of member desiredFr
     */
    const DVec<T>& GetDesiredFr() const {
        return desiredFr;
    }

    /**
     * @brief Setter method of member desiredFr.
     */
    void SetDesiredFr(const DVec<T>& desiredFr) {
        this->desiredFr = desiredFr;
    }

protected:

    /**
     * @brief Update Jc by reading the contact jacobian from MIT floating base model.
     * @return true if the update has finished.
     */
    bool UpdateJc();

    /**
     * @brief Update JcDotQdot by reading the contact jacobian from MIT floating base model.
     * @return true if the update has finished.
     */
    bool UpdateJcDotQdot();

    /**
     * @brief Update Uf matrix.
     * Currently the matrix is constant, so the method just return true.
     * @todo Consider add dynamic terrain information to UpdateUf().
     * @return true if the update has finished.
     */
    bool UpdateUf();

    /**
     * @brief Update ineqVec.
     * Currently only limiting the force on Z axis not to be greater than the weight of quadruped,
     * or it will start bumping.
     * @return true if the update has finished.
     */
    bool UpdateIneqVec();
    
    /**
     * @brief Pointer to MIT floating base model.
     * Used to get Jc and JDotQDot.
     */
    FloatingBaseModel<T> *fbModel;

    /**
     * @brief Maximum force of the contact point along z-axis.
     */
    T maxFz;

    /**
     * @brief Current index of contact point.
     * 0, 1, 2, 3 for FR, FL, RR, RL.
     */
    int indexContact;

    /**
     * @brief Dimension of constraint matrix.
     * Usually set to 6, including 4 conic constraints and 2 boundary constraints.
     */
    int dimU;

    /**
     * @brief Friction factor of the terrain.
     * @todo Consider add dynamic terrain information to mu.
     */
    T mu;

    /**
     * @brief z-force index in force vector.
     * Usually set to 2.
     */
    int indexFz;

    /**
     * @brief 6x3 inequivalent constraint martix, including conic and boundary constraints.
     *
     */
    DMat<T> Uf;


    /**
     * @brief Desired reaction force.
     * This is set to the result force from MPC solver.
     */
    DVec<T> desiredFr;

    /**
     * @brief 6x1 inequivalent vector.
     * The sixth entry is set to -maxFz to satisfy fz < maxFz.
     */
    DVec<T> ineqVec;

    /**
     * @brief Single contact jacobian of corresponding contact point.
     */
    DMat<T> Jc;

    /**
     * @brief Derivative of Jc dot derivative of q.
     * Used in null-space projection.
     */
    DVec<T> JcDotQdot;

    /**
     * @brief Dimension of contact point.
     * Usually set to 3. (x, y, z)
     */
    size_t dimContact;

};

#endif // QR_SINGLE_CONTACT_H
