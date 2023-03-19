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

#include "controllers/wbc/qr_single_contact.hpp"


template<typename T>
qrSingleContact<T>::qrSingleContact(FloatingBaseModel<T> *robot, int pt):
    fbModel(robot),
    maxFz(robot->totalNonRotorMass() * (T)9.81),
    indexContact(pt),
    dimU(6),
    mu(0.4f),
    dimContact(3)
{
    indexFz = dimContact - 1;

    desiredFr = DVec<T>::Zero(dimContact);
    Jc = DMat<T>(dimContact, 18);
    JcDotQdot = DVec<T>::Zero(dimContact);

    Uf = DMat<T>::Zero(dimU, dimContact);

    /* Uf matrix seems like:
     * |  0   0   1  |
     * |  1   0   mu |
     * | -1   0   mu |
     * |  0   1   mu |
     * |  0  -1   mu |
     * |  0   0  -1  |
     */
    Uf(0, 2) = 1.;

    Uf(1, 0) = 1.;
    Uf(1, 2) = mu;

    Uf(2, 0) = -1.;
    Uf(2, 2) = mu;

    Uf(3, 1) = 1.;
    Uf(3, 2) = mu;

    Uf(4, 1) = -1.;
    Uf(4, 2) = mu;

    Uf(5, 2) = -1.;
}


template<typename T>
bool qrSingleContact<T>::UpdateContactSpec()
{
    UpdateJc();
    UpdateJcDotQdot();
    UpdateUf();
    UpdateIneqVec();
    return true;
}


template<typename T>
bool qrSingleContact<T>::UpdateJc()
{
    Jc = fbModel->_Jc[indexContact];
    return true;
}


template<typename T>
bool qrSingleContact<T>::UpdateJcDotQdot()
{
    JcDotQdot = fbModel->_Jcdqd[indexContact];
    return true;
}


template<typename T>
bool qrSingleContact<T>::UpdateUf()
{
    return true;
}


template<typename T>
bool qrSingleContact<T>::UpdateIneqVec()
{
    ineqVec = DVec<T>::Zero(dimU);
    ineqVec[5] = -maxFz;
    return true;
}


template class qrSingleContact<float>;
