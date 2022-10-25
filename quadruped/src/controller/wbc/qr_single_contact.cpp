// The MIT License

#include "controller/wbc/qr_single_contact.hpp"

// [ Fx, Fy, Fz ]
template <typename T>
qrSingleContact<T>::qrSingleContact(FloatingBaseModel<T>* robot, int pt)
    : qrContactSpec<T>(3), robot_sys_(robot), _max_Fz(robot->totalNonRotorMass()*(T)9.81), _contact_pt(pt), _dim_U(6)
{
    qrContactSpec<T>::idx_Fz_ = 2;
    qrContactSpec<T>::Jc_ = DMat<T>(qrContactSpec<T>::dim_contact_, 18); // (3, 18)
    qrContactSpec<T>::JcDotQdot_ = DVec<T>::Zero(qrContactSpec<T>::dim_contact_); // (3,)
    qrContactSpec<T>::Uf_ = DMat<T>::Zero(_dim_U, qrContactSpec<T>::dim_contact_); // (6, 3)

    T mu(0.4);

    qrContactSpec<T>::Uf_(0, 2) = 1.;

    qrContactSpec<T>::Uf_(1, 0) = 1.;
    qrContactSpec<T>::Uf_(1, 2) = mu;
    qrContactSpec<T>::Uf_(2, 0) = -1.;
    qrContactSpec<T>::Uf_(2, 2) = mu;

    qrContactSpec<T>::Uf_(3, 1) = 1.;
    qrContactSpec<T>::Uf_(3, 2) = mu;
    qrContactSpec<T>::Uf_(4, 1) = -1.;
    qrContactSpec<T>::Uf_(4, 2) = mu;

    // Upper bound of normal force
    qrContactSpec<T>::Uf_(5, 2) = -1.;
}

template <typename T>
qrSingleContact<T>::~qrSingleContact() {}

template <typename T>
bool qrSingleContact<T>::_UpdateJc() {
    qrContactSpec<T>::Jc_ = robot_sys_->_Jc[_contact_pt]; // (3, 18)
    return true;
}

template <typename T>
bool qrSingleContact<T>::_UpdateJcDotQdot() {
    qrContactSpec<T>::JcDotQdot_ = robot_sys_->_Jcdqd[_contact_pt];
    return true;
}

template <typename T>
bool qrSingleContact<T>::_UpdateUf() {
    return true;
}

template <typename T>
bool qrSingleContact<T>::_UpdateInequalityVector() {
    qrContactSpec<T>::ieq_vec_ = DVec<T>::Zero(_dim_U);
    qrContactSpec<T>::ieq_vec_[5] = -_max_Fz;
    return true;
}

template class qrSingleContact<float>;