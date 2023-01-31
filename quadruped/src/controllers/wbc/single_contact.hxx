// [ Fx, Fy, Fz ]
template<typename T>
SingleContact<T>::SingleContact(FloatingBaseModel<T> *robot, int pt)
    : dimContact(3), b_set_contact_(false), robot_sys_(robot), _max_Fz(robot->totalNonRotorMass() * (T)9.81), _contact_pt(pt), _dim_U(6)
{
    idx_Fz_ = dimContact - 1;// 2, because normally (tau_x,y,z , linear_x,y,z)
    Fr_des_ = DVec<T>::Zero(dimContact);
    Jc_ = DMat<T>(dimContact, 18);          // (3, 18)
    JcDotQdot_ = DVec<T>::Zero(dimContact); // (3,)
    Uf_ = DMat<T>::Zero(_dim_U, dimContact);// (6, 3)

    T mu(0.4);

    Uf_(0, 2) = 1.;

    Uf_(1, 0) = 1.;
    Uf_(1, 2) = mu;
    Uf_(2, 0) = -1.;
    Uf_(2, 2) = mu;

    Uf_(3, 1) = 1.;
    Uf_(3, 2) = mu;
    Uf_(4, 1) = -1.;
    Uf_(4, 2) = mu;

    // Upper bound of normal force
    Uf_(5, 2) = -1.;
}

template<typename T>
SingleContact<T>::~SingleContact()
{}

template<typename T>
bool SingleContact<T>::UpdateContactSpec()
{
    _UpdateJc();
    _UpdateJcDotQdot();
    _UpdateUf();
    _UpdateInequalityVector();
    b_set_contact_ = true;
    return true;
}

template<typename T>
bool SingleContact<T>::_UpdateJc()
{
    Jc_ = robot_sys_->_Jc[_contact_pt];// (3, 18)
    return true;
}

template<typename T>
bool SingleContact<T>::_UpdateJcDotQdot()
{
    JcDotQdot_ = robot_sys_->_Jcdqd[_contact_pt];
    return true;
}

template<typename T>
bool SingleContact<T>::_UpdateUf()
{
    return true;
}

template<typename T>
bool SingleContact<T>::_UpdateInequalityVector()
{
    ieq_vec_ = DVec<T>::Zero(_dim_U);
    ieq_vec_[5] = -_max_Fz;
    return true;
}

template class SingleContact<float>;
