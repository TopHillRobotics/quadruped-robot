// The MIT License

#ifndef QR_SINGLE_CONTACT_HPP
#define QR_SINGLE_CONTACT_HPP

#include "dynamics/qr_floating_base_model.hpp"
#include "controller/wbc/qr_contact_spec.hpp"

template <typename T>
class qrSingleContact : public qrContactSpec<T> {
public:
    qrSingleContact(FloatingBaseModel<T>* robot, int contact_pt);
    virtual ~qrSingleContact();

    void setMaxFz(T max_fz) { _max_Fz = max_fz; }

protected:
    T _max_Fz;
    int _contact_pt;
    int _dim_U;

    virtual bool _UpdateJc();
    virtual bool _UpdateJcDotQdot();
    virtual bool _UpdateUf();
    virtual bool _UpdateInequalityVector();

    FloatingBaseModel<T>* robot_sys_;
};

#endif //QR_SINGLE_CONTACT_HPP