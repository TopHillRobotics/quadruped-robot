#ifndef WBC_SINGLE_CONTACT
#define WBC_SINGLE_CONTACT

#include "dynamics/floating_base_model.hpp"

template<typename T>
class SingleContact {
public:
    SingleContact(FloatingBaseModel<T> *robot, int contact_pt);
    virtual ~SingleContact();
    
    void setMaxFz(T max_fz) { _max_Fz = max_fz; }
    size_t getDim() const { return dimContact; }
    size_t getDimRFConstraint() const { return Uf_.rows(); }
    size_t getFzIndex() const { return idx_Fz_; }

    void getContactJacobian(DMat<T>& Jc) { Jc = Jc_; }
    void getJcDotQdot(DVec<T>& JcDotQdot) { JcDotQdot = JcDotQdot_; }
    void UnsetContact() { b_set_contact_ = false; }

    void getRFConstraintMtx(DMat<T>& Uf) { Uf = Uf_; }
    void getRFConstraintVec(DVec<T>& ieq_vec) { ieq_vec = ieq_vec_; }
    const DVec<T>& getRFDesired() { return Fr_des_; }
    void setRFDesired(const DVec<T>& Fr_des) { Fr_des_ = Fr_des; }
    
    bool UpdateContactSpec();

protected:
    bool _UpdateJc();
    bool _UpdateJcDotQdot();
    bool _UpdateUf();
    bool _UpdateInequalityVector();
    
    FloatingBaseModel<T> *robot_sys_;

    T _max_Fz;
    int _contact_pt;
    int _dim_U;

    int idx_Fz_;
    DMat<T> Uf_;
    DVec<T> ieq_vec_;
    DVec<T> Fr_des_;

    DMat<T> Jc_;
    DVec<T> JcDotQdot_;
    size_t dimContact;
    bool b_set_contact_;
};

#include "controllers/wbc/single_contact.hxx"
#endif
