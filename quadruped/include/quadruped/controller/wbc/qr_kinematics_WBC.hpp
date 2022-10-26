// The MIT License

#ifndef QR_KINEMATICS_WHOLE_BODY_CONTROL_HPP
#define QR_KINEMATICS_WHOLE_BODY_CONTROL_HPP

#include "controller/wbc/qr_contact_spec.hpp"
#include "controller/wbc/qr_task.hpp"

template <typename T>
class KinWBC {
public:
    KinWBC(size_t num_qdot);
    ~KinWBC() {}

    /**
     * @brief 
     */
    bool FindConfiguration(const DVec<T>& curr_config,
                            const std::vector<qrTask<T>*>& task_list,
                            const std::vector<qrContactSpec<T>*>& contact_list,
                            DVec<T>& jpos_cmd, DVec<T>& jvel_cmd);

    DMat<T> Ainv_;

private:

    /**
     * @brief 
     */
    void _PseudoInverse(const DMat<T>& J, DMat<T>& Jinv);
    
    /**
     * @brief 
     */
    void _BuildProjectionMatrix(const DMat<T>& J, DMat<T>& N);

    double threshold_;
    size_t num_qdot_;
    size_t num_act_joint_;
    DMat<T> I_mtx;
};
#include "qr_kinematics_WBC.hxx"

#endif // QR_KINEMATICS_WHOLE_BODY_CONTROL_HPP