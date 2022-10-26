// The MIT License

#ifndef QR_TASK_HPP
#define QR_TASK_HPP

#include "common/qr_cTypes.h"
#include "common/qr_se3.h"
#include "common/qr_algebra.h"
#include "dynamics/qr_floating_base_model.hpp"

#define TK qrTask<T>

template <typename T>
class qrTask {
public:
    /**
     * @brief 
     * @param dim
     */
    qrTask(size_t dim)
        : b_set_task_(false),
        dim_task_(dim),
        op_cmd_(dim),
        pos_err_(dim),
        vel_des_(dim),
        acc_des_(dim) {}

    virtual ~qrTask() {}

    /**
     * @brief 
     * @param op_cmd
     */
    void getCommand(DVec<T>& op_cmd) { op_cmd = op_cmd_; }

    /**
     * @brief 
     * @param Jt
     */
    void getTaskJacobian(DMat<T>& Jt) { Jt = Jt_; }
    
    /**
     * @brief 
     * @param JtDotQdot
     */
    void getTaskJacobianDotQdot(DVec<T>& JtDotQdot) { JtDotQdot = JtDotQdot_; }

    /**
     * @brief 
     * @param pos_des
     * @param vel_des
     * @param acc_des
     */
    bool UpdateTask(const void* pos_des, const DVec<T>& vel_des,
                    const DVec<T>& acc_des) {
        _UpdateTaskJacobian();
        _UpdateTaskJDotQdot();
        _UpdateCommand(pos_des, vel_des, acc_des);
        _AdditionalUpdate();
        b_set_task_ = true;
        return true;
    }

    /**
     * @brief 
     */
    bool IsTaskSet() { return b_set_task_; }
    
    /**
     * @brief 
     */
    size_t getDim() { return dim_task_; }
    
    /**
     * @brief 
     */
    void UnsetTask() { b_set_task_ = false; }

    /**
     * @brief Get the error of position
     */
    const DVec<T>& getPosError() { return pos_err_; }
    
    /**
     * @brief Get the desired velocity
     */
    const DVec<T>& getDesVel() { return vel_des_; }
    
    /**
     * @brief Get the desired acceleration
     */
    const DVec<T>& getDesAcc() { return acc_des_; }

protected:

    /**
     * @brief Update op_cmd_
     */
    virtual bool _UpdateCommand(const void* pos_des, const DVec<T>& vel_des,
                                const DVec<T>& acc_des) = 0;

    /**
     * @brief Update Jt_
     */
    virtual bool _UpdateTaskJacobian() = 0;
    
    
    /**
     * @brief Update JtDotQdot_ 
     */ 
    virtual bool _UpdateTaskJDotQdot() = 0;

    /**
     * @brief Additional Update (defined in child classes)
     */ 
    virtual bool _AdditionalUpdate() = 0;

    bool b_set_task_;
    size_t dim_task_;

    DVec<T> op_cmd_;
    DVec<T> JtDotQdot_;
    DMat<T> Jt_;

    DVec<T> pos_err_;
    DVec<T> vel_des_;
    DVec<T> acc_des_;
    int dim_config = 18;
};

#endif // QR_TASK_HPP