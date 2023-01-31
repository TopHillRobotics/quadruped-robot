#ifndef _CONVEXMPCLOCOMOTION_H
#define _CONVEXMPCLOCOMOTION_H

#include "./mit_mpc_interface.h"
#include "fsm/control_fsm_data.hpp"
#include "controllers/balance_controller/torque_stance_leg_controller.h"
#include "controllers/wbc/wbc_locomotion_controller.hpp"

namespace Quadruped {
    /** @brief  
 * A modified MIT version convex mpc impelement
 */
    class MITConvexMPCStanceLegController : public TorqueStanceLegController {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        MITConvexMPCStanceLegController(Robot *robot,
                                        GaitGenerator *gaitGenerator,
                                        StateEstimatorContainer<float> *stateEstimators,
                                        ComAdjuster *comAdjuster,
                                        PosePlanner *posePlanner,
                                        FootholdPlanner *footholdPlanner,
                                        UserParameters &userParameters,
                                        std::string configFilepath);

        virtual void Reset(float t);
        virtual std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();
        void Run(std::map<int, MotorCommand> &legCommand, int gaitType, int robotMode = 0);
    private:
        void _SetupCommand();
        void _UpdateMPC(Robot *_quadruped);
        void _SolveDenseMPC(Robot *_quadruped);

        float _yaw_turn_rate = 0.0;
        float _yaw_des_true = 0.0;
        float _roll_des;
        float _pitch_des;
        float _x_vel_des = 0.;
        float _y_vel_des = 0.;
        float _body_height;
        Vec3<float> rpy_int;
        Vec3<float> rpy_comp;

        int iterationsInaMPC;
        const int horizonLength;
        int numHorizonL = 1;
        int defaultIterationsInMpc;
        float dt;
        float dtMPC;
        unsigned long long iterationCounter = 0;

        Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> _mpcTable;
        Eigen::Matrix<float, 3, 4> f_ff;// the force that leg excerts to the ground.
        Eigen::Matrix<float, 3, 4> f;

        Vec3<float> posDesiredinWorld;
        Vec3<float> vDesWorld;
        Eigen::Matrix<float, 3, 4> pFoot;
        float trajAll[20 * 36];
        float Q[12];

        bool mpcUpdated = false;
        bool useWBC = false;
    };

}// namespace Quadruped

#endif//CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
