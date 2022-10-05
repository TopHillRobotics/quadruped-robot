#ifndef QR_MIT_MPC_STANCE_LEG_CONTROLLER_H
#define QR_MIT_MPC_STANCE_LEG_CONTROLLER_H

#include "qr_mit_mpc_interface.h"
#include "qr_sparse_cmpc.h"
#include "controller/qr_torque_stance_leg_controller.h"

struct CMPC_Jump {
    static constexpr int START_SEG = 6;
    static constexpr int END_SEG = 0;
    static constexpr int END_COUNT = 2;
    bool jump_pending = false;
    bool jump_in_progress = false;
    bool pressed = false;
    int seen_end_count = 0;
    int last_seg_seen = 0;
    int jump_wait_counter = 0;

    void debug(int seg)
    {
        (void)seg;
        //printf("[%d] pending %d running %d\n", seg, jump_pending, jump_in_progress);
    }

    void trigger_pressed(int seg, bool trigger)
    {
        (void)seg;
        if (!pressed && trigger) {
            if (!jump_pending && !jump_in_progress) {
                jump_pending = true;
                //printf("jump pending @ %d\n", seg);
            }
        }
        pressed = trigger;
    }

    bool should_jump(int seg)
    {
        debug(seg);

        if (jump_pending && seg == START_SEG) {
            jump_pending = false;
            jump_in_progress = true;
            //printf("jump begin @ %d\n", seg);
            seen_end_count = 0;
            last_seg_seen = seg;
            return true;
        }

        if (jump_in_progress) {
            if (seg == END_SEG && seg != last_seg_seen) {
                seen_end_count++;
                if (seen_end_count == END_COUNT) {
                    seen_end_count = 0;
                    jump_in_progress = false;
                    //printf("jump end @ %d\n", seg);
                    last_seg_seen = seg;
                    return false;
                }
            }
            last_seg_seen = seg;
            return true;
        }

        last_seg_seen = seg;
        return false;
    }
};

/*!
 * Timer for measuring time elapsed with clock_monotonic
 */
class MITTimer {
 public:

  /*!
   * Construct and start timer
   */
  explicit MITTimer() { start(); }

  /*!
   * Start the timer
   */
  void start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

  /*!
   * Get milliseconds elapsed
   */
  double getMs() { return (double)getNs() / 1.e6; }

  /*!
   * Get nanoseconds elapsed
   */
  int64_t getNs() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
           1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  /*!
   * Get seconds elapsed
   */
  double getSeconds() { return (double)getNs() / 1.e9; }

  struct timespec _startTime;
};

class qrMITConvexMPCStanceLegController: public qrStanceLegController {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    qrMITConvexMPCStanceLegController(
        qrRobot *robot,
        qrGaitGenerator *gaitGenerator,
        qrRobotEstimator *robotVelocityEstimator,
        qrGroundSurfaceEstimator *groundEstimatorIn,
        qrComPlanner *comPlanner,
        qrFootholdPlanner *footholdPlanner,
        Eigen::Matrix<float, 3, 1> desired_speed,
        float desiredTwistingSpeed,
        float desiredBodyHeight,
        int numLegs,
        std::string configFilepath,
        std::vector<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45});

    virtual void Reset(float t);

    void run(std::map<int, qrMotorCommand>& legCommand, int gaitType, int robotMode=0);

    virtual std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

    void UpdateDesCommand();

    bool currently_jumping = false;
    Vec4<float> contact_state;
    Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor> _mpcTable;

    /**
     * @brief update linear velocity and angular velocity of controllers
     * @param linSpeed: linear velocity
     * @param angSpeed: yaw twist velocity
     */
    void UpdateControlParameters(const Eigen::Vector3f& linSpeed, const float& angSpeed) override;

    void initStateDes();

private:
    void _SetupCommand();

    int vaildTrajNum=0;

    float _yaw_turn_rate = 0.;
    float _yaw_des;
    float _yaw_des_true = 0.0;

    float _roll_des;
    float _pitch_des;

    float _x_vel_des = 0.;
    float _y_vel_des = 0.;

    // High speed running
    float _body_height = 0.28;
    float _body_height_running = 0.29;
    float _body_height_jumping = 0.36;

    void recompute_timing(int iterations_per_mpc);
    void updateMPCIfNeeded(qrRobot *_quadruped, bool omniMode);
    void solveDenseMPC(qrRobot *_quadruped);
    void solveSparseMPC(qrRobot *_quadruped);
    void initSparseMPC();

    int iterationsInaMPC; // 15
    const int horizonLength;    //5, 10
    int numHorizonL= 1;
    int default_iterations_in_mpc;
    float dt;                 // 0.002
    float dtMPC;              // 0.03
    int iterationCounter = 0;
    Eigen::Matrix<float,3,4> f_ff; // the force that leg excerts to the ground.
    Eigen::Matrix<float,3,4> f;
    Vec4<float> swingTimes;
    Mat3<float> Kp, Kd, Kp_stance, Kd_stance, Kp1;
    bool firstRun = true;
    bool firstSwing[4];//true
    float swingTimeRemaining[4];
    float stand_traj[6];
    int current_gait;
    int gaitNumber;

    Vec3<float> world_position_desired;
    Vec3<float> rpy_int;
    Vec3<float> rpy_comp;
    float x_comp_integral = 0;

    Eigen::Matrix<float, 3, 4> pFoot;
    Vec3<float> v_des_world;
    Eigen::Matrix<float, 3, 4> result;
    float trajAll[20 * 36];
    bool myflags = false;
    float Q[12];

    CMPC_Jump jump_state;

    vectorAligned<Vec12<double>> _sparseTrajectory;
    SparseCMPC _sparseCMPC;

    bool useWBC = false;

    Vec12<float> stateDes;
    Vec12<float> stateCur;

    float moveBasePhase = 1.f;
    int N=0;
    Eigen::Matrix<bool, 4, 1> contacts;
    // Eigen::Matrix<float, 13, 1> XWeight;
};

#endif // QR_MIT_MPC_STANCE_LEG_CONTROLLER_H
