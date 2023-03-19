#include "command_list.h"
#include "parse_cmd.h"
#include "robot_types.h"
// #include <iostream>
#include <time.h>
#include <string.h>
#include "Eigen/Dense"
using namespace std;
using namespace Eigen;

typedef Matrix< double, 3, 1> Vec3;

const double kDegree2Radian = 3.1415926 / 180;

class MotionExample{
  private:

  public:
    void setCmd(RobotCmd &cmd){
      
    }
    void PreStandUp(RobotData data, RobotCmd &cmd, double time);
    void StandUp(RobotData data, RobotCmd &cmd, double time);
    void SwingToAngle(Vec3 initial_angle, Vec3 goal_xyz_angle, double total_time, double run_time, double cycle_time, string side, RobotCmd &cmd);
    void CubicSpline(double init_pos, double init_vel, double goal_pos, double goal_vel, double run_time, double cycle_time, double total_time, double &sub_goal_pos, double &sub_goal_pos_next, double &sub_goal_pos_next2);
    void GetInitData(RobotData data, double time);
};

