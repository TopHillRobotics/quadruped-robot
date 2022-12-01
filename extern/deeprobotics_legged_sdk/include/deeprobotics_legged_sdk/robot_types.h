/**
 * @file robot_types.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef ROBOT_TYPES_H_
#define ROBOT_TYPES_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>

struct ImuData{
  int32_t timestamp;
  union{
    float buffer_float[9];
    uint8_t buffer_byte[3][12];
    struct{
      float angle_roll,angle_pitch,angle_yaw;
      float angular_velocity_roll,angular_velocity_pitch,angular_velocity_yaw;
      float acc_x,acc_y,acc_z;
    };
  };
};



typedef struct{
  float pos;
  float vel;
  float tor;
  float temperature;
}MotorData;

struct RobotData{
  union{
    MotorData joint_data[12];
    struct {
      MotorData fl_leg[3];
      MotorData fr_leg[3];
      MotorData hl_leg[3];
      MotorData hr_leg[3];
    };
  };
};

typedef struct{
  float pos;
  float vel;
  float tor;
  float kp;
  float kd;
}JointCmd;

struct RobotCmd{
  union{
    JointCmd joint_cmd[12];
    struct {
      JointCmd fl_leg[3];
      JointCmd fr_leg[3];
      JointCmd hl_leg[3];
      JointCmd hr_leg[3];
    };
  };
};
typedef struct{
  uint32_t tick;
  ImuData imu;
  RobotData motor_state;
  double fl_tor[3];
  double fr_tor[3];
  double hl_tor[3];
  double hr_tor[3];

}RobotState;
#endif  // ROBOT_TYPES_H_