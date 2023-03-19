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


#pragma pack(4)  
 struct DataSendPose  
  {  
	  float pos_x;  
	  float pos_y;  
	  float pos_z;  
	  float ori_x;  
	  float ori_y;  
	  float ori_z;  
	  float ori_w;  
	  timespec stamp;  
	  timespec stamp_vision_now;
	};  
#pragma pack()  

typedef struct ControlCommands
{
	// basic
	bool   is_stand;				///< up/down 
	bool   switch_to_force_model;	///< force model 
	bool   turn_over_flag;			///< 
	bool   raise_head_flag;
	bool   start_action_flag;
	bool   change_swing_leg_flag;	///< 
	bool   start_pronk_flag;
	bool   is_vision_mode;       	///< true: vision; false: Manual
	int    gait;                 	///< switch: 0: trot; 1: stair; 2: grasslandTrot; 3:trotRun; ......
	int    jump_type;    		 	///< single-click: false, double-click: true
	int    velocity_control_y;   	///< hold LT: 1, hold RT: -1, loosen: 0
	double left_axis_x;         	///<                  value: between -1 and 1, resolution: 1/32767
	double left_axis_y;       	  	///< Forward speed.  value: between -1 and 1, resolution: 1/32767
	double right_axis_x;       		///<                  value: between -1 and 1, resolution: 1/32767
	double right_axis_y;       		///< Turn speed.      value: between -1 and 1, resolution: 1/32767
 	bool is_twist_body_mode;
	int lower_robot_velocity_flag;
	int lower_robot_body_height_flag;

	// state  true : error  false : normal
	bool is_gyro_data_lose;
	bool is_drive_heat_warning;
	bool is_wifi_lose;
	bool is_joystick_disconnect;
	bool is_heat_data_lose;
	bool is_slam_lose;
	bool is_battary_low;
	int  is_driver_error;
	int  motor_temperature;
	int  driver_temperature;

	// vision or more expand
	int    gesture_value;
	bool   vision_is_reach_charge_area;
	double vision_turning_angle;
	bool retry_charge;
	bool charge_over;
	int voice_value;
	int soft_emergency_cnt;
	double slam_yaw_correct;
	double slam_yaw_vel;
	double slam_x_direction_vel;
	double slam_y_direction_vel;

	double target_pos_x;
	double target_pos_y;
	double target_theta;

	double current_lidar_pos_x;
	double current_lidar_pos_y;
	double current_lidar_theta;
	int navigation_task_num;

	// 旧绝影自主充电使用
	double vision_forward_vel;
	double vision_side_vel;

	double vision_location_x;
	double vision_location_y;
	double vision_location_theta;

	double lidar_location_x;
	double lidar_location_y;
	double lidar_location_theta;
	double roll_command, pitch_command, yaw_command, x_command, y_command, height_command; 
	int32_t adjust_rate;
	bool is_normal_walk;  ///< true: 张开腿行走 

	union{  
	  struct{  
		  float robot_pos_x;  
		  float robot_pos_y;  
		  float robot_pos_z;  
		  float robot_ori_x;  
		  float robot_ori_y;  
		  float robot_ori_z;  
		  float robot_ori_w;  
	  };  
	  struct{  
		  float robot_pos[3];  
		  float robot_ori[4];  
	  }; 
	  float grid_map_posture[9];  
	  //timespec time;  
	  float tv_sec;float tv_nsec;  
  	};  
  	DataSendPose data_send_pose; 
		int grid_map_normal; 
		int send_pose_normal;  

} ControlCommands;




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