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


#ifndef SEND_TO_ROBOT_H_
#define SEND_TO_ROBOT_H_

#include <iostream>
#include <cmath>
#include <stdint.h>
#include <array>
#include <thread>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include "command_list.h"
#include "udpsocket.hpp"
#include "robot_types.h"

#define ROBOT_IP "192.168.1.120"
#define ROBOT_PORT 43893

#define ABLE 2
#define UNABLE 1

class SendToRobot{
  private:
    RobotCmd robot_cmd;
    CommandList cmd_list;
    UDPSocket *udpSocket;
    void cmd_done(Command& command);
  public:
    void startWork();
    void work();
    void init(void);
    void set_send(RobotCmd&);
    void all_joint_back_zero(void);
    void robot_state_init(void);
    void set_send_pose(void);
    void set_cmd(uint32_t code , uint32_t value);
    void control_get(int mode){
      if(mode == UNABLE){
        RobotCmd cmd;
        for(int i = 0; i < 12; i++){
          cmd.joint_cmd[i].pos = 0.0;
          cmd.joint_cmd[i].tor = 0.0;
          cmd.joint_cmd[i].vel = 0.0;
          cmd.joint_cmd[i].kp = 0.0;
          cmd.joint_cmd[i].kd = 5.0;
        }
        set_send(cmd);
        sleep(2);
        cmd_list.set_command(new Command(0x0113, 0,0));
      }else if (mode == ABLE){
        cmd_list.set_command(new Command(0x0114, 0,0));
      }
    }// void control_get(void);
};

class Time_Tool{
  private:
    int tfd;    //定时器描述符
    int efd;    //epoll描述符
    int fds, ret;
    uint64_t value;
    struct epoll_event ev, *evptr;
    struct itimerspec time_intv; //用来存储时间
  public:
    timespec system_time;
    void time_init(int ms);
    int time_interrupt();
    double get_now_time(double start_time);
    double get_start_time();
};


#endif  // PARSE_CMD_H_
