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


#ifndef PARSE_CMD_H_
#define PARSE_CMD_H_

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

#include "robot_types.h"
#include "udpserver.hpp"
#include "command_list.h"
#define LOCAL_PORT 43897

#define JOINT_POS_CMD 0x0902
#define JOINT_VEL_CMD 0x0903
#define JOINT_TOR_CMD 0x0904

class ParseCMD{
  private:
    RobotState state_rec;
  public:
    void startWork();
    void work();
    RobotState& get_recv();
};


#endif  // PARSE_CMD_H_