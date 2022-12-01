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

#include "deeprobotics_legged_sdk/parse_cmd.h"

using namespace std;
CommandList command_list_,upload_command_list_;
double joint_data[12];
void ParseCMD::work()
{
	int count = 0;

  EthCommand c;
  CommandMessage cm;
  Command cmd_test;

  UDPServer udpServer;
  timespec test_time;

  int tfd;    //定时器描述符
  int efd;    //epoll描述符
  int fds, ret;
  uint64_t value;
  struct epoll_event ev, *evptr;
  struct itimerspec time_intv; //用来存储时间

  tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
  if(tfd == -1) {
      printf("create timer fd fail \r\n");
   }

  time_intv.it_value.tv_sec = 0; //设定2s超时
  time_intv.it_value.tv_nsec = 1000*100;
  time_intv.it_interval.tv_sec = time_intv.it_value.tv_sec;   //每隔2s超时
  time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

  printf("timer start ...\n");
  timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器
  
  efd = epoll_create1(0); //创建epoll实例
  if (efd == -1) {
      printf("create epoll fail \r\n");
      close(tfd);
  }
  
  evptr = (struct epoll_event *)calloc(1, sizeof(struct epoll_event));
  if (evptr == NULL) {
      printf("epoll event calloc fail \r\n");
      close(tfd);
      close(efd);
   }

  ev.data.fd = tfd; 
  ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
  epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中

  RobotCmd test_cmd;
  int pos_count = 0;
  int vel_count = 0;
  int tor_count = 0;
  udpServer.onRawMessageReceived = [&](const char* message, int length, string ipv4, uint16_t port) {
    clock_gettime(1,&test_time);
    memcpy(&cm,message,sizeof(cm));
    if(cm.command.type == command_type::CommandType::kMessValues){
      switch (cm.command.code){
        case JOINT_POS_CMD:
        
          break;
        case 0x0906:
          clock_gettime(1,&test_time);
          memcpy(&state_rec, cm.data_buffer, sizeof(state_rec));
          // for(int i = 0; i < 4; i++){
          //   cout << "fl  " << state_rec.motor_state.fl_leg[0].pos << "  "  << state_rec.motor_state.fl_leg[1].pos  << "  " << state_rec.motor_state.fl_leg[2].pos << endl;
          //   cout << "fr  " << state_rec.motor_state.fr_leg[0].pos << "  "  << state_rec.motor_state.fr_leg[1].pos  << "  " << state_rec.motor_state.fr_leg[2].pos << endl;
          //   cout << "hl  " << state_rec.motor_state.hl_leg[0].pos << "  "  << state_rec.motor_state.hl_leg[1].pos  << "  " << state_rec.motor_state.hl_leg[2].pos << endl;
          //   cout << "hr  " << state_rec.motor_state.hr_leg[0].pos << "  "  << state_rec.motor_state.hr_leg[1].pos  << "  " << state_rec.motor_state.hr_leg[2].pos << endl;
          //   cout << "--------------------------------------" << endl;
          // }
          // cout << "fl  " << state_rec.fl_tor[0] << "  "  << state_rec.fl_tor[1]  << "  " << state_rec.fl_tor[2] << endl;
          // cout << "fr  " << state_rec.fr_tor[0] << "  "  << state_rec.fr_tor[1]  << "  " << state_rec.fr_tor[2] << endl;
          // cout << "hl  " << state_rec.hl_tor[0] << "  "  << state_rec.hl_tor[1]  << "  " << state_rec.hl_tor[2] << endl;
          // cout << "hr  " << state_rec.hr_tor[0] << "  "  << state_rec.hr_tor[1]  << "  " << state_rec.hr_tor[2] << endl;
          // cout <<"get  imu  -------------------------- " << state_rec.tick << endl;
          break;
      default:
        break;
      }
    }else{
      // to do simple command;
    }
  };
  // Bind the server to a port.
    udpServer.Bind(43897, [](int errorCode, string errorMessage) {
      // BINDING FAILED:
    cout << errorCode << " : " << errorMessage << endl;
  });


	while (1){
    sleep(1);
	}
}

void ParseCMD::startWork()
{
  std::thread work_thread(std::bind(&ParseCMD::work, this));
	work_thread.detach();
}
RobotState& ParseCMD::get_recv(){
  return state_rec;
}
