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

#include "parse_cmd.h"

using namespace std;
CommandList command_list_,upload_command_list_;
double joint_data[12];
void ParseCMD::work()
{
	// int count = 0;

  EthCommand c;
  CommandMessage cm;
  Command cmd_test;
  ControlCommands user_command_set_;

  UDPServer udpServer;
  timespec test_time;

  int tfd;    //定时器描述符
  int efd;    //epoll描述符
  int fds, ret;
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
  // double dt_time = 0.0;
  ev.data.fd = tfd; 
  ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
  epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中

  RobotCmd test_cmd;
  int pos_count = 0;
  int vel_count = 0;
  int tor_count = 0;
  uint32_t value = 0;
  uint32_t last_value = 0;
  double last_time = 0.0;
  double now_time = 0.0;
  double dt_time = 0.0;
  double max_time = 0.0;
  double min_time = 1000000.0;
  double dt_sum = 0.0;
  double ave_time = 0.0;
  uint32_t drop_count = 0;
  uint32_t count = 0;
  uint32_t time_out_count = 0;
  udpServer.onRawMessageReceived = [&](const char* message, int length, string ipv4, uint16_t port) {
    clock_gettime(1,&test_time);
  // if(manager_->command_list_.get_command_front(c)>0){
  //   HeatData* heat_data = 0;
    memcpy(&cm,message,sizeof(cm));
    Command nc(cm.command.code,cm.command.paramters_size, cm.data_buffer);
    if(cm.command.type == command_type::CommandType::kMessValues){
      switch (cm.command.code){
        case JOINT_POS_CMD:
        
          break;
        case 0x0906:
          clock_gettime(1,&test_time);
          memcpy(&state_rec, cm.data_buffer, sizeof(state_rec));
          break;
          case 600:
            memcpy((void*) &(user_command_set_.data_send_pose) , nc.get_command_parameters(),sizeof(user_command_set_.data_send_pose)); 
            // cout << "get data pose" << endl;
            timespec time_now;
            clock_gettime(0,&time_now);
            cout << time_now.tv_sec << "." << time_now.tv_nsec << endl;
            // dt_time =  ((double)time_now.tv_sec + (double)time_now.tv_nsec/1e9 - 
            //     (double)user_command_set_.data_send_pose.stamp_vision_now.tv_sec - 
            //     (double)user_command_set_.data_send_pose.stamp_vision_now.tv_nsec/1e9)*1e3;
            // if(dt_time > 7){
            // cout << "dt_time " << dt_time << endl;
            
            // }
          break;
          
      default:
        break;
      }
    }else{
       switch (cm.command.code){
        case 0x21040001:
            value = (uint32_t)nc.get_command_value(); 
            timespec time_now;
            clock_gettime(0,&time_now);
            if(value != 0 && value != last_value + 1){
              cout << "error  drop" << endl;
              drop_count++;
            }
            now_time = (double)((double)time_now.tv_sec + (double)time_now.tv_nsec/1e9)*1e3;
            dt_time = now_time - last_time;
            last_value = value;
            last_time = now_time;
            if(value == 0){
              break;
            }
            count ++;
            dt_sum += dt_time;
            ave_time = dt_sum/count;
            if(dt_time > max_time){
              max_time = dt_time;
            }
            if(dt_time < min_time ){
              min_time = dt_time;
            }
            if(dt_time > 250){
              time_out_count ++;
            }
            printf("index  %6d , dt_time  %04.3f, max %+04.3f, min %+04.3f, ave %+04.3f, timeout count  %6d drop count %2d", value, dt_time, max_time, min_time, ave_time, time_out_count,drop_count);
            cout << endl;
            // cout << "index " << count <<"  dt_time " << dt_time << " max  " << max_time  << "  min  " << min_time  << " ave " << ave_time  << " drop count " << drop_count << endl;
            break;
        break;
       }
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
