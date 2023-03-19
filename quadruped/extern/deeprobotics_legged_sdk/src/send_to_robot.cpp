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


#include "send_to_robot.h"

using namespace std;
void SendToRobot::init(void){
  // const string IP = "192.168.41.1";
  const string IP = "192.168.1.120";
  const uint16_t PORT = 43893;
  udpSocket = new UDPSocket(true);
  udpSocket->Connect(IP, PORT);

}
void SendToRobot::cmd_done(Command& command_temp){
  // Command command_temp;
  CommandMessage command_message={{0}};
  uint32_t message_size = 0;
    command_message.command.code = command_temp.get_command_code().to_ulong();
      
      if(command_temp.get_command_parameters()==0){
        // 普通命令
        command_message.command.type = command_type::CommandType::kSingleValue;
        command_message.command.value = command_temp.get_command_value();
        message_size = sizeof(command_message.command);
      }
      else{
        // 带大量数据的命令
        command_message.command.type = command_type::CommandType::kMessValues;
        command_message.command.paramters_size = command_temp.get_command_parameters_size();
        if(command_message.command.paramters_size > kCommandDataBufferSize){
          std::cout <<"[Error E_Speaker] The message of over load !"<< std::endl;
          return;
        }
        // std::cout << "send   " << count++ <<std:: endl;
      }
        // 复制数据并释放Command中的内容
        memcpy(command_message.data_buffer,
              command_temp.get_command_parameters(),
              command_message.command.paramters_size);
        delete [](char*)command_temp.get_command_parameters();
        
        message_size = sizeof(command_message.command) +
                                      command_message.command.paramters_size;
        char buffer[4096];
        memcpy(buffer,&command_message,message_size);                              
        udpSocket->Send(buffer,message_size);
}

void SendToRobot::work(){

}

void SendToRobot::startWork(){
  std::thread work_thread(std::bind(&SendToRobot::work, this));
	work_thread.detach();
}

void SendToRobot::set_send(RobotCmd& cmd){

  timespec time_sdk;
  static float i = 0.00;
  clock_gettime(0, &time_sdk);
  size_t cmd_size = sizeof(cmd);
  char *buffer = new char[cmd_size];
  memcpy(buffer, &cmd, cmd_size);
  Command command_temp(0x0111,sizeof(cmd), buffer);
  cmd_done(command_temp);

}
void SendToRobot::set_send_pose(void){
  size_t cmd_size = sizeof(DataSendPose);
  char *buffer = new char[cmd_size];
  // memcpy(buffer, &cmd, cmd_size);
  // cmd_done(new Command(600,sizeof(DataSendPose), buffer));
    Command command_temp(600,sizeof(DataSendPose), buffer);
  cmd_done(command_temp);
}

void SendToRobot::all_joint_back_zero(){
   Command command_temp(0x31010C05,0, 0);
  cmd_done(command_temp);
}
void SendToRobot::robot_state_init(){
  Command command_temp(0x31010C05,0, 0);
  cmd_done(command_temp);
  usleep(1000*1000*7);
  Command command_temp_1(0x0114,0,0);

  cmd_done(command_temp_1);//PS：超过5ms，未发数据set_send(cmd)，会失去控制权，要重新发送获取控制权
}
void SendToRobot::set_cmd(uint32_t code , uint32_t value){
    Command command_temp(code, value);
  cmd_done(command_temp);
}


void Time_Tool::time_init(int ms){
  tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
  if(tfd == -1) {
      printf("create timer fd fail \r\n");
   }
  time_intv.it_value.tv_sec = 0; //设定2s超时
  time_intv.it_value.tv_nsec = 1000*1000*ms;
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
}

int Time_Tool::time_interrupt(){
  fds = epoll_wait(efd, evptr, 1, -1);    //阻塞监听，直到有事件发生
    if(evptr[0].events & EPOLLIN){   
    ret = read(evptr->data.fd, &value, sizeof(uint64_t));
      if (ret == -1) {
        printf("read return 1 -1, errno :%d \r\n", errno);
        return 1;
      }            
    }
  return 0;
}

double Time_Tool::get_now_time(double start_time){
  clock_gettime(1,&system_time);  
  return system_time.tv_sec + system_time.tv_nsec/1e9 - start_time;
}

double Time_Tool::get_start_time(){
  clock_gettime(1,&system_time);
  return system_time.tv_sec + system_time.tv_nsec/1e9;
}

