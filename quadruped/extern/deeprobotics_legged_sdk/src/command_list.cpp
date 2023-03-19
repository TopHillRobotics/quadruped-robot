/**
 * @file command_list.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "command_list.h"


CommandList::CommandList() {
  pthread_mutex_init(&mutex_,NULL);
  list_capacity_ = 100;
}

CommandList::~CommandList() {
  pthread_mutex_lock(&mutex_);
  while(!command_list_.empty()){
    delete (command_list_.back());
    command_list_.pop_back();
  }
  pthread_mutex_unlock(&mutex_);
  pthread_mutex_destroy(&mutex_);
}

uint32_t CommandList::set_command(Command* command){
  if(command_list_.size() >= list_capacity_) return 1;
  pthread_mutex_lock(&mutex_);

  command_list_.push_back(command);

  pthread_mutex_unlock(&mutex_);
  return 0;
}

// return command count in list
uint32_t CommandList::get_command_front(Command& command){
  size_t quantity = command_list_.size();
  if(command_list_.empty()){
    return quantity;
  }
  pthread_mutex_lock(&mutex_);

  command = *(command_list_.front());
  delete command_list_.front();
  command_list_.pop_front();

  pthread_mutex_unlock(&mutex_);
  return quantity;
}

// return user command count in list
uint32_t CommandList::get_command_back(Command& command){
  size_t quantity = command_list_.size();
  if(command_list_.empty()){
    return quantity;
  }
  pthread_mutex_lock(&mutex_);

  command = *(command_list_.back());
  delete command_list_.back();
  command_list_.pop_back();

  pthread_mutex_unlock(&mutex_);
  return quantity;
}
