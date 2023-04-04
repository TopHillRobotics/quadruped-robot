#ifndef GAZEBO_MODEL_SPAWN_H
#define GAZEBO_MODEL_SPAWN_H

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>

class GazeboSpawner {

public:

  GazeboSpawner(const std::string robot_type,
                ros::NodeHandle& nh): d_robot_type(robot_type), d_nh(nh){
    ;
  }

  bool spawn_model(std::string urdf_param);

  bool delete_model();

  void start_controllers();

  bool stop_controllers();

  void load_controllers();

  void unload_controllers();

  void set_model_position(double x, double y, double z);

  void set_model_orientation(double w, double x, double y, double z);

protected:

  static const std::vector<std::string> controller_list;

  static geometry_msgs::Pose model_pose;

private:

  std::string d_robot_type;

  ros::NodeHandle d_nh;

  void load_controller_once(std::string controller_name);

  void unload_controller_once(std::string controller_name);

};

#endif // GAZEBO_MODEL_SPAWN_H
