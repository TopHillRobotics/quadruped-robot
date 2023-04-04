#include <ros/ros.h>

#include "qr_gazebo/gazebo_model_spawn.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spawn_model");
  ros::NodeHandle nh;

  const char* type_cstr = argv[1];
  std::string robot_type(type_cstr);

  GazeboSpawner manager(robot_type, nh);

  manager.set_model_position(0, 0, 0.4);
  manager.set_model_orientation(0, 0, 0, 0);

  /// "robot_description" was loaded in .launch file
  /// make sure the robot hasn't been created
  if(!manager.spawn_model("robot_description")){
    ROS_ERROR("Fail to spawn model in gazebo: %s", type_cstr);
    return 1;
  }

  printf("----------------------------------------------------------------\n");
  printf("Press Enter key to start controllers.\n");
  printf("----------------------------------------------------------------\n");

  getchar();
  manager.load_controllers();
  manager.start_controllers();

  printf("----------------------------------------------------------------\n");
  printf("Press Enter key to delete controllers and model.\n");
  printf("----------------------------------------------------------------\n");

  getchar();

  if(!manager.stop_controllers()){
    ROS_ERROR("Fail to stop controllers in gazebo: %s", type_cstr);
    return 1;
  }

  manager.unload_controllers();
  manager.delete_model();

  return 0;
}
