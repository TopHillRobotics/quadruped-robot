^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openai_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fixed issue that asked dependencies of other RobotEnvs and Tasks that we are not using
* Added some changes to dependencies
* Added all dependencies for the package
* Removed dependencies of theconstruct
* Added rewards messages to package
* Corrected error of robot reset
* Removed logs
* Fixed some issues
* Added changes for shadow robot
* Created TaskEnv not tested
* Added done method
* Added actions in Task Env for UR5
* Added Basic version of the RobotEnv for UR5
* Removed logs to debug
* Added reset movement to guarantee good start
* Added camera sensors
* Finished untested version of task
* Added done method
* Added action conversion Sawyer
* Added first simple version of Sawyer Robot Env
* Adding movement functions for Sawyer
* Adding movement system for Sawyer
* Added sawyer env barebones
* Corrected missing parameter
* Fixed Bugs
* Added TaskEnv
* Added done method
* Adding Task Env
* Added Wamv Robot Env
* Added minor structure changes
* Added barebones of VMRC env
* Moved to New Clearer Structure
* Added first version of the StayUp Hopper Env
* Debuging code
* Finished first round in TaskEnv for hopper
* Added observations space
* Porting all the functions from old system learning to new openai_ros
* Fixed superfluous line in parrot en and created the first version for hopper
* Added barebones of Hopper and some minor mods to drone
* Made TaskEnv work replacing dynamic action check with hardcoded wait
* Started to test the Task for drone
* Added Version one of the DroneTaskEnv
* Created the done method and its auxiliary methods
* Added New actions
* Corrected bug that broke the openai ros
* Removed file copied where it shoulndt and added parrot to init
* Added landing and takeoff sequences
* Corrected some Errors
* Adding task goto
* Added RobotEnv
* Adding Parrot DRone Env
* Merge branch 'master' into environments-documentation-mig
* Changed reset to world in Turtlebot2 because cameras took for ever to restart in the other methos
* Merged in environments-documentation-mig (pull request #3)
  Environments documentation mig
  Approved-by: RDaneelOlivaw <duckfrost@gmail.com>
* Added changes
* Removed the ACtion stop
* Added correction for the moving system due to inexactitudes of the steering system by friction
* Added some corrections for the reset world
* Corrected Imu Max value too low
* Added new done but still bugs in movement
* Adding rewards and done to make it go to a desired point
* Added world reset option for systems that sim reset breaks the tf and others
* Corrected certain errors
* Added Subscribers for all sensors
* Added Barebones of SumitXL Env
* Added minor improvements to make example work
* Adedd barebones and first version of the turtlebot3 robot env
* Minor changes
* Merged in environments-documentation-mig (pull request #2)
  Environments documentation mig
  Approved-by: RDaneelOlivaw <duckfrost@gmail.com>
* Added fix to deactivate init physiscs parameters
* Fixing bugs
* Added some changes
* Added Task for turtlebot for maze navigation
* Finished version 1 of the turtlebot2 robot env
* Added sensor readings to env
* Added barebones turtlebot2
* Added basic index
* Added first working docs for modules and the different environments
* Added gitignore to avoud uploading compiled documentation
* Added Documentation support
* Added changes for publishing only the final episode acumulated reward
* Corrected some conversion issues
* Updated code
* Updated code
* Updated code
* Updated code
* Updated code
* Added cartpole envs
* Renamed to openai_ros
* Contributors: RDAneelOlivaw, RDaneelOlivaw, aezquerro, rDaneelOlivaw
