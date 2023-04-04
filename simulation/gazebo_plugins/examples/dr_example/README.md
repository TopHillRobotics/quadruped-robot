## Domain Randomization example

This example showcases the some of the main features offered by the multi-purpose [Domain Randomization plugin], namely:

1. Change gravity 3D vector;
1. Scale spawned model;
1. Change individual link mass;
1. Change individual joint properties, such as :
    1. Joint limits (angle, effort and velocity);
    1. Joint friction;
    1. Joint damping;
1. Change joint PID controller P gain;
1. Change individual link's visual colors.

Other [documented] features **not** showcased (for now) include:

1. Change individual collision surface properties;
1. Change individual link inertia tensor;

Every implemented feature has been mentioned or explored in recent literature, namely by OpenAI in their work on [dexterous manipulation].
We provide an [Interface class] for interacting with the plugin. 

### Prerequisites

This example requires [Shadow's Dexterous Hand] to be spawned in simulation.
For this, we have used [ROS] and the Gazebo-ROS bridge software [gazebo_ros_pkgs] -- installation instructions [here].
Finally we need Shadow's official [robot description package].
The latter needs to be downloaded to the active catkin workspace.

```bash
cd catkin_ws/src && git clone https://github.com/shadow-robot/sr_common.git
```

### Running

Open up two terminals in the root directory of the repository.

On terminal 1 launch ROS and gazebo server:
```bash
cd ~/workspace/gap/ &&
source setup.sh &&
roscore & rosrun gazebo_ros gazebo worlds/domain_randomization.world --verbose
```

On terminal 2 run:
```bash
# Spawn shadow hand
rosrun gazebo_ros spawn_model -file models/shadowhand_motor.urdf -urdf -model shadowhand
# Launch domain randomization example
./build/bin/dr_example
```

<!-- Links -->

[Domain Randomization plugin]: tree/dev/plugins/domain_randomization
[documented]: http://web.tecnico.ulisboa.pt/joao.borrego/gap/classDRInterface.html
[Interface class]: /../../tree/dev/utils
[Shadow's Dexterous Hand]: https://www.shadowrobot.com/products/dexterous-hand/
[ROS]: http://www.ros.org/
[gazebo_ros_pkgs]: http://wiki.ros.org/gazebo_ros_pkgs
[here]: http://gazebosim.org/tutorials?tut=ros_installing
[robot description package]: https://github.com/shadow-robot/sr_common
[dexterous manipulation]: https://blog.openai.com/learning-dexterity/
