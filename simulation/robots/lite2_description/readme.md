meshes文件夹下为机器人各部位的3D模型。

urdf文件夹下为机器人的描述文件，其中：

- Lite2A_gazebo.urdf里的小腿+足底碰撞模型采用连杆+足底小球的方式，支持在gazebo，rviz，raisim等平台使用
- Lite2A.urdf里的小腿+足底碰撞模型采用直接读取SHANK.obj的方式，由于gazebo读取的问题，不支持在gazebo使用，支持在rviz，raisim等平台使用

具体效果示意图片已单独发送。

可以在采用添加模型库或者roslaunch的方法将机器人模型导入Gazebo中，roslaunch采用的launch文件里导入urdf部分代码示例如下：

```
<!-- 导入urdf文件，第一个参数"robot_function_pkg"修改为功能包名，第二个参数'$(find robot_function_pkg)/urdf/urdf/test_robot.urdf'修改为urdf文件的路径 -->
<param name="robot_function_pkg" command="cat '$(find robot_function_pkg)/urdf/urdf/test_robot.urdf'" />

<!-- 导入spawn_urdf节点，使Gazebo能够识别urdf文件。需要修改的参数为功能包的路径"/robot_function_pkg" -->
<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param /robot_function_pkg -urdf -model robot" />
```



需要注意的是urdf文件里每个<link>下的<visual>里的filename = “package://**robot_function_pkg/meshes/meshes/xxx.dae**”部分，需要改成自己的ros包或对应的路径，防止读取出错。

运行gazebo或rviz时，需要提前在当前终端source，否则可能会出现找不到对应文件的问题。

同时附上两种urdf转化完的sdf格式供使用。

