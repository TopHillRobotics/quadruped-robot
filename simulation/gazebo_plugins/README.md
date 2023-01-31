<p align="center"> 
    <img src=".image/logo.png" alt="Gazebo Awesome Plugins" width="100%">
</p>

--------------------

GAP is a set of tools written in C++ for [Gazebo] which provide an interface to interact programatically with the simulator.
We built GAP to support our research in **Domain Randomization**.
Unlike most similar software, these tools **do not** strictly depend on [ROS].
They include:

1. **[NEW]** [Domain Randomization plugin], a multi-purpose plugin to change physical properties of models, as well as visual appearence.

1. [Camera Utils plugin], to control camera objects, namely moving the camera and saving rendered frames at specific instants.

1. [Visual Utils plugin], to control the visual appearance of an object during simulation, including changing the Visual object's pose, material and scale.

1. [World Utils plugin], that allows you to spawn models either by a uri reference or directly with an sdf string.

This project was originally conceived so we could develop a [scene generator] in Gazebo, employing domain randomisation in an attempt to bridge the **reality gap** between real life images and synthetically generated frames.

Check out [`tf-object-detection`](https://github.com/jsbruglie/tf-shape-detection), in which we trained a *state--of--the--art* deep CNN using this synthetic dataset.

### Examples

Check out the [examples] and see what you can achieve with these plugins.

<center>
<table>
  <tr>
  <td width="35%"><img src=.image/shadowhand.gif height="400px" alt="shadowhand"></td>
  <td width="55%"><img src=.image/demo.gif height="400px" alt="scene_example"></td>
  </tr>
  </table>
</center>

### Documentation

Take a look at the automatic [documentation] for file and class description.


### Dependencies

The code has been tested in Gazebo 9.0.0 from the official stable repository as well as built from source 9.4.1 and running on Ubuntu 16.04.5 and 18.04.

Gazebo internal message passing relies on Protobuf, which is why the compiler needs to be installed in order
to generate the tools' custom messages.
Eigen 3 is required for [scene_example].

```bash
sudo apt install protobuf-compiler  # Required
sudo apt install libeigen3-dev      # Required for scene_example
```
For custom texture generation, we have developed a [pattern generation tool], which can randomly generate a high number of 4 different types of textures and produces materials in a format Gazebo can recognise.

### Compilation

Clone the repository to your workspace directory and build from source.

```bash
cd ~/workspace/gap/ &&
mkdir build && cd build && cmake ../ && make -j8
```

Alternatively you can build each plugin/tool individually in a similar fashion.

### Initialization

Make sure you properly initialise the required environment variables.
We provide a simple script for this:

```bash
cd ~/workspace/gap &&
source setup.sh
```

### Reference

These tools were developed to further our research regarding domain randomisation.
We include a [published conference paper] and its reference for citation purposes.

```
@inproceedings{borrego2018,
  author = {J. Borrego and R. Figueiredo and A. Dehban and P. Moreno and A. Bernardino and J. Santos-Victor},
  booktitle = {2018 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)},
  title = {A generic visual perception domain randomisation framework for Gazebo},
  year = {2018},
  pages = {237-242},
  keywords = {Machine learning;Neural networks;Object detection;Proposals;Robots;Task analysis;Training},
  doi = {10.1109/ICARSC.2018.8374189},
  month = {April}
}

```

### Disclaimer

This project is not directly affiliated with Gazebo.

<!-- Links -->

[Gazebo]: http://gazebosim.org/
[ROS]: http://www.ros.org/
[Domain Randomization plugin]: plugins/domain_randomization
[Camera Utils plugin]: plugins/camera_utils
[Visual Utils plugin]: plugins/visual_utils
[World Utils plugin]: plugins/world_utils
[examples]: examples
[scene generator]: examples/scene_example
[scene_example]: examples/scene_example
[documentation]: http://web.tecnico.ulisboa.pt/joao.borrego/gap/
[pattern generation tool]: https://github.com/ruipimentelfigueiredo/pattern-generation-lib
[published conference paper]: http://vislab.isr.ist.utl.pt/wp-content/uploads/2018/04/jborrego-icarsc2018.pdf
