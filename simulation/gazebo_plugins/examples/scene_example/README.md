### Random scene generation

This example intends to create a large dataset of randomized scenes of up to 10 objects, namely boxes, spheres and cylinders, collecting both fully simulated Full-HD images and each object's classification and bounding box.

### Prerequisites

#### Custom materials

Generate custom textures and have Gazebo load them at launch.
Gazebo expects custom scripts to be present in `media/materials` with the material descriptions in `media/materials/scripts` in a `.material` file, and it assumes the texture images mentioned in these files are in `media/materials/textures/`.

You can setup your own `media` directory in any location provided you export it to `GAZEBO_RESOURCE_PATH`:

```bash
# e.g. assuming you have a `/DATA/media` directory
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/DATA/
```

The VisualUtils plugin is designed to randomly pick an available material in a round-robin fashion provided its name matches a given prefix (e.g Plugin/flat or Plugin/perlin).
We provide a tool for [random texture generation].

You may want to concatenate each resulting `.material` script into a single one, similarly to [plugin.material].

### Running

Open up two terminals in the root directory of the repository.
On terminal 1 run gazebo server:
```bash
cd ~/workspace/gap/ &&
source setup.sh &&
gzserver worlds/spawner.world
```

On terminal 2 run the example client:
```bash
cd ~/workspace/gap/ &&
./build/bin/scene_example -s 200 -d ./train/SHAPES2018/dataset/ -i ./train/SHAPES2018/images/ 
```

This should generate a dataset with 200 images, spread across two subdirectories 000 and 100.
This is due to performance concerns.
You can use `scene_example --help` to obtain an explanation of each command-line argument.

### Debugging dataset output

We provide a [debugging tool] written in Python 3, which relies on Tkinter and Pillow to create the GUI, and shows the resulting dataset, one image at a time.
To install Tkinter and Pillow in Ubuntu systems run
``` bash
sudo apt install python3-tk &&
sudo pip3 install Pillow
```

### Example output

<p align="center"> 
    <img src=https://github.com/jsbruglie/gap/blob/dev/.image/scene_example.png>
</p>

[plugin.material]: plugin.material
[random texture generation]: https://github.com/ruipimentelfigueiredo/pattern-generation-lib
[debugging tool]: https://github.com/jsbruglie/gap/blob/dev/scripts/scene_checker.py
