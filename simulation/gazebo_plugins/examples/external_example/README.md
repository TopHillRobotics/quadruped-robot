## External client

In order to build your own client you need only perform the following steps:

1. Install our tools as a CMake package, provided it was already compiled:
```bash
cd ~/workspace/gap/build
sudo make install
```
The package can easily be uninstalled by running `sudo make uninstall`

2. Add our package to your CMakeLists.txt

```CMake
find_package(gap REQUIRED)
include_directories(${GAP_INCLUDE_DIRS})
link_directories(${GAP_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAP_CXX_FLAGS}")
```

3. Finally, link our messages library to your client executable in CMakeLists.txt

```CMake
add_executable (external_client external_client.cc)
target_link_libraries(external_client
    gap_msgs
    ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
```

You can check the provided sample external client for a complete example.

### Troubleshooting

- Make sure to run Gazebo in verbose mode, by adding `--verbose` flag:
```bash
gazebo --version
```
- Gazebo instance can not load `libgap_msgs.so`. It is likely that Gazebo has no way to find this library when instancing a plugin. To fix this you can simply use the environment variable `LD_LIBRARY_PATH`. For `gap-1.5`:
```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/gap-1.5
```