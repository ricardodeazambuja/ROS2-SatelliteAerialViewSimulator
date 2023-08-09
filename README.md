# ros2_satellite_aerial_view_simulator
ROS 2 wrapper node for https://github.com/ricardodeazambuja/AerialViewGenerator

## TL;DR;
`$ git clone https://github.com/ricardodeazambuja/ros2_satellite_aerial_view_simulator.git`


`$ colcon build --symlink-install --packages-select ros2_satellite_aerial_view_simulator`

`$ setup install/setup.bash`

`$ ros2 launch ros2_satellite_aerial_view_simulator aerialview.launch.py`

`$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r teleop_twist_keyboard:cmd_vel:=/quadctrl/flying_sensor/ctrl_twist_sp`

By default it will publish:
* the coordinate frames (tf) between `map` and `flying_sensor`
* RGB image as if it was from a camera pointing downwards (`/carla/flying_sensor/rgb_down/image`)
* fake depth image (constant value) just because I needed it for my own stuff (`/carla/flying_sensor/depth_down/image`)

The topics are all parameters, check the [launch file](src/ros2_satellite_aerial_view_simulator/ros2_satellite_aerial_view_simulator/launch/aerialview.launch.py) and the parameters in the [main script](src/ros2_satellite_aerial_view_simulator/ros2_satellite_aerial_view_simulator/ros2_satellite_aerial_view_simulator/aerialimages.py).

Tested only using my own docker image running ROS 2 Galactic (`ricardodeazambuja/ros2_quad_sim_python`).

![image](https://github.com/ricardodeazambuja/ros2_satellite_aerial_view_simulator/assets/6606382/1cc3f68c-ebb5-4ec5-9289-b51732fbcafb)
