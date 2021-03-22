### setup
```
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src/
git clone -b debug/livox git@github.com:tier4/autoware_launcher.iv.universe.git
git clone git@github.com:Livox-SDK/livox_ros2_driver.git
cd ..
colcon build --symlink-install
```

### hardware settings
Please prepare three livox horizon devises and set serial number to config files.

There are front_left, front_center, front_right files in below directory.
```
livox_ws/src/autoware_launcher.iv.universe/sensing_launch/config/aip_x1/default
```

please replace cmdline_input_bd_code to real device's one.
```
/**:
  ros__parameters:
    cmdline_input_bd_code: '100000000000000'
```



### launch
```
source ~/livox_ws/install/setup.bash
rviz2 -d ~/livox_ws/src/autoware_launcher.iv.universe/livox_debug.rviz
ros2 launch sensing_launch sensing.launch.xml sensor_model:=aip_x1
```
