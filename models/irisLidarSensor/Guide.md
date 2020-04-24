# Setup

copy "1019_iris_lidar" to '~/Firmware/ROMFS/px4fmu_common/init.d-posix'

copy "iris_lidar" to '~/Firmware/Tools/sitl_gazebo/models'

# Run
start by source:

```bash 
source /home/$USER/Firmware/Tools/setup_gazebo.bash /home/$USER/Firmware
/home/$USER/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware
export
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware/Tools/sitl_gazebo
```

and run the command:
```bash
roslaunch px4 posix_sitl.launch vehicle:=iris_lidar
```
