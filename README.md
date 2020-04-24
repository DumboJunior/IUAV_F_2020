# IUAV_F_2020

## setup

``` bash
cd ~/catkin_ws/src
catkin_create_pkg track_object std_msgs rospy roscpp
```

``` bash
cd track_object/src/
mkdir dev_track_obj
cd dev_track_obj
git clone https://github.com/DumboJunior/IUAV_F_2020.git
```

``` bash
cd ~/catkin_ws
catkin build
source devel/setup.bash

```

## Run

``` bash
cd ~/catkin_ws
rosrun track_object track_object.py
```
