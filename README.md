# IUAV_F_2020

## setup

``` bash
cd ~/catkin_ws
catkin_create_pkg track_object
```

``` bash
cd track_object/src/
mkdir dev_track_obj
git clone https://github.com/DumboJunior/IUAV_F_2020.git
```

``` bash
catkin build
source devel/setup.bash

```

## Run

``` bash
cd ~/catkin_ws
rosrun Track_obj track_object.py
```
