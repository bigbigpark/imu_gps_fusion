# GPS + IMU fusion

* `gps` : x, y position
* `imu` : heading angle

<br/>

## Prerequisites

install [FAST-LIO](https://github.com/hku-mars/FAST_LIO)

<br/>

## How to run

~~~bash
git clone https://github.com/bigbigpark/imu_gps_fusion.git
catkin build
~~~

~~~bash
roslaunch pose_fusion fusion.launch
~~~