# gnss_cal (not finish yet)
The whole project aims to improve the accuracy of GNSS with the help of lidar. Multipath and NLOS's effect are mitigated here. EKF is used to smoothen trajectory.

**Attention: Result is not validated**
---
## Prequisite:
1. ROS melodic
2. Ubuntu 18.04
3. Lidar, GPS, IMU related ros driver

## Node

|   ros node       |function                                                  |
|------------------|----------------------------------------------------------|
| Plane check node | check planes about Pointcloud collecting from Lidar      |
| Gnss cal node    | Analysis satellites infomation from broadcasting ephemris| 
| Particle node    | Improve GPS' accuracy according to NLOS                  |
| ekf node         | Extend Kalman Filter to smoothen the trajectory          |

## Execute
+ Every rosnode can be executed seperately.
+ set topic's name before excution 
```
roslaunch gnss_cal gnss_ekf_imu_pf.launch
```
## literature
https://link.springer.com/chapter/10.1007/978-981-16-6324-6_51
