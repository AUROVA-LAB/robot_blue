# blue_package
This package contains the files needed to launch all the components necessaries to run the BLUE mobile robotics research platform in the ROS environment. Each directory of each package contains:

### Launch files
This directory contains the files necessaries to run all the sensors and actuators, both separatelly and all together. This file depends on the ROS packages that contais the drivers for each sensor/actuator.

### Parameters
This directory contains the .yaml files that include the adjustment parameters related with this robot.

### URDF files
This files contains all the transformations between base_link of robot to each sensor.

### Publications
del Pino, I., Munoz-Banon, M. A., Cova-Rocamora, S., Contreras, M. A., Candelas, F. A., & Torres, F. (2020). Deeper in BLUE. Journal of Intelligent & Robotic Systems, 98(1), 207-225.

Citation:
``` 
@article{del2020deeper,
  title={Deeper in BLUE},
  author={del Pino, Ivan and Munoz-Banon, Miguel A and Cova-Rocamora, Saul and Contreras, Miguel A and Candelas, Francisco A and Torres, Fernando},
  journal={Journal of Intelligent \& Robotic Systems},
  volume={98},
  number={1},
  pages={207--225},
  year={2020},
  publisher={Springer}
}
``` 

Muñoz–Bañón, M. Á., del Pino, I., Candelas, F. A., & Torres, F. (2019). Framework for fast experimental testing of autonomous navigation algorithms. Applied Sciences, 9(10), 1997.

Citation:
``` 
@article{munoz2019framework,
  title={Framework for fast experimental testing of autonomous navigation algorithms},
  author={Mu{\~n}oz--Ba{\~n}{\'o}n, Miguel {\'A} and del Pino, Iv{\'a}n and Candelas, Francisco A and Torres, Fernando},
  journal={Applied Sciences},
  volume={9},
  number={10},
  pages={1997},
  year={2019},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```

**The hardware components contained in this platform are:** sensor Lidar 3D Velodyne  VPL16, sensor Lidar 2D Hokuyo UBG-04LX-F01, camera  RGBD Intel Realsense D435, GPS-RTK Ublox M8P, and sensor IMU CHR-UM7. It also contains a generic module for the robotization of ackermann vehicles developed by the AUROVA group.

![BLUE_picture](/robot_blue/images/blue.jpg)
