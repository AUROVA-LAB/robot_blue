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

**The hardware components contained in this platform are:** sensor Lidar 3D Velodyne  VPL16, sensor Lidar 2D Hokuyo UBG-04LX-F01, camera  RGBD Intel Realsense D435, GPS-RTK Ublox M8P, and sensor IMU CHR-UM7. It also contains a generic module for the robotization of ackermann vehicles developed by the AUROVA group.

![BLUE_picture](/images/blue.jpg)
