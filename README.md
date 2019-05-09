# blue_package
This package contains the files needed to launch all the components necessaries to run the BLUE mobile robotics research platform in the ROS environment. Each directory of each package contains:

### Launch files
This directory contains the files necessaries to run all the sensors and actuators, both separatelly and all together.

### Parameters
This directory contains the .yaml files that include the adjustment parameters related with this robot, but used by other ROS packages. In this case, the directory contains two files with the robot parameters requiered by [ACML package](http://wiki.ros.org/amcl), and [aurova_control](https://github.com/AUROVA-LAB/aurova_control) respectively.

### URDF files
This files contains all the transformations between base_link of robot to each sensor.

### Publications
del Pino, I., Muñoz-Bañon, M. Á., Cova-Rocamora, S., Contreras, M. Á., Candelas, F. A., & Torres, F. (2019). Deeper in BLUE. Journal of Intelligent & Robotic Systems, 1-19.

**The hardware components contained in this platform are:** sensor Lidar 3D Velodyne  VPL16, sensor Lidar 2D Hokuyo UBG-04LX-F01, camera  RGBD Intel Realsense D435, GPS-RTK Ublox M8P, and sensor IMU CHR-UM7. It also contains a generic module for the robotization of ackermann vehicles developed by the AUROVA group.

![BLUE_picture](/images/blue.jpg)
