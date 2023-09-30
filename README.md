# Autonomous Poop Detection Robot
This is the source code repo for 2023 Spring Senior Design Project for at UCLA. The code intends to create a robot that is able to navigate autonomously, look for poops lying on the ground, and capture it using ROS2 Framework.   
   
T**he presentation detailing the features of the robot can be found** [**HERE**](https://docs.google.com/presentation/d/1VEbmPY12iC3s48DDqnpAfTMvGVQjlws_VFBhDWr3rWM/edit?usp=sharing) 
## Dependencies
- ROS2 Humble
- xacro
- ROS2 Control
- Navigation 2
- SLAM Toolbox
- OpenCV

## Important Folders
### CRAP_navigation
*Coded by Jinkai*  
The folder defines a ROS2 package that executes navigation related tasks such as the main control script, logics to look for poop, and logics to capture the poop. `CRAP_navigation` sub-folder defines the core functions of the system. `config` folder contains configuration related to NAV2 stack, velocity muxing, and Extended Kalman Filter. `maps` folder contains stored maps for reference by the NAV2 stack. `launch` folder contains necessary launch files to launch each of the functionalities of the robot, mostly for testing and debugging purposes.
### CRAP_sim_def 
*Coded by Jinkai*  
The folder defines a ROS2 package to launch the robot either in simulation or in the physical environment. `urdf` folder contains urdf definition files for the robot. `launch` folder contains launch files that are responsible for launching **either** the simulation or the actual physical hardwares. 
### Hardware Drivers
*Adopted and Coded by Jinkai*
The folder contains drivers for various hardwares of the robot, as well as Arduino codes. `ROSArduinoBridge` defines scripts that drive an Arduino for the lower-level control of two drive motors, in serial control with the controller raspberry. `diffdrive_arduino` is adopted to provide ROS2 control interface that handles kinematics with the Arduino controlling the drive motors. `poop_collection_arduino` defines scripts that drive another Arduino for executing the poop capturing actions, in serial communication with the raspberry. `serial_motor_demo` defines serial interface between the raspberry and the Arduinos, and it also defines an action server that executes the poop capturing action.  `serial_motor_msgs` define the customized ROS2 messages and actions used by `serial_motor_demo`. 
### learning_node
*Coded by Yini*  
The folder defines a ROS2 package that executes CV task that identifies poop location in 3D using a RGBD camera. 

## To Run
1. Compile ROS2 packages
2. Map operating environment with SLAM toolbox, running `ros2 launch CRAP_navigation real_slam.launch.py`
3. Save the map under `CRAP_navigation/maps`
4. Modify `CRAP_navigation/launch/real_nav2.launch.py` so it reflects the map file name and path
5. Launch the robot hardwares with `ros2 launch CRAP_sim_def CRAP_sim_def/launch/real_robot_comprehensive_launch.launch.py`
6. Launch NAV2 Stack along with main controller with `ros2 launch CRAP_navigation real_nav2.launch.py`
7. (optional) Launch velocity muxing with `ros2 launch CRAP_navigation vel_mux.launch.py`

## Main Controller Logics
Defined under `CRAP_navigation/CRAP_navigation/main_controller.py`   
Three states are orchestrated encapsulated in the `timer_callback method`, which is triggered periodically by a timer created during the initialization of the crap_main_controller node.   
1. **Navigation State**: The robot navigates towards the detected object to a suitable position for collection. This state involves handling navigation goals and feedback to ensure the robot reaches the desired location.
2. **Collection State**: Once in position, the robot initiates the collection process. During this state, it sends a goal to an action server to collect the object.
3. **Searching State**: If no object is detected, the robot navigates to different points in its environment in search of objects.

## Credits
Patrick Goebel and James Nugen for the original ROSArduinoBridge   
Josh Newans for a forked ROSArduinoBridge, part of the serial_motor_demo, part of the serial_motor_msgs packages, and diffdrive_arduino   
YDLidar for providing a ROS2 driver for the Lidar ( Although the driver is faulty and needed to be debugged before actual use :( )     
DepthAI for providing a ROS2 driver for the RGBD camera

