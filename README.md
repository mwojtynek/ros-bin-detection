## ros-bin-detection

The software runs under the robot framework ROS (version: Hydro Release) and starts with the following commands in the Linux console:

### Launch the robot UR5 Universal Robots driver:
- roslaunch ur_bringup ur5_bringup_demonstrator.launch 

### Launch the simulation environment:
- roslaunch ur5_iml_moveit_config ur5_state_machine.launch

### launch the image processing node:
- rosrun perception perception

### launch the state machine:
- roslaunch state_machine state_machine.launch
<br>
Bochum, 06.01.2015
Michael Wojtynek
