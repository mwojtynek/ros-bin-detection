## ros-bin-detection

Software Stack for bin detection and MoveIt planning: https://youtu.be/SRn_uwcKheE
<br><br>
The software runs under the robot framework ROS (version: Hydro Release) and starts with the following commands in the Linux console:

### Launch the robot UR5 Universal Robots driver:
- roslaunch ur_bringup ur5_bringup_demonstrator.launch 

### Launch the simulation environment:
- roslaunch ur5_iml_moveit_config ur5_state_machine.launch

### Launch the image processing node:
- rosrun perception perception

### Launch the state machine:
- roslaunch state_machine state_machine.launch
<br>
Bochum, 06.01.2015<br>
Michael Wojtynek
<br>
<br>
<img src="https://github.com/mwojtynek/ros-bin-detection/blob/main/perception.jpg" alt="drawing" width="550"/>
<br>

#### SMACH state machine

<img src="https://github.com/mwojtynek/ros-bin-detection/blob/main/state_machine.png" alt="drawing" width="550"/>
