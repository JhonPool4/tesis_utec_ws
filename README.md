# Ros workspace:
## Description:
Control the ur5 robot with the two control methods at Cartesian level: (i) impedance and (ii) adaptive. On one hand, the impedance control method set the interaction force according to the rehabilitation band Theraband. On the other hand, the adaptive control method modify the inertia matrix to compensate the mass uncertainty. Finally, both control method are implemented in c++ for dynamic simulations on Gazebo; and the Pinocchio library is used to compute the dynamics of ur5 robot.

## Requirements:
- Ros Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
- Pinocchio (https://stack-of-tasks.github.io/pinocchio/download.html)
- Pandas (>=0.24)

## Config .bashrc file
Open .bashrc file
<pre><code>$ gedit ~/.bashrc </code></pre>
Add these lines:
<pre><code># ROS
source /opt/ros/noetic/setup.bash 
# ROS WORKSPACE (don't modify)
export work_space="${HOME}/catkin_ws/tesis_utec_ws"
source $work_space/devel/setup.bash
# EIGEN4: environment variable (could be modified)
export eigen_include_dir="$work_space/src/eigen-3.4.0"
# PINOCCHIO: environment variables
export pin_lib="/opt/openrobots/lib"
export pin_include_dir="/opt/openrobots/include"
</code></pre>    



## Config workspace:
Create the workspace
<pre><code>$ mkdir -p ~/catkin_ws/tesis_utec_ws/src 
$ cd ~/catkin_ws/
</code></pre>

Clone repository
<pre><code>$ git clone https://github.com/JhonPool4/tesis_utec_ws.git 
</code></pre>

Create necessary files
<pre><code>$ cd ~/catkin_ws/tesis_utec_ws/
$ catkin_make
</code></pre>

Additional ROS packages
<pre><code>$ sudo apt-get install ros-melodic-ros-control </code></pre>
<pre><code>$ sudo apt-get install ros-melodic-ros-controllers</code></pre>

    
## Packages info
### ur5_description (working)
- This package has urdf file of ur5 robot. 
- Display ur5 robot in rviz: roslaunch ur5_description ur5_rviz.launch
- Display ur5 robot in gazebo: roslaunch ur5_description ur5_gazebo.launch

### motion_ur5_gazebo (working)
- This package use gazebo and pinocchio library to perfrom a dynamic simulation of the ur5 robot with the control methods.
- ur5 robot + adaptive control:  roslaunch motion_ur5_gazebo exercise_1.launch
- ur5 robot + impedance control: roslaunch motion_ur5_gazebo exercise_2.launch

### my_control_gazego (working)
- This package store the control library with impedance and adaptive control method
    



