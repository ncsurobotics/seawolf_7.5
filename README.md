# seawolf_7.5
converting seawolf 7 into ROS

## Software Depedencies
Ubuntu 16.04
ROS Kinetic

## Building

In order to build this project go into the top level directory and run catkin_make. This is required even if there is no C++ code in the project. This should be run everytime non-script changes are made to the project (i.e new message type or xml/cmakelist changes)

## Running and Monitoring Nodes

To start a node you will need at least two terminals open, in both terminals you MUST run "source devel/setup.bash" within the top level directory of the project. In one terminal run "roscore", and then in the other run "rosrun [PACKAGE_NAME] [SCRIPT NAME]". This will start the node and show any ros logs that the node is logging. To view the topics this node is publishing open another terminal (that I believe should also be sourced) and then run "rostopic echo /[TOPIC NAME]"



