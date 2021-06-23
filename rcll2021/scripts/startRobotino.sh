#!/bin/bash
##
REFBOX_DIR="~/rcll-refbox/bin"
# SCRIPT_DIR="~/ownCloud/2021/scripts"
# ROBVW2_DIR="~/ownCloud/2021/rvw2"
# PYTHON_DIR="~/ownCloud/2021/python"
SCRIPT_DIR="~/git/btr2021/rcll2021/scripts"
ROBVW2_DIR="~/git/btr2021/rcll2021/rvw2"
PYTHON_DIR="~/git/btr2021/rcll2021/python"

ROBVIEW="robview"
# ROBVIEW="robview_interpreter"
for PROGNAME in roscore; do
	killall $PROGNAME
done

sudo chmod 777 /dev/ttyUSB?

gnome-terminal --geometry=105x56 --window\
	      -e "bash -c roscore"\
	--tab --working-directory="$SCRIPT_DIR" -e "bash -c $SCRIPT_DIR/rosRcllRefBoxNetwork.sh" \
	--tab --working-directory="$ROBVW2_DIR" -e "bash -c '$ROBVIEW -f $ROBVW2_DIR/ros.rvw2'" \
	--tab --working-directory="$PYTHON_DIR" -e "bash -c 'rosrun rcll_btr_msgs robotino.py'" \
	--tab --working-directory="~/catkin_ws/src/rplidar_ros/launch" -e "bash -c 'roslaunch rplidar_a3.launch'" \
	--tab --working-directory="$PYTHON_DIR" -e "bash" # refbox.py

