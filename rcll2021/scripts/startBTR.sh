#!/bin/bash
##
REFBOX_DIR="~/rcll-refbox/bin"
# SCRIPT_DIR="~/ownCloud/2021/scripts"
# ROBVW2_DIR="~/ownCloud/2021/rvw2"
# PYTHON_DIR="~/ownCloud/2021/python"
SRC_DIR="~/git/btr2021/rcll2021"
SCRIPT_DIR="$SRc_DIR/scripts"
ROBVW2_DIR="$SRC_DIR/rvw2"
PYTHON_DIR="$SRC_DIR/python"

for PROGNAME in roscore llsf-refbox llsf-refbox-shell python; do
	killall $PROGNAME
done

gnome-terminal --geometry=105x56 --window\
	      -e "bash -c roscore"\
	--tab --working-directory="$REFBOX_DIR" -e "bash -c $REFBOX_DIR/llsf-refbox" \
	--tab --working-directory="$REFBOX_DIR" -e "bash -c $REFBOX_DIR/llsf-refbox-shell" \
	--tab --working-directory="$SCRIPT_DIR" -e "bash -c $SCRIPT_DIR/rosRcllRefBox.sh" \
	--tab --working-directory="$SCRIPT_DIR" -e "bash -c $SCRIPT_DIR/robotinoSimDemo.sh" \
	--tab --working-directory="$ROBVW2_DIR" -e "bash -c 'robview -f $ROBVW2_DIR/ros.rvw2'" \
	--tab --working-directory="$PYTHON_DIR" -e "bash -c 'rosrun rcll_btr_msgs robotino.py'" \
     	--tab --working-directory="$PYTHON_DIR" -e "bash -c 'python $PYTHON_DIR/refbox.py test'" \
	--tab --working-directory="$SCRIPT_DIR" -e "bash -c $SCRIPT_DIR/simDemoLimit.sh" \

