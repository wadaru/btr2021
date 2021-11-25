#!/bin/bash
ROBOT=`hostname`
NUM=`echo ${ROBOT##*["-"]}`
echo $NUM
