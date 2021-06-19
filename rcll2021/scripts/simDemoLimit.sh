#!/bin/bash
cpulimit -z -l 10 -p `ps aux| grep robotinosim.exe|head -n1|awk '{print $2}'`

