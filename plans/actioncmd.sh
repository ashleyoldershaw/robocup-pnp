#!/bin/sh
# Use: ./actioncmd.sh <robotname> <actionname> {start|end|interrupt}
# Example: ./actioncmd.sh diago goto_kitchen start

rostopic pub /$1/PNPActionCmd std_msgs/String "data: '$2 $3'" --once

