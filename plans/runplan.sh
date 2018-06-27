#!/bin/sh
# Use: ./runplan.sh <robotname> <planname>
# Example: ./runplan.sh diago_0 cocktail_party

./genplan.sh $2.plan $2.er
rosparam set /$1/pnp/plan_folder `pwd`
rostopic pub /$1/planToExec std_msgs/String "data: '$2'" --once

