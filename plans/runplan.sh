#!/bin/sh
rostopic pub /$1/planToExec std_msgs/String "data: '$2'" --once
