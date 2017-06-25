#!/bin/sh
# Use: ./runplan.sh <planname>
# Example: ./runplan.sh cocktail_party

rostopic pub /diago/planToExec std_msgs/String "data: '$1'" --once
