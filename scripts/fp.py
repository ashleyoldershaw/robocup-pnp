import os
import sys
sys.path.insert(0, os.getenv('PNP_HOME')+'/PNPros/ROS_bridge/pnp_ros/py')

import pnp_cmd_ros
from pnp_cmd_ros import *

p = PNPCmd()

p.begin()

p.exec_action('enter', 'maindoor')
p.exec_action('turn', '90_ABS')

p.exec_action('followperson', '', interrupt='stopfollow', recovery='skip_action')
# rosparam set /diago_0/PNPconditionsBuffer/stopfollow 1

p.exec_action('say', 'goodbye')

p.exec_action('goto', 'exit')


p.end()

