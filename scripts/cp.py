import os
import sys
sys.path.insert(0, os.getenv('PNP_HOME')+'/PNPros/ROS_bridge/pnp_ros/py')

import pnp_cmd_ros
from pnp_cmd_ros import *

p = PNPCmd()

p.begin()

p.plan_cmd('cocktail_party', 'start')

while (not p.get_condition('stopplan')):
    time.sleep(1)

p.plan_cmd('cocktail_party', 'stop')

p.exec_action('goto','exit')

p.end()

