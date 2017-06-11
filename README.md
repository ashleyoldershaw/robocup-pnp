# robocupathome_pnp #

PNP Action Server for RoboCup@Home domain

## Build ##

Compile it in a ROS catkin workspace.
Requires: pnp_ros

## Run ##

Launch ROS nodes to control a robot and the action servers used in the plans.

Launch rcathome_pnp.launch

```
$ roslaunch robocupathome_pnp rcathome_pnp.launch
```

## Test ##

* Run a plan

```
$ cd plans
$ ./runplan.sh <robotname> <planname>
```

Example: 

```
$ ./runplan.sh diago cocktail_party
```

Note: use 'stop' as planname to stop the current plan.


* Test single actions

```
$ cd plans
$ ./actioncmd.sh <robotname> <actionname> {start|end|interrupt}

```

Example:

```
$ ./actioncmd.sh diago goto_kitchen start
```



