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

## Quick run instructions ##

Robot moving in the apartment in a Navigation test.

Terminal 1:

```
$ roscore
```

Note: first time you run this command takes a few seconds.

Terminal 2:

```
$ cd ~/src/stage_environments/scripts
$ ./start_simulation.py
```

Select:
    Map: peccioli@Home
    Robot: diago
    Localization: thin_localizer
    Navigation: move_base
    Start
    
Terminal 3:

```
$ cd ~/src/robocupathome_pnp/launch
$ roslaunch rcathome_pnp.launch
```

Terminal 4:

```
$ cd ~/src/robocupathome_pnp/plans
$ ./runplan.sh diago navigation
```

## Cocktail Party plan execution ##

- Start terminals 1 to 4 as in the previous section, using 'cocktail_party' as plan name.

- Observe the flow of execution of the plan on Terminal 3

    - The robot enters the apartment, reaches the living_room and looks for a person.

    - When the plan is waiting for a person (waitfor_personhere action), 
      move a colored square in the simulator in front of the robot (drag with mouse).

    - The robot asks which drink, send a condition with the following command in another terminal

```
    $ rostopic pub /diago/PNPConditionEvent std_msgs/String "data: 'drink_coke'" --once
```

    (you can replace coke with your favourite drink).

    - The robot goes to the fridge, grabs the drink, returns to the living_room and says 
      to the person to take the drink.

    - Use the following command when done:

```
      $ rostopic pub /diago/PNPConditionEvent std_msgs/String "data: 'done'" --once
```

    - The robot leaves the apartment, while you can enjoy the drink ;-)))





