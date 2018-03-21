# robocupathome_pnp #

PNP Action Server for RoboCup@Home domain

## Build ##

Compile it in a ROS catkin workspace.
Requires:  ROSPlan, PNP, pnp_ros, pnp_rosplan, ...

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
$ ./runplan.sh diago_0 cocktail_party
```

Note: use 'stop' as planname to stop the current plan.


* Test single actions

```
$ cd plans
$ ./actioncmd.sh <robotname> <actionname> {start|end|interrupt}

```

Example:

```
$ ./actioncmd.sh diago_0 goto_kitchen start
```

## Quick run instructions ##

Robot moving in the apartment in a Navigation test.
Generation and execution of the plan described in 
~/src/ROSPlan/src/rosplan/rosplan_demos/common/
d_robocupathome.pddl + p_navigation.pddl


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
    Localization: srrg_localizer
    Navigation: move_base
    Demo: rcathome
    Start



Terminal 3:

```
rosservice call /kcl_rosplan/planning_server
```

_Note_: this service uses the following files for generating the final PNP:
rosplan/rosplan_demos/common/d.pddl - domain
rosplan/rosplan_demos/common/p.pddl - problem
robocupathome_pnp/plans/default.er - execution rules

The generated plan is written as a text file in robocupathome_pnp/plans/plan.txt
and as a PNP file in robocupathome_pnp/plans/AUTOGEN_plan_0.pnml 
PNP (.pnml) files can be read with Jarp (see Petri Net Plans library).


## Single plan execution ##


Terminal 4:

```
$ cd ~/src/robocupathome_pnp/plans
$ ./runplan.sh navigation
```

Note: to stop a plan at any time use

```
$ ./runplan.sh stop
```


## Cocktail Party plan execution ##


- Start the plan generation of p_cocktailparty.pddl

- Observe the flow of execution of the plan:

    - The robot enters the apartment, reaches the living_room and looks for a person.

    - When the plan is waiting for a person (waitfor_personhere action), 
      move a colored square in the simulator in front of the robot (drag with mouse).

    - The robot asks which drink, send a condition about which drink you would like (see below).

    - The robot goes to the fridge, grabs the drink, returns to the living room and says 
      to the person to take the drink.

    - Tell the robot when done (see below).

    - The robot leaves the apartment, while you can enjoy the drink ;-)))



Commands to send conditions (in another terminal)

Condition about what you'd like to drink.

```
$ rostopic pub /diago_0/PNPConditionEvent std_msgs/String "data: 'drink_coke'" --once
```

Condition about you got the drink.

```
$ rostopic pub /diago_0/PNPConditionEvent std_msgs/String "data: 'done'" --once
```

## Actions available ##

```
goto_<location>
enter_<door>
exit_<door>
grab

GUIinit
say_<interaction>
ask_<interaction>
answer_<interaction>

waitfor_<condition>
lookfor_<condition>

wait
restartcurrentplan
stopcurrentplan
```

Locations

```
home
entrance
exit
corridor1U
corridor1D
corridor2U
bedroom
bedroomout
livingroom
hall
technicalroom
kitchen
kitchentable
fridge
```

Doors

```
maindoor
```

## Conditions available ##

Sensor based conditions

```
personhere
persondetected
```

MODIM interactions (GUI buttons or ASR)



