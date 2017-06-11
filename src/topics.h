// defines the names of the topics to send data between modules

#ifndef __TOPICS__
#define __TOPICS__

#define TOPIC_ROBOT_LOCATION "amcl_pose"
#define TOPIC_LASER_OBSMAP "laser_obstacle_map"

#define TOPIC_MOVE_BASE "move_base"
#define TOPIC_TURN "turn"
#define TOPIC_FOLLOWCORRIDOR "follow_corridor"
#define TOPIC_FOLLOWPERSON "follow_person"
#define TOPIC_SAY "say"
#define TOPIC_STAGE_SAY "stage_say"

#define TOPIC_PLANTOEXEC "planToExec"
#define TOPIC_PNPACTIVEPLACES "pnp/currentActivePlaces"
#define TOPIC_PNPCONDITION "PNPConditionEvent"
#define TOPIC_PNPACTIONCMD "PNPActionCmd"

#define PARAM_PNPCONDITIONBUFFER "PNPconditionsBuffer/"

#define TOPIC_RCOMMESSAGE "/RCOMMessage"

#endif
