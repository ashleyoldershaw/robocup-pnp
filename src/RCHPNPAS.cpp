#include <boost/thread/thread.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/algorithm/string.hpp>
#include <tcp_interface/RCOMMessage.h>
#include <laser_analysis/LaserObstacleMap.h>

#include "topics.h"

#include "RCHPNPAS.h"

RCHPNPActionServer::RCHPNPActionServer(ros::NodeHandle n) : PNPActionServer(), handle(n), handlep("~")  
{
    event_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);
    plantoexec_pub = handle.advertise<std_msgs::String>(TOPIC_PLANTOEXEC, 10);
    rcom_pub= handle.advertise<tcp_interface::RCOMMessage>(TOPIC_RCOMMESSAGE,10);

	// Actions implemented in PNPActionServer
    register_action("noaction",&PNPActionServer::none,(PNPActionServer*)this);
    register_action("waitfor",&PNPActionServer::waitfor,(PNPActionServer*)this);
    register_action("restart",&PNPActionServer::restartcurrentplan,(PNPActionServer*)this);

	// Actions implemented here
    register_action("goto",&RCHPNPActionServer::goto_movebase,this);
    register_action("turn",&RCHPNPActionServer::turn,this);
    register_action("enter",&RCHPNPActionServer::enter,this);
    register_action("exit",&RCHPNPActionServer::exit,this);
    register_action("tell",&RCHPNPActionServer::tell,this);
    register_action("say",&RCHPNPActionServer::say,this);
    register_action("lookfor",&RCHPNPActionServer::lookfor,this);
    register_action("interact",&RCHPNPActionServer::interact,this);
    register_action("ask",&RCHPNPActionServer::ask,this);
    register_action("answer",&RCHPNPActionServer::answer,this);
    register_action("approach",&RCHPNPActionServer::approach,this);
    register_action("grab",&RCHPNPActionServer::grab,this);
    register_action("sense",&RCHPNPActionServer::sense,this);
    register_action("GUIstart",&RCHPNPActionServer::GUIinit,this);

    handle.param("robot_name",robotname,string("diago"));

    listener = new tf::TransformListener();

    ac_movebase = NULL; ac_turn = NULL; ac_followcorridor = NULL; ac_followperson = NULL;

    
    laser_obsmap_sub = handle.subscribe(TOPIC_LASER_OBSMAP, 1, &RCHPNPActionServer::laserobsmapCallback, this);
    tcp_sub = handle.subscribe(TOPIC_RCOMMESSAGE, 10, &RCHPNPActionServer::tcpCallback, this);
    //cond_sub = handle.subscribe(TOPIC_PNPCONDITION, 10, &RCHPNPActionServer::conditionCallback, this);
    active_places_sub = handle.subscribe(TOPIC_PNPACTIVEPLACES, 10, &RCHPNPActionServer::active_places_callback, this);
	
    PNP_cond_pub = handle.advertise<std_msgs::String>(TOPIC_PNPCONDITION, 10);
	stage_say_pub = handle.advertise<std_msgs::String>(TOPIC_STAGE_SAY, 10);

    //last_condition_received="";

    targetGX=-999; targetGY=-999; targetGTh_deg=999;

	initDoors();
	initLocations();
}


void RCHPNPActionServer::active_places_callback(const std_msgs::String::ConstPtr& msg)
{
    ConditionCache.clear();
    if (msg->data=="init;") {
        ROS_INFO("Init place.");
    }
}


