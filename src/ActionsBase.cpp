#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>
#include <tcp_interface/RCOMMessage.h>
#include <boost/algorithm/string.hpp>
#include <std_srvs/Empty.h>

#include "RCHPNPAS.h"
#include "topics.h"

using namespace std;

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)

inline double norm_pi(double a) {
    if (a>M_PI) return a-2*M_PI;
    else if (a<M_PI) return a+2*M_PI;
    else return a;
}

// absolute difference between angles
inline double abs_diff(double a, double b) {
    double c = fabs(a-b);
    if (c<M_PI) return c;
    else return 2*M_PI - c;
}

#define SPEECH_TIMEOUT 5


bool RCHPNPActionServer::getRobotPose(std::string robotname, double &x, double &y, double &th_rad) {
    if (listener==NULL) {
        listener = new tf::TransformListener();
    }

    string src_frame = "/map";
    string dest_frame = "/" + robotname + "/base_frame";
    if (robotname=="") { // local trasnformation
        src_frame = "map";
        dest_frame = "base_link";
    }

    tf::StampedTransform transform;
    try {
        listener->waitForTransform(src_frame, dest_frame, ros::Time(0), ros::Duration(3));
        listener->lookupTransform(src_frame, dest_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        th_rad = 999999;
        ROS_ERROR("Error in tf trasnform %s -> %s\n",src_frame.c_str(), dest_frame.c_str());
        ROS_ERROR("%s", ex.what());
        return false;
    }
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    th_rad = tf::getYaw(transform.getRotation());

    return true;
}




bool RCHPNPActionServer::getLocationPosition(string loc, double &GX, double &GY) {

	if (locationcoords.find(loc)!=locationcoords.end()) {
		GX = locationcoords[loc].X; GY = locationcoords[loc].Y;
	}
    else {
        ROS_ERROR_STREAM("Location "<<loc<<" unknown.");
        return false;
    }

    ROS_INFO_STREAM("Location " << loc << " at " << GX  << " , " << GY);  

    return true;
}


bool RCHPNPActionServer::getDoorEntrancePosition(string loc, double &GX, double &GY) {

	if (doorcoords.find(loc)!=doorcoords.end()) {
		GX = doorcoords[loc].enterX; GY = doorcoords[loc].enterY;
	}
    else {
        ROS_ERROR_STREAM("Door "<<loc<<" unknown.");
        return false;
    }

    ROS_INFO_STREAM("Door " << loc << " entrance at " << GX  << " , " << GY);  

    return true;
}

bool RCHPNPActionServer::getDoorExitPosition(string loc, double &GX, double &GY) {

	if (doorcoords.find(loc)!=doorcoords.end()) {
		GX = doorcoords[loc].exitX; GY = doorcoords[loc].exitY;
	}
    else {
        ROS_ERROR_STREAM("Door "<<loc<<" unknown.");
        return false;
    }

    ROS_INFO_STREAM("Door " << loc << " exit at " << GX  << " , " << GY);  

    return true;
}




/*
 *   ACTION UTILS
 */


void RCHPNPActionServer::clear_costmaps()
{
    ROS_INFO("Clear costmap!");

    char srvname[80];
    sprintf(srvname,"/%s/move_base_node/clear_costmaps",robotname.c_str());
    
    ros::ServiceClient client = handle.serviceClient<std_srvs::Empty>(srvname);
    std_srvs::Empty srv;
    if (client.call(srv)) {
        ROS_INFO("Costmaps cleared.\n");
    }
    else {
        ROS_ERROR("Failed to call service %s", srvname );
    }
}



void RCHPNPActionServer::do_movebase(float GX, float GY, float GTh_DEG, bool *run) { // theta in degrees

  targetGX=GX; targetGY=GY; targetGTh_deg=GTh_DEG;

  mtx_movebase.lock();

/*
  std::string def_force_scale, def_momentum_scale;

  ros::param::get("/"+robotname+"/gradientBasedNavigation/force_scale", def_force_scale);
  ros::param::get("/"+robotname+"/gradientBasedNavigation/momentum_scale", def_momentum_scale);

  ros::param::set("/"+robotname+"/gradientBasedNavigation/force_scale", 0.4);
  ros::param::set("/"+robotname+"/gradientBasedNavigation/momentum_scale", 0.3);
*/

  double force_scale = 0.4, momentum_scale = 0.3;

  handle.setParam("/"+robotname+"/gradientBasedNavigation/force_scale", force_scale);
  handle.setParam("/"+robotname+"/gradientBasedNavigation/momentum_scale", momentum_scale);
  ros::spinOnce();


  if (ac_movebase==NULL) { //create the client only once
    // Define the action client (true: we want to spin a thread)
    ac_movebase = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(TOPIC_MOVE_BASE, true);

    // Wait for the action server to come up
    while(!ac_movebase->waitForServer(ros::Duration(5.0)) && *run){
	    ROS_INFO("Waiting for move_base action server to come up");
    }
  }

  if (!*run) return;

  double RX,RY,RTH;
  while (!(getRobotPose(robotname, RX, RY, RTH))) {
      ROS_ERROR("Cannot get robot pose!!!");
      ros::Duration(0.25).sleep();
  }

  // Read time
  double secs =ros::Time::now().toSec();
  while (secs==0) {  // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY
	  ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
	  ros::Duration(0.25).sleep();
      secs =ros::Time::now().toSec();
  }

  clear_costmaps(); ros::Duration(0.25).sleep();

  // Set the goal (MAP frame)
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = GX;
  goal.target_pose.pose.position.y = GY;
  goal.target_pose.pose.orientation.z = sin(RAD(GTh_DEG)/2);
  goal.target_pose.pose.orientation.w = cos(RAD(GTh_DEG)/2);

  // compute target angle
  double tth = atan2(GY-RY, GX-RX);
  double dth = abs_diff(tth,RTH);
  if (dth>M_PI/2) {
     do_turn("ABS",DEG(tth),run);
  }

  // Send the goal
  ROS_INFO("move_base: sending goal %.1f %.1f",GX,GY);
  ac_movebase->sendGoal(goal);

  // Wait for termination (check distance every delay seconds)
  double delay = 0.1;
  double d_threshold=0.5, d=d_threshold+1.0;
  while (!ac_movebase->waitForResult(ros::Duration(delay)) && (*run) && (d>d_threshold)) {
    // ROS_INFO("Running...");
    if (getRobotPose(robotname, RX, RY, RTH))
      d = fabs(GX-RX)+fabs(GY-RY);
  }

  // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
  ac_movebase->cancelAllGoals(); ros::Duration(0.1).sleep(); // wait a little

  mtx_movebase.unlock();

}



void RCHPNPActionServer::do_turn(string absrel_flag, float GTh_DEG, bool *run) {

    if (ac_turn==NULL) {
      ac_turn = new actionlib::SimpleActionClient<rococo_navigation::TurnAction>(TOPIC_TURN, true);

      while(!ac_turn->waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for turn action server to come up");
      }
    }

    if (!*run) return;

    rococo_navigation::TurnGoal goal;

    goal.target_angle = GTh_DEG;
    goal.absolute_relative_flag = absrel_flag;
    goal.max_ang_vel = 50.0;  // deg/s

    // Send the goal
    ROS_INFO("Sending goal TURN %f", GTh_DEG);
    ac_turn->cancelAllGoals(); ros::Duration(0.1).sleep();
    ac_turn->sendGoal(goal);

    while (!ac_turn->waitForResult(ros::Duration(0.1)) && (*run)){
       // ROS_INFO("Turning...");
    }
    ac_turn->cancelAllGoals();
}



