#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>
#include <tcp_interface/RCOMMessage.h>
#include <boost/algorithm/string.hpp>

#include "RCHPNPAS.h"
#include "topics.h"

using namespace std;

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)

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






/*
 *   ACTION UTILS
 */



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
    while(!ac_movebase->waitForServer(ros::Duration(5.0))){
	    ROS_INFO("Waiting for move_base action server to come up");
    }
  }

  // Read time
  double secs =ros::Time::now().toSec();
  while (secs==0) {  // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY
	  ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
	  ros::Duration(0.25).sleep();
      secs =ros::Time::now().toSec();
  }

  // Set the goal (MAP frame)
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = GX;
  goal.target_pose.pose.position.y = GY;
  goal.target_pose.pose.orientation.z = sin(RAD(GTh_DEG)/2);
  goal.target_pose.pose.orientation.w = cos(RAD(GTh_DEG)/2);

  // Send the goal
  ROS_INFO("move_base: sending goal %.1f %.1f",GX,GY);
  ac_movebase->sendGoal(goal);

  // Wait for termination (check distance every delay seconds)
  double delay = 0.1;
  double d_threshold=0.5, d=d_threshold+1.0;
  while (!ac_movebase->waitForResult(ros::Duration(delay)) && (*run) && (d>d_threshold)) {
    // ROS_INFO("Running...");
    double RX,RY,RTH;
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



void RCHPNPActionServer::do_follow_corridor(float GX, float GY, bool *run) {

  double force_scale = 0.6, momentum_scale = 0.2;

  handle.setParam("/"+robotname+"/gradientBasedNavigation/force_scale", force_scale);
  handle.setParam("/"+robotname+"/gradientBasedNavigation/momentum_scale", momentum_scale);
  ros::spinOnce();

  cout << "Follow corridor params - gbn: " <<  force_scale << " " << momentum_scale << endl;
/*
  double par;
  handle.getParam("/"+robotname+"/gradientBasedNavigation/force_scale",par);

  if (par<0.001)
    ROS_ERROR("force_scale gbn too low!!!");
  else
    ROS_INFO("force_scale gbn: %.2f", par);
*/

  if (ac_followcorridor==NULL) {
    // Define the action client (true: we want to spin a thread)
    ac_followcorridor = new actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction>(TOPIC_FOLLOWCORRIDOR, true);

    // Wait for the action server to come up
    while(!ac_followcorridor->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for follow_corridor action server to come up");
    }
  }

  // Read time
  double secs =ros::Time::now().toSec();
  while (secs==0) {  // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY
      ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
      ros::Duration(0.25).sleep();
    secs =ros::Time::now().toSec();
  }

  // Set the goal (MAP frame)
  rococo_navigation::FollowCorridorGoal goal;
  goal.target_X = GX;  goal.target_Y = GY;   // goal
  goal.max_vel = 1.0;  // m/s

  // Send the goal
  ROS_INFO("Sending goal follow corridor %.1f %.1f",GX,GY);
  ac_followcorridor->sendGoal(goal);

  // Wait for termination
  double d_threshold=0.5, d=d_threshold+1.0;
  while (!ac_followcorridor->waitForResult(ros::Duration(0.1)) && (*run) && (d>d_threshold)) {
      // ROS_INFO("Running...");
      double RX,RY,RTH;
      if (getRobotPose(robotname, RX, RY, RTH))
        d = fabs(GX-RX)+fabs(GY-RY);
      if (d<d_threshold*2.0)
        handle.setParam("/"+robotname+"/gradientBasedNavigation/force_scale", force_scale*0.66);
  }

  handle.setParam("/"+robotname+"/gradientBasedNavigation/force_scale", force_scale);

  // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
  ac_followcorridor->cancelAllGoals(); ros::Duration(0.1).sleep(); // wait 
}


void RCHPNPActionServer::do_follow_person(bool *run) {

    double max_vel = 1.0; 

    if (ac_followperson==NULL) {
        ac_followperson = new actionlib::SimpleActionClient<rococo_navigation::FollowPersonAction>(TOPIC_FOLLOWPERSON, true);
    }

    // Wait for the action server to come up
    while(!ac_followperson->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for follow_person action server to come up");
    }

    // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
    ac_followperson->cancelAllGoals(); ros::Duration(0.1).sleep();

    // Set the goal
    rococo_navigation::FollowPersonGoal goal;
    goal.target_X = 0;  goal.target_Y = 0;   // unused so far
    goal.max_vel = max_vel;  // m/s

    // Send the goal
    ROS_INFO("Sending goal");
    ac_followperson->sendGoal(goal);

	  // Wait for termination
    while (!ac_followperson->waitForResult(ros::Duration(0.1)) && (*run)) {
	    // ROS_INFO_STREAM("Running... [" << ac_followperson.getState().toString() << "]");
    }
    // ROS_INFO_STREAM("Finished [" << ac.getState().toString() << "]");

    // Print result
    if (ac_followperson->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("FollowPerson successful");
    else
        ROS_INFO("FollowPerson failed");

    // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
    ac_followperson->cancelAllGoals(); ros::Duration(0.1).sleep(); // wait 

}


void RCHPNPActionServer::do_fixedMove(string params, bool *run) {
  cout << "### Executing Fixed Move " << params << " ... " << endl;
  double GX, GY, RX, RY, RTH, IRX, IRY, IRTH;
  if (!getRobotPose(robotname, IRX, IRY, IRTH)) {
    ROS_ERROR("Fixed move: Cannot determine robot pose.");
    return;
  }

  ros::Publisher desired_cmd_vel_pub = handle.advertise<geometry_msgs::Twist>("/"+robotname+"/desired_cmd_vel", 1);

  double force_scale_old, momentum_scale_old;
  handle.getParam("/"+robotname+"/gradientBasedNavigation/force_scale", force_scale_old);
  handle.setParam("/"+robotname+"/gradientBasedNavigation/force_scale", 0.05);
  handle.getParam("/"+robotname+"/gradientBasedNavigation/momentum_scale", momentum_scale_old);
  handle.setParam("/"+robotname+"/gradientBasedNavigation/momentum_scale", 0.0);


  //Desired translation in x
  double tx;// = atof(params.c_str());
  double timeout_factor;

  vector<string> vparams;
  boost::split(vparams, params, boost::is_any_of("_")); // split parameters
  if (vparams.size()==1) {
    tx = atof(vparams[0].c_str());
    timeout_factor = 20;
  }
  else {
    tx = atof(vparams[0].c_str());
    timeout_factor = atof(vparams[1].c_str());
  }

  //Desired goal
  GX = RX + tx * cos(RTH);
  GY = RY + tx * sin(RTH);

  RX = IRX;
  RY = IRY;
  double threshold = 0.05;
  double velocity = 0.4;
  double timeout = (tx / velocity)*timeout_factor;
  double rate = 0.1;
  double timer = 0.0;
  while (fabs(IRX-RX)<tx && fabs(IRY-RY)<tx && timer < timeout ){
    geometry_msgs::Twist cmd;
    cmd.linear.x = velocity;
    desired_cmd_vel_pub.publish(cmd);
    ros::Duration(rate).sleep(); // wait ...    
    getRobotPose(robotname, RX, RY, RTH);
    timer += rate;
  }

  handle.setParam("/"+robotname+"/gradientBasedNavigation/force_scale", force_scale_old); //Restoring
  handle.setParam("/"+robotname+"/gradientBasedNavigation/momentum_scale", momentum_scale_old);

  //stop robot
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0;
  desired_cmd_vel_pub.publish(cmd);
  ros::Duration(rate).sleep(); // wait ...    
  
}


