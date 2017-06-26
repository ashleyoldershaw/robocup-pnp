#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>
#include <tcp_interface/RCOMMessage.h>
#include <boost/algorithm/string.hpp>

#include "RCHPNPAS.h"
#include "topics.h"

using namespace std;


#define SPEECH_TIMEOUT 5


// Peccioli@Home map

void RCHPNPActionServer::initLocations() {
	locationcoords["home"].set(8.5, 6.5);
	locationcoords["entrance"].set(8.3, 7.0);
	locationcoords["exit"].set(10.3, 5.0);
	locationcoords["corridor1U"].set(6.5, 17.5);
	locationcoords["corridor1D"].set(6.0, 8.0);
	locationcoords["corridor2U"].set(8.0, 17.5);
	locationcoords["bedroom"].set(3.0, 18.0);
	locationcoords["bedroomout"].set(5.7, 17.5);
	locationcoords["livingroom"].set(4.5, 3.5);
	locationcoords["hall"].set(8.3, 7.0);
	locationcoords["technicalroom"].set(3.0, 9.0);
	locationcoords["kitchen"].set(8.0, 11.0);
	locationcoords["kitchentable"].set(8.0, 12.5, 0);
	locationcoords["fridge"].set(9.0, 14.5, 90);
}

void RCHPNPActionServer::initDoors() {
	doorcoords["maindoor"].set(8.3, 7.0, 10.3, 5.0);
}



/*
 * ACTIONS
 */



void RCHPNPActionServer::goto_movebase(string params, bool *run) {

  cout << "### Executing Move " << params << " ... " << endl;

  vector<string> vparams;
  boost::split(vparams, params, boost::is_any_of("_")); // split parameters

  string loc = vparams[0];
  if (vparams.size()>1)
    loc = vparams[1];

  double GX,GY;
  if (getLocationPosition(loc,GX,GY)) {
    do_movebase(GX,GY,0,run);
  }
  else 
    ROS_WARN("Move: Cannot find location %s.",params.c_str());

  cout << "### Move " << params << ((*run)?" Completed":" Aborted") << endl;

}

// tell - just as say but with different preconditions
void RCHPNPActionServer::tell(string params, bool *run) {
    say(params,run);
}

void RCHPNPActionServer::say(string params, bool *run) {
  cout << "### Executing Say " << params << " ... " << endl;

  if (!*run)
      return;

  string to_send = "say_" + params;
  tcp_interface::RCOMMessage message_to_send;
  message_to_send.robotsender= robotname;
  message_to_send.robotreceiver="all";
  message_to_send.value= to_send;
  rcom_pub.publish(message_to_send);
  
  end_speech=false;
  int sleeptime=SPEECH_TIMEOUT;
  bool psim=false;
  ros::param::get("/use_sim_time",psim);
  
  if (psim) sleeptime=3;
  while (*run && sleeptime-->0 && !end_speech && ros::ok())
    ros::Duration(1.0).sleep();
  end_speech=false;

  if (psim) {
	std_msgs::String mess;
	mess.data = params;
	stage_say_pub.publish(mess);
  }

  cout << "### Say " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::ask(string params, bool *run) {
  cout << "### Executing Ask " << params << " ... " << endl;

  // waitfor("personhere",run);

  string to_send = "ask_" + params;
  tcp_interface::RCOMMessage message_to_send;
  message_to_send.robotsender= robotname;
  message_to_send.robotreceiver="all";
  message_to_send.value= to_send;
  rcom_pub.publish(message_to_send);

  end_speech=false;
  int sleeptime=SPEECH_TIMEOUT;
  while (*run && sleeptime-->0 && !end_speech && ros::ok())
    ros::Duration(1.0).sleep();
  end_speech=false;

  cout << "### Ask " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::answer(string params, bool *run) {

    cout << "### Executing Answer " << params << " ... " << endl;

    waitforloc(params,run);

    cout << "### Answer " << params << ((*run)?" Completed":" Aborted") << endl;

}

void RCHPNPActionServer::approach(string params, bool *run) {

    cout << "### Executing Approach " << params << " ... " << endl;

    wait("1",run);

    cout << "### Approach " << params << ((*run)?" Completed":" Aborted") << endl;

}

void RCHPNPActionServer::grab(string params, bool *run) {

    cout << "### Executing Grab " << params << " ... " << endl;

    ros::Duration(3.0).sleep();

    cout << "### Grab " << params << ((*run)?" Completed":" Aborted") << endl;

}



void RCHPNPActionServer::enter(string params, bool *run) {

    cout << "### Executing Enter " << params << " ... " << endl;

    double GX,GY;
    if (getDoorEntrancePosition(params,GX,GY)) {
        do_movebase(GX,GY,0,run);
    }
    else 
        ROS_WARN("Move: Cannot find door %s.",params.c_str());

    cout << "### Enter " << params << ((*run)?" Completed":" Aborted") << endl;
}


void RCHPNPActionServer::exit(string params, bool *run) {

    cout << "### Executing Exit " << params << " ... " << endl;

    double GX,GY;
    if (getDoorExitPosition(params,GX,GY)) {
        do_movebase(GX,GY,0,run);
    }
    else 
        ROS_WARN("Move: Cannot find door %s.",params.c_str());

    cout << "### Exit " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::lookfor(string params, bool *run) {

    cout << "### Executing Lookfor " << params << " ... " << endl;

    wait("1",run);
    do_turn("ABS", 135, run); ros::Duration(1.0).sleep();
    do_turn("ABS", 215, run); ros::Duration(1.0).sleep();
    do_turn("ABS", 180, run); ros::Duration(1.0).sleep();

    cout << "### Lookfor " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::waitforloc(string params, bool *run) {

    cout << "### Executing Waitforloc " << params << " ... " << endl;

    vector<string> vparams;
    boost::split(vparams, params, boost::is_any_of("_")); // split parameters

    string xparams = ""; int i=0;
    for (i=0; i<vparams.size()-2; i++)
        xparams += vparams[i] + "_";
    xparams += vparams[i];

    PNPActionServer::waitfor(xparams,run);

    cout << "### Waitforloc " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::sense(string params, bool *run) {

    cout << "### Executing Sense " << params << " ... " << endl;

    vector<string> vparams;
    boost::split(vparams, params, boost::is_any_of("_")); // split parameters

    if (vparams[0]=="persondetected") {
        cout << "  Sense " << vparams[0] << " : false " << endl;
    }

    cout << "### Sense " << params << ((*run)?" Completed":" Aborted") << endl;

}

