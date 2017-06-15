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
	locationcoords["bedroomin"].set(3.0, 18.0);
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





/*
 * ACTIONS
 */



void RCHPNPActionServer::followperson(string params, bool *run) {

  if (!run) return;

  cout << "### Executing Follow Person " << params << " ... " << endl;

  do_follow_person(run);

  cout << "### Follow Person " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::goto_movebase(string params, bool *run) {

  cout << "### Executing Move " << params << " ... " << endl;

  double GX,GY;
  if (getLocationPosition(params,GX,GY)) {
    do_movebase(GX,GY,0,run);
  }
  else 
    ROS_WARN("Move: Cannot find location %s.",params.c_str());

  cout << "### Move " << params << ((*run)?" Completed":" Aborted") << endl;

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

    waitfor(params,run);

    cout << "### Answer " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::grab(string params, bool *run) {

    cout << "### Executing Grab " << params << " ... " << endl;

    ros::Duration(3.0).sleep();

    cout << "### Grab " << params << ((*run)?" Completed":" Aborted") << endl;

}



void RCHPNPActionServer::enter(string params, bool *run) {

    cout << "### Executing Enter " << params << " ... " << endl;

	
    // TODO
	// do_move(doorcoords[params].enterX,doorcoords[params].enterY);

    cout << "### Enter " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::exit(string params, bool *run) {

    cout << "### Executing Exit " << params << " ... " << endl;

    // TODO

    cout << "### Exit " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::lookfor(string params, bool *run) {

    cout << "### Executing Lookfor " << params << " ... " << endl;

    do_turn("ABS", 135, run);  ros::Duration(1.0).sleep();
    do_turn("ABS", 215, run); ros::Duration(1.0).sleep();
    do_turn("ABS", 180, run); ros::Duration(1.0).sleep();

    cout << "### Lookfor " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::approach(string params, bool *run) {

    cout << "### Executing Approach " << params << " ... " << endl;

    waitfor("HRIreceived",run);

    cout << "### Approach " << params << ((*run)?" Completed":" Aborted") << endl;

}


