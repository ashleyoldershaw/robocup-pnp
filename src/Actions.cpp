#include <boost/thread/mutex.hpp>
#include <tf/transform_listener.h>
#include <tcp_interface/RCOMMessage.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

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
 
void RCHPNPActionServer::exec(string params, bool *run) {
  string::size_type pos;
  pos=params.find('_',0);
  string command=params.substr(0,pos);
  string other=params.substr(pos+1);
  bool * booltrue;
  *booltrue = true;
  actionExecutionThread("diago_0", command, other, booltrue);
}

string recoveryplan = "";
string recoveryactions = "";
string actioninexecution = "";
string failurecondition = "";

void RCHPNPActionServer::rulebuilder(string params, bool *run) {
  // this function is good because it doesn't use ROS at all, so it can be used on other stuff
  // TODO add a way of naming the plan, i.e. if (command == "addname")
  string::size_type pos;
  pos=params.find('_',0);
  string command=params.substr(0,pos);
  string other=params.substr(pos+1);

  if (command == "addaction") {
    actioninexecution = other;
    ROS_INFO("Action chosen: \"%s\"", actioninexecution.c_str());
  }
  else if (command == "addcondition") {
    failurecondition = other;
  }
  else if (command == "addrecoverystep") {
    recoveryactions += other;
    recoveryactions += "; ";
  }
  else if (command == "nameplan") {
    string recoveryplan = "*if* " + failurecondition + " *do* " + recoveryactions + " *confidence* 0.5" + '\n';
    std_msgs::String msg;
    msg.data = recoveryplan;
    ROS_INFO("Recovery plan generated: \"%s\"", msg.data.c_str());
    rulebuilder_pub.publish(msg);
    recoveryactions = "";
    ros::spinOnce();
  }
}


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

void RCHPNPActionServer::turn(string params, bool *run) {

  cout << "### Executing Turn " << params << " ... " << endl;

  vector<string> vparams;
  boost::split(vparams, params, boost::is_any_of("_")); // split parameters

  stringstream ss(vparams[0]);
  float deg;
  ss >> deg;
  string absrel_flag = "REL";
  if (vparams.size()>1)
    absrel_flag = vparams[1];

  do_turn(absrel_flag, deg,run);

  cout << "### Turn " << params << ((*run)?" Completed":" Aborted") << endl;

}



void RCHPNPActionServer::followperson(string params, bool *run) {

  cout << "### Executing FollowPerson " << params << " ... " << endl;

  float max_vel = 0.5;
  if (params!="") {
      vector<string> vparams;
      boost::split(vparams, params, boost::is_any_of("_")); // split parameters
      if (vparams.size()>0) {
          stringstream ss(vparams[0]);
          ss >> max_vel;
      }
  }

  cout << "   do_followperson " << max_vel << " ... " << endl;

  do_followperson(max_vel, run);

  cout << "### FollowPerson " << params << ((*run)?" Completed":" Aborted") << endl;


}



// tell - just as say but with different preconditions
void RCHPNPActionServer::tell(string params, bool *run) {
    say(params,run);
}


string RCHPNPActionServer::sendMODIM(string modimstr) {
  // Send to modim
  char* mip = getenv ("MODIM_IP");
  std::string modim_ip = "127.0.0.1";
  if (mip!=NULL){
    modim_ip = std::string(mip);
    printf ("MODIM_IP defined. Using %s", modim_ip.c_str());
  } else
    printf ("MODIM_IP not defined. Using %s", modim_ip.c_str());

  int modim_port = 9101;
  bool bc = tcpclient.connect(modim_ip.c_str(), modim_port);
  char buffer[200]; buffer[0]='\0';
  if (bc) {
    cout << "Send: " << modimstr << endl;    

    tcpclient.send(modimstr.c_str());
    tcpclient.send(" ###ooo###\n\r");
    
    int br = tcpclient.TCPClient::receive(buffer, 200);
    if (buffer[0]=='u' && buffer[1]=='\'') {
        buffer[0]=' '; buffer[1]=' '; buffer[br-2]='\0';
    }
    else if (buffer[0]=='\'') {
        buffer[0]=' '; buffer[br-2]='\0';
    }
    
  tcpclient.close();
  }

  string r = string(buffer); boost::trim(r);
  size_t p = r.find_first_of("\n\r\t");
  r = r.substr(0,p);
  cout << "Received: [" << r << "]" << endl;    
  return r;

}


void RCHPNPActionServer::sendMODIM_text(string params) {
    ostringstream ss;

    string r = params;
    for (int i=0; i<r.size(); i++) {
        if (r[i]=='_') r[i]=' ';
    }

    ss << "im.display.display_text(\"" << r <<"\",\"default\")" << "\n\r";
    sendMODIM(ss.str());
}


void RCHPNPActionServer::sendMODIM_buttons(string params) {

    sendMODIM("im.display.remove_buttons()");

    ostringstream ss;
    ss << "im.display.display_buttons(";
    if (params.find("drink")!=string::npos) 
        ss << "[ ('coke','Coke'), ('beer','Beer') ]";
    else if (params.find("somethingelse")!=string::npos) 
        ss << "[ ('yes','Yes'), ('no','No') ]";
    else if (params.find("done")!=string::npos) 
        ss << "[ ('done','OK') ]";
    ss << ")\n\r";
    sendMODIM(ss.str());
}

void RCHPNPActionServer::MODIM_init() {
    sendMODIM("im.display.remove_buttons()");
    
    sendMODIM("im.setProfile(['*', '*', 'en', '*'])");

    char* demo_p = getenv ("DEMO_PATH");
    std::string demo_path = "/home/ubuntu/demos/cocktail_party_demo/";
    if (demo_p!=NULL){
       demo_path = std::string(demo_p);
       printf ("DEMO_PATH defined. Using %s", demo_path.c_str());
    } else
       printf ("DEMO_PATH not defined. Using %s", demo_path.c_str());


    sendMODIM("im.setPath('"+demo_path+"')");
    
    sendMODIM_text("Welcome");
}

void RCHPNPActionServer::GUIinit(string params, bool *run) {
    MODIM_init();
  wait("1",run);
}

void RCHPNPActionServer::interact(string params, bool *run) {
  cout << "### Executing Interact " << params << " ... " << endl;

  if (!*run)
      return;

  vector<string> vparams;
  boost::replace_all(params, "000", "+");
  boost::split(vparams, params, boost::is_any_of("+")); // split parameters
  // for (size_t i = 0; i < vparams.size(); i++){
  //   std::cerr << "vparams: " << vparams[i] << std::endl;
  // }
  
  if (vparams.size() == 1){
    //Case interact_action
    std::string action = vparams[0];
    std::string modim_str = "im.execute(\""+action+"\")";
    sendMODIM(modim_str);
  } else if (vparams.size() > 1){
    //Case interact_action_paramsanswer
    std::string action = vparams[0];
    std::string modim_str = "im.ask(\""+action+"\")";
    std::string r = sendMODIM(modim_str);
    std::cout << "Answer: " << r << std::endl;

    std_msgs::String out;
    out.data = r;
    PNP_cond_pub.publish(out);

    std::string paramsanswer = vparams[1];
    for (size_t i = 2; i<vparams.size(); i++)
      paramsanswer = paramsanswer + "_" + vparams[i];

    waitfor(paramsanswer, run);
  }

  cout << "### Interact " << params << ((*run)?" Completed":" Aborted") << endl;
  
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
  
  sendMODIM_text(params);

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

  cout << "TODO DEBUG!!!" << endl;
  
#if 0
// TODO debug

  string to_send = "ask_" + params;
  tcp_interface::RCOMMessage message_to_send;
  message_to_send.robotsender= robotname;
  message_to_send.robotreceiver="all";
  message_to_send.value= to_send;
  rcom_pub.publish(message_to_send);

  sendMODIM_text(params);

  end_speech=false;
  int sleeptime=SPEECH_TIMEOUT;
  while (*run && sleeptime-->0 && !end_speech && ros::ok())
    ros::Duration(1.0).sleep();
  end_speech=false;

  sendMODIM_buttons(params);
#endif

  cout << "### Ask " << params << ((*run)?" Completed":" Aborted") << endl;

}


void RCHPNPActionServer::answer(string params, bool *run) {

    cout << "### Executing Answer " << params << " ... " << endl;

  cout << "TODO DEBUG!!!" << endl;
  
#if 0
// TODO debug

    string r = sendMODIM("im.display.answer()"); // blocking - waiting for reply
    cout << "Answer: " << r << endl;

    vector<string> vparams;
    boost::split(vparams, params, boost::is_any_of("_")); // split parameters

    stringstream ss;
    if (vparams.size()==1)
        ss << r;
    else
        ss << vparams[0] << "_" << r;

    cout << "Publish condition: " << ss.str() << endl;

    std_msgs::String out;
    out.data = ss.str();
    PNP_cond_pub.publish(out);

    // waitfor(params,run);

    sendMODIM("im.display.remove_buttons()");

#endif

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

void RCHPNPActionServer::sense(string params, bool *run) {

    cout << "### Executing Sense " << params << " ... " << endl;

    vector<string> vparams;
    boost::split(vparams, params, boost::is_any_of("_")); // split parameters

    if (vparams[0]=="persondetected") {
        cout << "  Sense " << vparams[0] << " : false " << endl;
    }

    cout << "### Sense " << params << ((*run)?" Completed":" Aborted") << endl;

}






