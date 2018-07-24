#include <boost/algorithm/string.hpp>

#include "RCHPNPAS.h"
#include "topics.h"



using namespace std;

void RCHPNPActionServer::laserobsmapCallback(laser_analysis::LaserObstacleMap msg) {

    // cout << "laser obstacle: " << msg << endl;

    // read parameters
    int min_points;
    double max_dist;
    handle.param<int>("/"+robotname+"/PNPconditions/params/laser_obstacle_min_points", min_points, 2);
    handle.param<double>("/"+robotname+"/PNPconditions/params/laser_obstacle_max_dist", max_dist, 2.0);

    // cout << "Laser obstacle params: min_point: " <<  min_points << " max_dist: " << max_dist << endl;

    bool r = (msg.npoints>=min_points && msg.mx<max_dist);
    string param = PARAM_PNPCONDITIONBUFFER + string("personhere"); 
    handle.setParam(param, r?1:0);


    r = (msg.npoints>2 && msg.npoints<20 && fabs(msg.my)<1.0 && msg.mx<6.0);
    param = PARAM_PNPCONDITIONBUFFER + string("persondetected");
    handle.setParam(param, r?1:0);


}



void RCHPNPActionServer::tcpCallback(tcp_interface::RCOMMessage msg) {
    string sm = msg.value;
    if (sm!="") {
        boost::algorithm::to_lower(sm);
        cout << "Received from ASR: " << sm << endl;


        if (sm=="[end_synth]" || sm.find("[end_synth]")!=string::npos) {
            end_speech=true;
            cout << "END SPEECH " << sm << endl;
            return;
        }
        vector<string> toks;
        boost::split(toks,sm,boost::is_any_of("()\" \n\r"));
        if (toks.size()>1) {
            vector<string>::iterator it = toks.begin();
            string pre = *it++; // should be either 'button' or 'asr'
            string frame = "-";
            if (pre=="asr") {
                frame = *it++;
            }
            string value = "-";
            for ( ; it!=toks.end(); it++)
                if (*it!="") {
                    value = *it; break; 
                }
            cout << "Tokenize from ASR: " << toks.size() << " - " << pre << " - " << frame << " - " << value << endl;
            boost::replace_all(value,"000","_");
            std_msgs::String out;
            out.data = "HRIreceived";
            PNP_cond_pub.publish(out);
            ros::Duration s(0.5);
            s.sleep();
            out.data = value;
            PNP_cond_pub.publish(out);
            cout << "Published PNP condition from ASR: " << out.data << endl;
        }
    }
}


void RCHPNPActionServer::pepperSonarCB(sensor_msgs::Range msg) {
    this->pepper_sonar_range = msg.range;
    bool r = (this->pepper_sonar_range<1.0);
    string param = PARAM_PNPCONDITIONBUFFER + string("obstaclehere"); 
    handle.setParam(param, r?1:0);
    // cout << "Sonar range: " << this->pepper_sonar_range << " -> obstaclehere: " << r << endl;
}

/*
void RCHPNPActionServer::conditionCallback(std_msgs::String msg) {
    // ROS_INFO("Condition received: %s",msg.data.c_str());
    last_condition_received = msg.data;
}
*/



int RCHPNPActionServer::evalCondition(string cond) {

    int r=-1; // -1 default, conditions will be evaluated by PNPConditionEvent variable

    if (cond == "true")  return 1; 

    if (cond == "closetotarget") {
        double RX, RY, RTh_rad; // robot pose 
        double dist=3.0;
        if (getRobotPose(robotname, RX, RY, RTh_rad)) {
            r = ((targetGX-RX)*(targetGX-RX) + (targetGY-RY)*(targetGY-RY) < dist*dist)?1:0;
            // ROS_WARN("evalCondition: closetotarget: R:%.1f %.1f T:%.1f %.1f  -> %d", RX,RY,targetGX,targetGY,r);
        }
        else { 
            ROS_WARN("evalCondition: cannot get robot pose.");
        }
    }

    // when this function returns -1, the condition is evaluated from
    // the events published to PNPConditionEvent or from PNPBuffer variables
    return r;

}


