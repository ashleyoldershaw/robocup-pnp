#ifndef __RCHPNPAS_H__
#define __RCHPNPAS_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <pnp_ros/PNPActionServer.h>
#include <rococo_navigation/TurnAction.h>
#include <rococo_navigation/FollowCorridorAction.h>
#include <rococo_navigation/FollowPersonAction.h>

#include <tcp_interface/RCOMMessage.h>
#include <laser_analysis/LaserObstacleMap.h>

#include <map>
#include <boost/thread/thread.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)

struct LocationCoords {

	double X, Y, TH_rad, TH_deg;

	void set(double x, double y, double th_deg) {
		X=x; Y=y; TH_rad=RAD(th_deg); TH_deg=th_deg;
	}

	void set(double x, double y) {
		X=x; Y=y; TH_rad=-999; TH_deg=-999;
	}

};


struct DoorCoords {
	double enterX, enterY, exitX, exitY;
	void set(double x1, double y1, double x2, double y2) {
		enterX=x1; enterY=y1; exitX=x2; exitY=y2;
	}
};


class RCHPNPActionServer : public PNPActionServer
{
private:
    ros::NodeHandle handle, handlep;
    ros::Publisher event_pub, plantoexec_pub, hri_pub, rcom_pub;
    tf::TransformListener* listener;

    // action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_movebase;
    actionlib::SimpleActionClient<rococo_navigation::TurnAction> *ac_turn;
    actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction> *ac_followcorridor;
    actionlib::SimpleActionClient<rococo_navigation::FollowPersonAction> *ac_followperson;

    // condition subscribers
    ros::Subscriber laser_obsmap_sub; // receiving data from laser_analysis
    ros::Subscriber tcp_sub; // receiving data from tcp_interface
    ros::Subscriber cond_sub; // receiving data from PNP condition
    ros::Subscriber active_places_sub; // receiving labels of active places in PNP

    ros::Publisher PNP_cond_pub, stage_say_pub;
    
    std::string robotname;
    boost::mutex mtx_movebase;

    double targetGX, targetGY, targetGTh_deg; // current goal target set by goto action

	map<string, DoorCoords> doorcoords; // door coordinates
	map<string, LocationCoords> locationcoords; // location coordinates

public:

    RCHPNPActionServer(ros::NodeHandle n);

    // Get current robot pose
    bool getRobotPose(string robotname, double &x, double &y, double &th_rad);

    // Get coordinates of semantic locations and doors
    bool getLocationPosition(string loc, double &GX, double &GY);
    bool getDoorEntrancePosition(string loc, double &GX, double &GY);
    bool getDoorExitPosition(string loc, double &GX, double &GY);

	// init Door coordinates
	void initDoors();
	// init Location coordinates
	void initLocations();

    /*
     * ACTIONS
     */
    void goto_movebase(string params, bool *run);
    void enter(string params, bool *run);
    void exit(string params, bool *run);
    void tell(string params, bool *run);
    void say(string params, bool *run);
    void lookfor(string params, bool *run);
    void ask(string params, bool *run);
	void answer(string params, bool *run);
	void grab(string params, bool *run);
	void sense(string params, bool *run);
	void approach(string params, bool *run);
	void waitforloc(string params, bool *run);



	// ACTION BASE UTILS
    void clear_costmaps();
    void do_movebase(float GX, float GY, float GTh_DEG, bool *run);
    void do_turn(string absrel_flag, float GTh_DEG, bool *run);

    /*
     * CONDITIONS FUNCTIONS AND CALLBACKS
     */
    
    virtual int evalCondition(string cond);
//    void conditionCallback(std_msgs::String msg);

    void laserobsmapCallback(laser_analysis::LaserObstacleMap msg);
    void tcpCallback(tcp_interface::RCOMMessage msg);
    void active_places_callback(const std_msgs::String::ConstPtr& msg);
    
//    string last_condition_received;
    bool end_speech;

};

#endif

