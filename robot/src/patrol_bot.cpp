// core
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <vector>

// AR addition
# include <geometry_msgs/Twist.h>
# include <geometry_msgs/PoseWithCovarianceStamped.h>
# include <nav_msgs/Odometry.h>
# include <nav_msgs/OccupancyGrid.h>
# include <ar_track_alvar/AlvarMarkers.h>
# include <costmap_2d/costmap_2d_ros.h>
# include <costmap_2d/costmap_2d.h>
# include <math.h>


// Logging addition
#include <iostream>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const std::string MAP_FRAME = "map";
const std::string ODOM_FRAME = "odom";
const std::string ODOM_TOPIC = "odom";
const std::string AR_POSE_TOPIC = "ar_pose_marker";
const std::string AMCL_TOPIC = "amcl_pose";
const std::string INITIAL_POSE_TOPIC = "initialpose";
const std::string VELOCITY_TOPIC = "mobile_base/commands/velocity";




// Remote control
const std::string PATROL_STATUS_TOPIC = "turtlebot_patrol_status";
const std::string PATROL_COMMAND_TOPIC = "turtlebot_patrol_command";
const std::string ROS_ERROR = "[ROS ERROR]";
const std::string SUCCESS = "success";
const std::string CMD_STANDBY = "standby";



tfScalar current_x;
tfScalar current_y;

bool REMOTE_CONTROL = false;

MoveBaseClient *ac_ptr = NULL;


// autonomous mode
int ROUNDS = 3;
std::vector<tfScalar> x_locations;
std::vector<tfScalar> y_locations;
std::vector<std::vector<tfScalar> > dist;


ros::Publisher status_pub;
ros::Subscriber cmd_sub;

// Logging 
int num_errors = 0;
time_t timev;
char times[1000];
std::ofstream *log_file;


// Core Methods
void fillInRequest(tfScalar start_x, tfScalar start_y, tfScalar end_x, tfScalar end_y, nav_msgs::GetPlan &srv);
tfScalar makePlan(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);
bool achieveGoal(tfScalar x, tfScalar y, MoveBaseClient &ac);



// core methods
void fillInRequest(tfScalar start_x, tfScalar start_y, tfScalar end_x, tfScalar end_y, nav_msgs::GetPlan &srv){
	srv.request.start.header.frame_id="map";
	srv.request.start.pose.position.x = start_x;
	srv.request.start.pose.position.y = start_y;
	srv.request.start.pose.orientation.w = 1.0;
	srv.request.goal.header.frame_id="map";
	srv.request.goal.pose.position.x = end_x;
	srv.request.goal.pose.position.y = end_y;
	srv.request.goal.pose.orientation.w = 1.0;
	srv.request.tolerance = 2;
}

tfScalar makePlan(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv){
	tfScalar distance = 0;
	if(serviceClient.call(srv)){
		if(!srv.response.plan.poses.empty()){
			for(int i = 1; i < srv.response.plan.poses.size(); i++){
				geometry_msgs::PoseStamped prev = srv.response.plan.poses[i-1];
				geometry_msgs::PoseStamped curr = srv.response.plan.poses[i];
				distance = distance + sqrt(pow(curr.pose.position.x - prev.pose.position.x, 2) + pow(curr.pose.position.y - prev.pose.position.y, 2));
			}
		}else{
			ROS_WARN("got empty plan");
		}
		return distance;
	}else{
		ROS_FATAL("could not call move_base/make_plan service");
		exit(-1);
	}
}

bool achieveGoal(tfScalar x, tfScalar y, MoveBaseClient &ac){
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();


	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.orientation.w = 1;
	ac.sendGoal(goal);
	ac.waitForResult();
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	}
	else{
		ROS_WARN("The base failed for some reason.");
		return false;
	}
	return true;
}

void getTime(char *s){
	time(&timev);
	struct tm * p = localtime(&timev);
    strftime(s, 1000, "%c", p);
}

template <typename T>
  std::string NumberToString ( T Number )
  {
     std::ostringstream ss;
     ss << Number;
     return ss.str();
  }

void report_status(std::string status){
	 getTime(times);

	 std_msgs::String msg;
	 std::stringstream ss;
	 ss << status;
	 msg.data = ss.str();

	 status_pub.publish(msg);
	 *log_file <<  timev << " " << status.c_str() << "\n";
	 log_file->flush();
	 ROS_INFO(status.c_str());
}

void cmdCallback(const std_msgs::String::ConstPtr& msg)
{
	// format of goal is "x y"
	char* pEnd;
	double goal_x, goal_y;
	goal_x = strtod (msg->data.c_str(), &pEnd);
	goal_y = strtod (pEnd, NULL);

	report_status("Goto (" + NumberToString(goal_x) + " ," + NumberToString(goal_y) + ")");

	bool succeed = false;
	succeed = achieveGoal(goal_x, goal_y, *ac_ptr);
    if(succeed){
    	report_status(SUCCESS);
    }
    else{
    	report_status(ROS_ERROR);
    }
}

int main(int argc, char **argv){

	ros::init(argc, argv, "ata_navigation_node");

	ros::NodeHandle nh;

	nh.getParam("REMOTE_CONTROL", REMOTE_CONTROL);
	nh.getParam("x_locations", x_locations);
	nh.getParam("y_locations", y_locations);
	nh.getParam("ROUNDS", ROUNDS);
	
	// Initialize communication with patrol manager
	status_pub = nh.advertise<std_msgs::String>(PATROL_STATUS_TOPIC, 1000);

	// Create log for this patrol
	getTime(times);
	std::string log_name("/home/yulun/catkin_ws/src/ata_navigation/log/");
	log_name.append(times);
	log_name.append("_patrol_log.txt");
	std::ofstream log(log_name.c_str());
	log_file = &log;

	for(int i = 0; i < 10; i++){
		ros::spinOnce();
	}


	MoveBaseClient ac("move_base", true);
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(60));
	ROS_INFO("Connected to move_base server");
	ac_ptr = &ac;
    const int size = x_locations.size();


	// determine robot's initial pose
	tf::TransformListener poseTFListener;
	tf::StampedTransform posetransform;
	try{
		ROS_INFO("Wait for initial position");
		poseTFListener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
		poseTFListener.lookupTransform("/map", "/base_link", ros::Time(0),posetransform);
		current_x = posetransform.getOrigin().x();
		current_y = posetransform.getOrigin().y();
		ROS_INFO("Initial robot position: %f, %f", current_x, current_y);

	}catch(tf::TransformException &ex){
		ROS_ERROR("Can't determine initial position");
		return -1;
	}

	// Autonomous mode
	if(!REMOTE_CONTROL){
		bool succeed = false;
		int round = 0;

		while(nh.ok() && round < ROUNDS){
			for(int i = 0 ; i < size; i++){


				succeed = achieveGoal(x_locations[i], y_locations[i], *ac_ptr);

				if(succeed){
					report_status(NumberToString(i + 1) + " success");
				}else{
					report_status(NumberToString(i + 1) + " error");
				}
				ros::Duration(30).sleep();

			}
			round++;
		}

		log_file->close();
	}
	else{ // remote control mode
		cmd_sub = nh.subscribe(PATROL_COMMAND_TOPIC, 1000, cmdCallback);
		ros::spin();
	}
	

	return 0;
}

