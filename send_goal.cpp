#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

int main(int argc, char** argv) {

	char *robot_id = argv[1];

	ros::init(argc, argv, "send_goals");
	ros::NodeHandle nh;

	double goal_x, goal_y, goal_theta;
	string control;

	// Create the string "robot_X/move_base"
	string move_base_str = "/robot_";
	move_base_str += robot_id;
	move_base_str += "/move_base";

	// create the action client
	MoveBaseClient ac(move_base_str, true);

	// Wait for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(5));
	ROS_INFO("Connected to move base server");

	// Check until control paramater is set to "targets"
	nh.getParam("control", control);
	while (control.compare("notargets") == 0) {
		ros::Duration(0.5).sleep();
		nh.getParam("control", control);
	}

	// Loop to visit the goals
	int visited = 0;
	ros::Time begin = ros::Time::now();
	while (control.compare("targets") == 0) {
		if (!nh.getParam("goal_x", goal_x) || !nh.getParam("goal_y", goal_y) || !nh.getParam("goal_theta", goal_theta))
		    cout << " Error receiving goal for robot " << robot_id << endl;
	
		// Send a goal to move_base
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = goal_x;
		goal.target_pose.pose.position.y = goal_y;

		double radians = goal_theta * (M_PI/180);
		tf::Quaternion quaternion;
		quaternion = tf::createQuaternionFromYaw(radians);
		geometry_msgs::Quaternion qMsg;
		tf::quaternionTFToMsg(quaternion, qMsg);
		goal.target_pose.pose.orientation = qMsg;

		ROS_INFO("Sending goal to robot no. %s: x = %f, y = %f", robot_id, goal_x, goal_y);
		ac.sendGoal(goal);
		ac.waitForResult();
		ros::Duration elapsed = ros::Time::now() - begin;

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Robot %s has reached the goal at time %f", robot_id, elapsed.toSec());
		else
			ROS_INFO("The base of robot %s failed for some reason", robot_id);
		nh.setParam("status", ++visited);	// tell auction node to send new goal if any
		ros::Duration(1.0).sleep(); 		// allow new goal coordinates update
		nh.getParam("control", control);
	}
	return 0;
}



