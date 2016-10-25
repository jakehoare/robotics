
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>
using namespace std;

#define ROBOTS 10	// number of agents
#define TARGETS 50	// number of targets
#define SPEED 0.65	// linear movement speed
#define ANGULAR 1.0	// rotational movement speed

double goalTolerance = 0.5;
string worldFrame = "map";

double robot_locations[ROBOTS][2];	// start positions
double target_locations[TARGETS][2];	// targets to be visited
bool allocated_target[TARGETS] = {0};	// flag, initially all are unallocated
vector<int> allocations[ROBOTS];	// allocated targets for each robot

void fillPathRequest(nav_msgs::GetPlan::Request &req, int robot, int begin, int end);
double callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv, bool printout);
void setLocations();
string paramString(int robot, string param);

int main(int argc, char** argv) {

	ros::init(argc, argv, "auction");
	ros::NodeHandle nh;

	string service_name = "/robot_0/move_base_node/make_plan";
	while (!ros::service::waitForService(service_name, ros::Duration(3.0))) {
		ROS_INFO("Waiting for service make_plan to become available");
	}

	ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
	if (!serviceClient) {
		ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());
		return -1;
	}

	// Load the initial and target locations
	setLocations();

	nav_msgs::GetPlan srv;

	int unallocated = TARGETS;
	double bid;
	double lowest_bid;
	int lowest_bidder;
	int lowest_target;
	int lowest_position;
	int start_position;
	double path_cost[ROBOTS] = {0.0};
	double psi_bids[ROBOTS][TARGETS] = {0.0};
	std::vector<int>::iterator it;

	if (argc > 1) {
		cout << "*** RUNNING SEQUENTIAL SINGLE ITEM AUCTION ***";
		if (argc == 2)
			cout << " (SIMPLE)" << endl;
		else 
			cout << " (FULL)" << endl;
		// SEQUENTIAL SINGLE ITEM AUCTION
		bool display = false;
		while (unallocated != 0) {
			lowest_bid = 999.0;
			lowest_bidder = -1;
			lowest_target = -1;
			lowest_position = -1;
			for (int robot = 0; robot < ROBOTS; ++robot) {
				for (int target = 0; target < TARGETS; ++target) {
					if (allocated_target[target])
						continue;
					// argc>2 means target can be inserted anywhere in current list of allocations for that robot
					// else target can only be added to end of current list
					if (argc > 2)
						start_position = 0;
					else
						start_position = allocations[robot].size();
					// Put target in all positions in current target list
					for (int position = start_position; position <= allocations[robot].size(); ++position) {
						// Calc cost from previous target (or start pos if none)
						if (position == 0)
							fillPathRequest(srv.request, robot, -1, target);
						else
							fillPathRequest(srv.request, robot, allocations[robot][position-1], target);
						bid = callPlanningService(serviceClient, srv ,display);
						// Add cost from target to next target (if any)
						if (position < allocations[robot].size()) {
							fillPathRequest(srv.request, robot, target, allocations[robot][position]);
							bid += callPlanningService(serviceClient, srv ,display);
							// Subtract cost between previous and next targets
							if (position == 0)
								fillPathRequest(srv.request, robot, -1, allocations[robot][position]);
							else
								fillPathRequest(srv.request, robot, allocations[robot][position-1], allocations[robot][position]);
							bid -= callPlanningService(serviceClient, srv ,display);

						}
						bid += path_cost[robot];
						//cout << "Robot " << robot << " bids " << bid << " for target " << target << " at pos " << position << endl;
						if (bid < lowest_bid) {
							lowest_bid = bid;
							lowest_bidder = robot;
							lowest_target = target;
							lowest_position = position;
						}
					}
				}
			}
			// End of of bidding - announce winner
			cout << "  Winner is " << lowest_bidder << " with " << lowest_bid << " for target " << lowest_target << " at pos " << lowest_position << endl;
			allocated_target[lowest_target] = true;
			--unallocated;
			it = allocations[lowest_bidder].begin();
			allocations[lowest_bidder].insert(it + lowest_position, lowest_target);
			path_cost[lowest_bidder] = lowest_bid;
		}
	} else {
		cout << "*** RUNNING PARALLEL SINGLE ITEM AUCTION ***" << endl;
		// PARALLEL SINGLE ITEM AUCTION	
		for (int target = 0; target < TARGETS; ++target) {
			lowest_bid = 999.0;		
			lowest_bidder = -1;
			lowest_target = -1;
			for (int robot = 0; robot < ROBOTS; ++robot) {
				// Get the path
				fillPathRequest(srv.request, robot, -1, target);
				if (!serviceClient) {
					ROS_FATAL("Connection to %s failed", serviceClient.getService().c_str());
					return -1;
				}
				psi_bids[robot][target] = callPlanningService(serviceClient, srv, false);
				if (psi_bids[robot][target] < lowest_bid) {
					lowest_bid = psi_bids[robot][target];
					lowest_bidder = robot;
					lowest_target = target;
				}
				//cout << "Robot " << robot << " bids " << psi_bids[robot][target] << " for target " << target << endl;				
			}
			// End of of bidding - announce winner		
			cout << "  Winner is " << lowest_bidder << " with " << lowest_bid;
			cout << " for target " << lowest_target << endl;
			allocations[lowest_bidder].push_back(lowest_target);
		}
		// For each robot calculate the path cost from allocated targets
		int begin, end;
		for (int robot = 0; robot < ROBOTS; ++robot) {
			begin = -1;
			for (int robotTarget = 0; robotTarget < allocations[robot].size(); ++robotTarget) {				
				end = allocations[robot][robotTarget];				
				fillPathRequest(srv.request, robot, begin, end);
				path_cost[robot] += callPlanningService(serviceClient, srv, false);
				begin = end;
			}
		}
	
	}

	// Display paths and costs
	for (int robot = 0; robot < ROBOTS; ++robot) {
		cout << "Cost for robot " << robot << " is " << path_cost[robot] << " to visit targets ";
		for (int robotTarget = 0; robotTarget < allocations[robot].size(); ++robotTarget)
			cout << allocations[robot][robotTarget] << " ";
		cout << endl;
	}
	double max_cost = 0.0;
	for (int robot = 0; robot < ROBOTS; ++robot) {
		if (path_cost[robot] > max_cost)
			max_cost = path_cost[robot];
	}
	cout << "Maximum cost is " << max_cost << endl;  

	return 0;	// TO RUN AUCTION ONLY, END HERE

	// Now send robots to targets
	int target;
	int status;
	while (true) {
		// Check each robot
		for (int robot = 0; robot < ROBOTS; ++robot) {
			nh.getParam(paramString(robot, "status"), status);
			if (status >= allocations[robot].size()) 	// No more targets to be visited
				nh.setParam(paramString(robot, "control"), "notargets");  
			else {	// Update next target
				nh.setParam(paramString(robot, "control"), "targets");
				target = allocations[robot][status];
				//cout << "Current target for robot " << robot << " is " << target << endl;
				nh.setParam(paramString(robot, "goal_x"), target_locations[target][0]);
				nh.setParam(paramString(robot, "goal_y"), target_locations[target][1]);
				nh.setParam(paramString(robot, "goal_theta"), 0);
			}
			ros::Duration(0.1).sleep();
		}
	}

}

void fillPathRequest(nav_msgs::GetPlan::Request &request, int robot, int begin, int end) {

	request.start.header.frame_id = worldFrame;
	if (begin == -1) {
		request.start.pose.position.x = robot_locations[robot][0];
		request.start.pose.position.y = robot_locations[robot][1];
	} else {
		request.start.pose.position.x = target_locations[begin][0];
		request.start.pose.position.y = target_locations[begin][1];
	}
	request.start.pose.orientation.w = 1.0;

	request.goal.header.frame_id = worldFrame;
	request.goal.pose.position.x = target_locations[end][0];
	request.goal.pose.position.y = target_locations[end][1];
	request.goal.pose.orientation.w = 1.0;

	request.tolerance = goalTolerance;
	//cout << "Target: " << target << " Robot: " << robot;
	//cout << " Calculating path from " << request.start.pose.position.x << ", " << request.start.pose.position.y 
//		<< " to " << request.goal.pose.position.x << ", " << request.goal.pose.position.y << endl;
}

double callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv, bool printout) {
	double cost = 0.0;
	double a_cost = 0.0;
	double prev_angle = 0.0;
	double x_move, y_move, prev_x, prev_y, angle;
	geometry_msgs::PoseStamped p;

	if (serviceClient.call(srv)) {
		if (!srv.response.plan.poses.empty()) {
			prev_x = srv.response.plan.poses[0].pose.position.x;
			prev_y = srv.response.plan.poses[0].pose.position.y;
			for (int i = 1; i < srv.response.plan.poses.size() - 2; ++i) {
				p = srv.response.plan.poses[i];
				x_move = p.pose.position.x - prev_x;
				y_move = p.pose.position.y - prev_y;
				cost += sqrt(pow(x_move, 2) + pow(y_move, 2));
				// angle is between 0 and 360
				angle = atan(y_move/x_move) * (180.0/M_PI);
				if (x_move < 0.0 && angle > 0.0)
					angle -= 180.0;
				else if (x_move < 0.0 && angle <= 0.0)
					angle += 180.0;
				if (abs(angle-prev_angle) > 180.0) {
					if (prev_angle > 0.0)
						prev_angle -= 360.0;
					else
						prev_angle += 360.0;
				}
				a_cost += abs(angle-prev_angle);
				if (printout) {
					cout << "  Position: " << p.pose.position.x << " " << p.pose.position.y << " " << endl;
					cout << "   Move (x, y, theta, theta change): " << x_move*100 << ", " << y_move*100 << ", " << angle << ", " << (angle-prev_angle) << "  " << a_cost << endl;
					cout << "    Costs:" << cost << ", " << a_cost << endl;
				}
				prev_x = p.pose.position.x;
				prev_y = p.pose.position.y;
				prev_angle = angle;
			}
		}
		else {
			ROS_WARN("Empty plan");
		}
	}
	else {
		ROS_ERROR("Failed to call service %s - is the robot moving?", serviceClient.getService().c_str());
	}
	a_cost += abs(prev_angle);	// Add angular cost of rotation back to zero degrees
	a_cost *= (M_PI/180.0);
	cost = (cost/SPEED) + (a_cost/ANGULAR);
	if (printout)
		cout << "Final cost: " << cost << endl;
	return cost;
}

void setLocations()
{
/*
	// USED FOR FLOORPLAN
	robot_locations[0][0] = 16.0;	// x coord
	robot_locations[0][1] = 10.0;	// y coord
	robot_locations[1][0] = 9.5;
	robot_locations[1][1] = 15.5;
	robot_locations[2][0] = 18.0;
	robot_locations[2][1] = 18.0;

	target_locations[0][0] = 28.0;
	target_locations[0][1] = 15.0;
	target_locations[1][0] = 14.5;
	target_locations[1][1] = 10.0;
	target_locations[2][0] = 16.5;
	target_locations[2][1] = 16.5;
	target_locations[3][0] = 7.0;
	target_locations[3][1] = 18.0;
	target_locations[4][0] = 15.5;
	target_locations[4][1] = 10.0;
	target_locations[5][0] = 10.0;
	target_locations[5][1] = 12.0;
	target_locations[6][0] = 22.0;
	target_locations[6][1] = 17.0;
	target_locations[7][0] = 28.0;
	target_locations[7][1] = 11.0;
*/
/*
	// USED FOR MAZE
	robot_locations[0][0] = 6.5;	// x coord
	robot_locations[0][1] = 36.0;	// y coord
	robot_locations[1][0] = 6.5;
	robot_locations[1][1] = 51.0;

	target_locations[0][0] = 4.0;
	target_locations[0][1] = 48.0;
	target_locations[1][0] = 4.0;
	target_locations[1][1] = 51.0;
	target_locations[2][0] = 9.0;
	target_locations[2][1] = 36.0;
	target_locations[3][0] = 9.0;
	target_locations[3][1] = 45.0;
*/

	// RANDOM
	// srand (time(NULL));
	srand(2);
	for (int robot = 0; robot < ROBOTS; ++robot) {
		robot_locations[robot][0] = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/50.0));	
		robot_locations[robot][1] = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/50.0));
		cout << robot <<" loc (x,y) " << robot_locations[robot][0] << " " << robot_locations[robot][1] << endl;
	}

	for (int target = 0; target < TARGETS; ++target) {
		target_locations[target][0] = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/50.0));
		target_locations[target][1] = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/50.0));
		cout << target << " tgt (x,y) " << target_locations[target][0] << " " << target_locations[target][1] << endl;
	}

}


string paramString(int robot, string param) {
	return "/robot_" + boost::lexical_cast<std::string>(robot) + "/"+ param;
}




