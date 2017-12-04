#include <math.h>
#include <iostream>
#include "path_planner.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"

// Constructor
PathPlanner::PathPlanner()
{
  max_vel_ = 22.275;
  desired_vel_ = max_vel_;
  vel_ = 0;
  lane_ = 2;
  d_ = 2 + 4* (lane_ - 1);
  tra_d_ = d_;
  all_states_ = {"KL", "LCL", "LCR", "DLCL", "DLCR"}; // The only valid state is keep lane. All lane changes are transitions to a new KL state
  // Double lane change just means that if the vehicle performs 2 sequential lane changes the cost will be minimal. It will not actually change 2 lines in one go
  num_lanes_ = 3;
  state_transition_over = true;
}


// Update states to the current vehicle state
void PathPlanner::updateStates(double x, double y, double s, double d, double yaw, double vel)
{
  x_ = x; y_ = y; s_ = s; d_ = d; yaw_ = yaw;
}


// Update map information
void PathPlanner::updateMap(const vector<double>& m_x, const vector<double>& m_y, const vector<double>& m_s, const vector<double>& m_dx, const vector<double>& m_dy)
{
  map_x_ = m_x; map_y_ = m_y; map_s_ = m_s; map_dx_ = m_dx; map_dy_ = m_dy;
}


// Generate a new trajectory and send it back to simulator, idea taken from the Udacity project walkthrough
void PathPlanner::newTrajectory(vector<double>& next_x, vector<double>& next_y, vector<vector<double> >& sensor_fusion)
{
  // Check whether the vehicle is currently making a lane change
  if(state_transition_over)
    decideNewState(sensor_fusion); // Will set desired_vel_ and d_ to match the best new state

  // Avoid collision
  collisionAvoidance(sensor_fusion);

  // Simple P controller to match the desired speed
  double P = 0.08;
  if(abs(desired_vel_ - vel_) >= 15)
    P = 0.03;
  else if(abs(desired_vel_ - vel_) >= 10)
    P = 0.035;
  else if(abs(desired_vel_ - vel_) >= 5)
    P = 0.05;

  vel_ += P * (desired_vel_ - vel_);

  // If the current d is close to the final d of the new trajectory the state transition is over
  if(abs(tra_d_ - d_) < 0.5)
    state_transition_over = true;

  double fps = 50; // Simulator frames per second
  int trajectory_length = 50;
  int current_trajectory_length = next_x.size(); // Number of points left from the previous trajectory

  // Generate a set of anchor points for the spline interpolation
  vector<double> anchors_x, anchors_y;
  double x_ref, y_ref, yaw_ref;

  if(current_trajectory_length < 2) // If previous trajectory is small use vehicle position
  {
    x_ref = x_;
    y_ref = y_;
    yaw_ref = yaw_;

    // Use the vehicle position and angle to get a point behind the vehicle, this enables a smooth transition
    double prev_x = x_ref - cos(yaw_ref);
    double prev_y = y_ref - sin(yaw_ref);

    anchors_x.push_back(prev_x);
    anchors_x.push_back(x_ref);

    anchors_y.push_back(prev_y);
    anchors_y.push_back(y_ref);
  }
  else
  { // If there are points left from the previous trajectory use the last two, and calculate the angle
    x_ref = next_x[current_trajectory_length - 1];
    y_ref = next_y[current_trajectory_length - 1];

    double prev_x = next_x[current_trajectory_length - 2];
    double prev_y = next_y[current_trajectory_length - 2];

    yaw_ref = atan2(y_ref - prev_y, x_ref - prev_x);

    anchors_x.push_back(prev_x);
    anchors_x.push_back(x_ref);

    anchors_y.push_back(prev_y);
    anchors_y.push_back(y_ref);
  }

  // Add three additional points: 50, 70, 90m in front of the vehicle (in Frenet coordinates)
  vector<double> xy_pos_1 = getXY(s_ + 50, tra_d_);
  vector<double> xy_pos_2 = getXY(s_ + 70, tra_d_);
  vector<double> xy_pos_3 = getXY(s_ + 90, tra_d_);

  anchors_x.push_back(xy_pos_1[0]);
  anchors_x.push_back(xy_pos_2[0]);
  anchors_x.push_back(xy_pos_3[0]);

  anchors_y.push_back(xy_pos_1[1]);
  anchors_y.push_back(xy_pos_2[1]);
  anchors_y.push_back(xy_pos_3[1]);

  // Transform the anchor points to the vehicle coordinate frame
  for(int i(0); i < anchors_x.size(); i++)
  {
    double shift_x = anchors_x[i] - x_ref;
    double shift_y = anchors_y[i] - y_ref;

    anchors_x[i] = shift_x * cos(-yaw_ref) - shift_y * sin(-yaw_ref);
    anchors_y[i] = shift_x * sin(-yaw_ref) + shift_y * cos(-yaw_ref);
  }

  // Create a spline class instance and assign the anchor points
  tk::spline s;
  s.set_points(anchors_x, anchors_y);

  // In order to match the velocity of the vehicle we need to space the points of the trajectory accordingly.
  // We used a linear approximation of the current vehicle position and the final target position on the spline.
  // The distance between these two points is then used to space the points to match the vehicle velocity.
  double target_x = 25;
  double target_y = s(target_x);

  double target_distance  = distance(0, 0, target_x, target_y);
  double N_divisions = target_distance / (vel_ * (1/fps));
  double current_x = 0; // In the vehicle coordinate frame
  double current_y;

  // Append new points in the map frame to the previous trajectory
  for(int i(0); i < trajectory_length - current_trajectory_length; i++)
  {
    current_x += target_x/N_divisions;
    double current_y = s(current_x);

    // Transform back to map coordinate frame
    double current_map_x = current_x * cos(yaw_ref) - current_y * sin(yaw_ref) + x_ref;
    double current_map_y = current_x * sin(yaw_ref) + current_y * cos(yaw_ref) + y_ref;

    next_x.push_back(current_map_x);
    next_y.push_back(current_map_y);
  }
}


// Get closest way point on the map
int PathPlanner::closestWaypointIndex(double x, double y)
{
	double closestLen = -1;  // Distance is always >= 0
	int closestWaypoint = 0;

	for(int i = 0; i < map_x_.size(); i++)
	{
		double map_x = map_x_[i];
		double map_y = map_y_[i];
		double dist = distance(x,y,map_x,map_y);
		if(closestLen == -1 || dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> PathPlanner::getFrenet(double x, double y, double theta)
{
	int next_wp = nextWaypoint(x,y, theta);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = map_x_.size()-1;
	}

	double n_x = map_x_[next_wp]-map_x_[prev_wp];
	double n_y = map_y_[next_wp]-map_y_[prev_wp];
	double x_x = x - map_x_[prev_wp];
	double x_y = y - map_y_[prev_wp];

	// Find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	// See if d value is positive or negative by comparing it to a center point
	double center_x = 1000-map_x_[prev_wp];
	double center_y = 2000-map_y_[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
		frenet_d *= -1;

	// Calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
		frenet_s += distance(map_x_[i],map_y_[i],map_x_[i+1],map_y_[i+1]);

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> PathPlanner::getXY(double s, double d)
{
	int prev_wp = -1;

	while(s > map_s_[prev_wp+1] && (prev_wp < (int)(map_s_.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%map_x_.size();

	double heading = atan2((map_y_[wp2]-map_y_[prev_wp]),(map_x_[wp2]-map_x_[prev_wp]));
	// The x,y,s along the segment
	double seg_s = (s-map_s_[prev_wp]);

	double seg_x = map_x_[prev_wp]+seg_s*cos(heading);
	double seg_y = map_y_[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}


// Calculate distance between two points
double PathPlanner::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


// Get next way point from current position
int PathPlanner::nextWaypoint(double x, double y, double theta)
{
	int closestWaypoint = closestWaypointIndex(x,y);

	double map_x = map_x_[closestWaypoint];
	double map_y = map_y_[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*M_PI - angle, angle);

  if(angle > M_PI/4)
  {
    closestWaypoint++;
    if (closestWaypoint == map_x_.size())
      closestWaypoint = 0;
  }

  return closestWaypoint;
}


// Get available states from the current state
vector<string> PathPlanner::getAvailableStates()
{
  vector<string> return_states = {"KL"};

  if(lane_ < num_lanes_)
    return_states.push_back("LCR");
  if(lane_ > 1)
    return_states.push_back("LCL");
  if(lane_ == 1)
    return_states.push_back("DLCR");
  if(lane_ == num_lanes_)
    return_states.push_back("DLCL");

 return return_states;
}


// Decide what is the best next action
void PathPlanner::decideNewState(vector<vector<double> >& sensor_fusion_)
{
  desired_vel_ = max_vel_;

  vector<string> available_states = getAvailableStates();
  // Each entry of the possible states will have ahead_vehicle_vel, distance to new lane, distance to ahead vehicle and new lane
  vector<vector<double> > possible_states;
  vector<string> possible_states_str; // The string value of the possible states
  // A double lane change is not reachable if a regular lane change is not reachable
  bool LCL_reachable = true;
  bool LCR_reachable = true;
  // If we decide to change lanes because of a double lane change maneuver, than use the vehicle speed from the normal lane change
  double DLC_ahead_vel;

  for (int a = 0; a < available_states.size(); a++)
  {
    // Possible and final d and lane will only differ in a double lane change, because we are checking 2 lanes left or right but we will move only one in the end, if it is possible.
    double possible_d;
    double final_d;
    double possible_lane;
    double final_lane;

    if(available_states[a] == "KL")
    {
      possible_lane = lane_;
      final_lane = possible_lane;
    }
    else if(available_states[a] == "LCL")
    {
      possible_lane = lane_ - 1;
      final_lane = possible_lane;
    }
    else if(available_states[a] == "LCR")
    {
      possible_lane = lane_ + 1;
      final_lane = possible_lane;
    }
    else if(available_states[a] == "DLCL" && LCL_reachable)
    {
      possible_lane = lane_ - 2;
      final_lane = possible_lane + 1;
    }
    else if(available_states[a] == "DLCR" && LCR_reachable)
    {
      possible_lane = lane_ + 2;
      final_lane = possible_lane - 1;
    }

    possible_d = 2 + 4* (possible_lane - 1);
    final_d = 2 + 4* (final_lane - 1);

    // Set the initial values for the lanes, if no vehicle ahead is detected we will pass this values to the cost eval
    double ahead_vehicle_vel = max_vel_;
    double distance_s = 1000;

    bool can_reach_this_state = true;

    // Check sensor fusion data
    for (int f = 0; f < sensor_fusion_.size(); f++)
    {
      double sf_vx = sensor_fusion_[f][3];
      double sf_vy = sensor_fusion_[f][4];
      double sf_s = sensor_fusion_[f][5];
      double sf_d = sensor_fusion_[f][6];

      double dist_sf = sf_s - s_;
      double dist_sf_d_abs = abs(possible_d - sf_d);

      if(available_states[a] != "KL" && dist_sf < 15 && dist_sf > -5 && dist_sf_d_abs < 1.0) // If we want to change lanes and there is a vehicle blocking that lane then don't allow that motion
      {
        // cout<<" NA - "<<available_states[a]<<" s: "<< dist_sf <<endl;
        can_reach_this_state = false;

        if(available_states[a] == "LCL")
          LCL_reachable = false;
        else
          LCR_reachable = false;

        break;
      }


      if(dist_sf_d_abs < 1.0 && dist_sf > 0 && dist_sf < 50) // If the sensor fusion vehicle is in the same lane and we are considering
      {
        // Transform vehicle velocities to Frenet velocities vd and vs, idea taken from the Udacity forum: https://discussions.udacity.com/t/calculation-of-speed-in-frenet-space/451483
        int next_waypoint_index = nextWaypoint(x_, y_, yaw_);
        double sf_vd = sf_vx * map_dx_[next_waypoint_index] + sf_vy * map_dy_[next_waypoint_index];

        double sf_vs = sqrt(sf_vx * sf_vx + sf_vy * sf_vy - sf_vd * sf_vd);

        if(sf_vs < ahead_vehicle_vel)
          ahead_vehicle_vel = sf_vs + 3; // Go closer to the front vehicle but don't exceed max vel. Useful when we do an overtake later

        if(ahead_vehicle_vel > max_vel_)
          ahead_vehicle_vel = max_vel_;

        if(dist_sf < distance_s)
          distance_s = dist_sf;

        if(available_states[a] != "KL")
        {
          DLC_ahead_vel = ahead_vehicle_vel;
        }
      }
    }

    if(can_reach_this_state)
    {
      // Append possible state values and names
      if(available_states[a] == "DLCL" || available_states[a] == "DLCR")
        possible_states.push_back({DLC_ahead_vel, final_d, distance_s, final_lane});
      else
        possible_states.push_back({ahead_vehicle_vel, final_d, distance_s, final_lane});

      possible_states_str.push_back(available_states[a]);
    }
  }

  int best_cost_indx = returnBestCostIndex(possible_states, possible_states_str);

  if(possible_states_str[best_cost_indx] != "KL")
  {
    cout<<" --- Taking action: "<< possible_states_str[best_cost_indx]<<endl;
  }

  desired_vel_ = possible_states[best_cost_indx][0]; // Assign new desired_vel_ from best state

  if(tra_d_ != possible_states[best_cost_indx][1])   // Assign new d_ to trajectory and start transition
  {
    state_transition_over = false;
    tra_d_ = possible_states[best_cost_indx][1];
    lane_ = possible_states[best_cost_indx][3];
  }
}


// Evaluate the cost and return the lowest one
int PathPlanner::returnBestCostIndex(vector<vector<double> >& possible_states, vector<string>& eval_s)
{
  double best_cost_value = -1; // Cost is always >= 0
  int best_cost_index = 0;
  // cout<<"cost: ";
  for (int i = 0; i < possible_states.size(); i++)
  {
    // cout<<eval_s[i]<<"> ";
    double temp_cost = evaluateCost(possible_states[i]);
    // cout<<" "<<temp_cost << " | ";
    if(best_cost_value == -1 || temp_cost < best_cost_value)
    {
      best_cost_value = temp_cost;
      best_cost_index = i;
    }
  }
  // cout<<endl;

 return best_cost_index;
}


// Evaluate cost for a particular state
double PathPlanner::evaluateCost(vector<double>& state_values_for_cost)
{
  double w1 = 0.05;
  double w2 = 0.001;
  double w3 = 1.85;

  double diff_v = state_values_for_cost[0] - max_vel_; // Ahead vehicle velocity difference to max velocity
  double cost_vel = diff_v * diff_v; // Quadratic cost for the velocity difference, prefer the lane with higher velocity

  double diff_d  = lane_ - state_values_for_cost[3]; // Difference between current lane d and possible d
  double cost_diff_d = diff_d * diff_d; // Prefers to stay at current lane

  double dist = state_values_for_cost[2]; // Distance to the vehicle ahead
  double cost_distance = 1.0 / dist; // Prefers to go to lane with bigger distance

  // cout<<"1: "<<w1 * cost_vel<< " 2: "<< w2 * cost_diff_d << " 3: "<< w3 * cost_distance<< " *";

  return w1 * cost_vel + w2 * cost_diff_d + w3 * cost_distance;
}


//
void PathPlanner::collisionAvoidance(vector<vector<double> >& sensor_fusion_)
{
  // Check sensor fusion data to avoid collisions
  for (int f = 0; f < sensor_fusion_.size(); f++)
  {
    double sf_vx = sensor_fusion_[f][3];
    double sf_vy = sensor_fusion_[f][4];
    double sf_s = sensor_fusion_[f][5];
    double sf_d = sensor_fusion_[f][6];

    double dist_sf = sf_s - s_;
    double dist_sf_d_abs = abs(tra_d_ - sf_d);

    double keep_distance = 20;

    if(dist_sf_d_abs < 1.0 && dist_sf > 0 && dist_sf < keep_distance) // If the vehicle is in the same lane and we are considering
    {
      // Transform vehicle velocities to Frenet velocities vd and vs
      int next_waypoint_index = nextWaypoint(x_, y_, yaw_);
      double sf_vd = sf_vx * map_dx_[next_waypoint_index] + sf_vy * map_dy_[next_waypoint_index];

      desired_vel_ = sqrt(sf_vx * sf_vx + sf_vy * sf_vy - sf_vd * sf_vd) - 0.5; // If closer than keep distance then it will back up slowly
    }
  }
}
