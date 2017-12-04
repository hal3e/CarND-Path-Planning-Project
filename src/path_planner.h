#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <string>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class PathPlanner {
  double x_;                  // Vehicle current states
  double y_;
  double s_;
  double d_;
  double yaw_;
  double vel_;

  int lane_;
  int num_lanes_;
  double desired_vel_;
  double max_vel_;
  double tra_d_;               // Trajectory d_, i.e. vehicle will move to this new d_
  bool state_transition_over;  // If the vehicle is changing state a new state can not be assigned

  vector<double> map_x_;       // Map information
  vector<double> map_y_;
  vector<double> map_s_;
  vector<double> map_dx_;
  vector<double> map_dy_;

  vector<string> all_states_; // All states and transitions that the vehicle can perform

  public:
    PathPlanner();
    ~PathPlanner(){}

    // Update states to the current vehicle state
    void updateStates(double x, double y, double s, double d, double yaw, double vel);

    // Update map information
    void updateMap(const vector<double>& m_x, const vector<double>& m_y, const vector<double>& m_s, const vector<double>& m_dx, const vector<double>& m_dy);

    // Generate a new trajectory and send it back to simulator
    void newTrajectory(vector<double>& next_x, vector<double>& next_y, vector<vector<double> >& sensor_fusion);

    // Get closest waypoint on the map
    int closestWaypointIndex(double x, double y);

    // Get next waypoint from current position
    int nextWaypoint(double x, double y, double theta);

    // Calculate distance between two points
    double distance(double x1, double y1, double x2, double y2);

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d);

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    vector<double> getFrenet(double x, double y, double theta);

    // Get available states from the current state
    vector<string> getAvailableStates();

    // Decide what is the best next action
    void decideNewState(vector<vector<double> >& sensor_fusion_);

    // Evaluate the cost and return the lowest one
    int returnBestCostIndex(vector<vector<double> >& possible_states, vector<string>& eval_s);

    // Evaluate cost for a particular state
    double evaluateCost(vector<double>& state_values_for_cost);

    void collisionAvoidance(vector<vector<double> >& sensor_fusion_);
};
#endif // PATH_PLANNER_H
