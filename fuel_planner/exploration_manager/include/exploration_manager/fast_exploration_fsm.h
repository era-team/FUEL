#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
  class FastPlannerManager;
  class FastExplorationManager;
  class PlanningVisualization;
  struct FSMParam;
  struct FSMData;

  enum EXPL_STATE
  {
    INIT,
    PAUSED,
    PLAN_TRAJ,
    PUB_TRAJ,
    EXEC_TRAJ,
    FINISH
  };

  class FastExplorationFSM
  {
  private:
    /* planning utils */
    shared_ptr<FastPlannerManager> planner_manager_;
    shared_ptr<FastExplorationManager> expl_manager_;
    shared_ptr<PlanningVisualization> visualization_;

    shared_ptr<FSMParam> fp_;
    shared_ptr<FSMData> fd_;
    EXPL_STATE state_;

    bool classic_;
    bool use_fuel;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
    ros::Subscriber start_sub_, pause_sub_, odom_sub_, waypoint_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, remapper_mode_pub_;
    ros::ServiceServer reset_service_, use_fuel_service_;

    /* helper functions */
    int callExplorationPlanner();
    void transitState(EXPL_STATE new_state, string pos_call);

    /* ROS functions */
    void FSMCallback(const ros::TimerEvent &e);
    void safetyCallback(const ros::TimerEvent &e);
    void frontierCallback(const ros::TimerEvent &e);
    void startCallback(const std_msgs::Empty &msg);
    void pauseCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    bool resetCallback(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);
    bool useFuelCallback(std_srvs::SetBoolRequest &request, std_srvs::SetBoolResponse &response);
    void visualize();
    void clearVisMarker();

    /* FastPlanner */
    enum FAST_PLANNER_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      REPLAN_NEW
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };
    double no_replan_thresh_, replan_thresh_;
    bool trigger_, have_target_;
    FAST_PLANNER_EXEC_STATE exec_state_;
    Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                             // target state
    int current_wp_;
    bool callKinodynamicReplan();       // front-end and back-end method
    bool callTopologicalTraj(int step); // topo path guided gradient-based
                                        // optimization; 1: new, 2: replan
    void changeFSMExecState(FAST_PLANNER_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    void waypointCallback(const nav_msgs::PathConstPtr &msg);

  public:
    FastExplorationFSM(/* args */)
    {
    }
    ~FastExplorationFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    vector<double> replan_time_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace fast_planner

#endif