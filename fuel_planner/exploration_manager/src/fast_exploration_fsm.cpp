
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner
{
  void FastExplorationFSM::init(ros::NodeHandle &nh)
  {
    fp_.reset(new FSMParam);
    fd_.reset(new FSMData);

    /*  Fsm param  */
    nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
    nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
    nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
    nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
    use_fuel = false;

    /* Initialize main modules */
    expl_manager_.reset(new FastExplorationManager);
    expl_manager_->initialize(nh);
    visualization_.reset(new PlanningVisualization(nh));

    planner_manager_ = expl_manager_->planner_manager_;

    state_ = EXPL_STATE::INIT;
    fd_->have_odom_ = false;
    fd_->state_str_ = {"INIT", "PAUSED", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH"};
    fd_->static_state_ = true;
    fd_->is_running_ = false;
    node_ = ros::NodeHandle(nh);

    /* Ros sub, pub and timer */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
    frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

    start_sub_ = nh.subscribe("/fuel/start", 1, &FastExplorationFSM::startCallback, this);
    pause_sub_ = nh.subscribe("/fuel/pause", 1, &FastExplorationFSM::pauseCallback, this);
    odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
    reset_service_ = nh.advertiseService("/fuel/reset", &FastExplorationFSM::resetCallback, this);
    use_fuel_service_ = nh.advertiseService("/fuel/use_fuel", &FastExplorationFSM::useFuelCallback, this);

    remapper_mode_pub_ = nh.advertise<std_msgs::Bool>("/fast_planner_server_node/set_remapper_mode", 1);
    replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
    new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
    bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

    // FastPlanner
    current_wp_ = 0;
    exec_state_ = FAST_PLANNER_EXEC_STATE::INIT;
    have_target_ = false;
    // have_odom_ -> fd_->have_odom_;
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    waypoint_sub_ =
        nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::waypointCallback, this);
  }

  void FastExplorationFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    if (use_fuel)
      return;
    trigger_ = true;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, msg->poses[0].pose.position.z;
    visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
    end_vel_.setZero();
    have_target_ = true;

    if (exec_state_ == FAST_PLANNER_EXEC_STATE::WAIT_TARGET)
      changeFSMExecState(FAST_PLANNER_EXEC_STATE::GEN_NEW_TRAJ, "TRIG");
    else if (exec_state_ == FAST_PLANNER_EXEC_STATE::EXEC_TRAJ)
      changeFSMExecState(FAST_PLANNER_EXEC_STATE::REPLAN_TRAJ, "TRIG");
  }

  void FastExplorationFSM::changeFSMExecState(FAST_PLANNER_EXEC_STATE new_state, string pos_call)
  {
    string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_"
                                                                                 "TRAJ"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void FastExplorationFSM::printFSMExecState()
  {
    string state_str[5] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_"
                                                                                 "TRAJ"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  bool FastExplorationFSM::callKinodynamicReplan()
  {
    auto t1 = ros::Time::now();
    ros::Time time_r = ros::Time::now();
    bool plan_success =
        planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

    auto t2 = ros::Time::now();
    if (plan_success)
    {
      replan_time_.push_back((t2 - t1).toSec());
      double mean1 = 0.0;
      for (auto t : replan_time_)
        mean1 += t;
      mean1 /= replan_time_.size();
      ROS_WARN("Replan number: %d, mean traj: %lf", replan_time_.size(), mean1);
    }

    if (plan_success)
    {
      planner_manager_->planYaw(start_yaw_);

      auto info = &planner_manager_->local_data_;
      info->start_time_ = time_r;

      /* publish traj */
      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

      for (int i = 0; i < pos_pts.rows(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i)
      {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

      bspline_pub_.publish(bspline);

      /* visulization */
      auto plan_data = &planner_manager_->plan_data_;

      visualization_->drawGeometricPath(plan_data->kino_path_, 0.05, Eigen::Vector4d(1, 0, 1, 1));
      visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true,
                                  0.15, Eigen::Vector4d(1, 0, 0, 1));

      return true;
    }
    else
    {
      cout << "generate new traj fail." << endl;
      return false;
    }
  }

  void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e)
  {
    if (use_fuel)
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

      switch (state_)
      {
      case INIT:
      {
        // Wait for odometry ready
        if (!fd_->have_odom_)
        {
          ROS_WARN_THROTTLE(1.0, "no odom.");
          return;
        }
        // Go to wait trigger when odom is ok
        if (fd_->is_running_)
          transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
        else
          transitState(EXPL_STATE::PAUSED, "FSM");
        break;
      }

      case PAUSED:
      {
        // Do nothing but wait for start
        ROS_WARN_THROTTLE(1.0, "wait for start.");
        break;
      }

      case FINISH:
      {
        ROS_INFO_THROTTLE(1.0, "finish exploration.");
        break;
      }

      case PLAN_TRAJ:
      {
        if (fd_->static_state_)
        {
          ROS_WARN("STATIC STATE!");
          // Plan from static state (hover)
          fd_->start_pt_ = fd_->odom_pos_;
          fd_->start_vel_ = fd_->odom_vel_;
          fd_->start_acc_.setZero();

          fd_->start_yaw_(0) = fd_->odom_yaw_;
          fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
          new_pub_.publish(std_msgs::Empty());
        }
        else
        {
          ROS_WARN("PLAN_TRAJ");
          // Replan from non-static state, starting from 'replan_time' seconds later
          LocalTrajData *info = &planner_manager_->local_data_;
          double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

          fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
          fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
          fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
          fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
          fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
          fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
          replan_pub_.publish(std_msgs::Empty());
        }

        // Inform traj_server the replanning
        int res = callExplorationPlanner();
        if (res == SUCCEED)
        {
          transitState(EXPL_STATE::PUB_TRAJ, "FSM");
        }
        else if (res == NO_FRONTIER)
        {
          transitState(EXPL_STATE::FINISH, "FSM");
          fd_->static_state_ = true;
          clearVisMarker();
        }
        else if (res == FAIL)
        {
          // Still in PLAN_TRAJ state, keep replanning
          ROS_WARN("plan fail");
          fd_->static_state_ = true;
        }
        break;
      }

      case PUB_TRAJ:
      {
        ROS_WARN("PUB_TRAJ");
        double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
        if (dt > 0)
        {
          bspline_pub_.publish(fd_->newest_traj_);
          fd_->static_state_ = false;
          transitState(EXPL_STATE::EXEC_TRAJ, "FSM");

          thread vis_thread(&FastExplorationFSM::visualize, this);
          vis_thread.detach();
        }
        break;
      }

      case EXEC_TRAJ:
      {
        ROS_WARN("EXEC_TRAJ");
        LocalTrajData *info = &planner_manager_->local_data_;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();

        // Replan if traj is almost fully executed
        double time_to_end = info->duration_ - t_cur;
        if (time_to_end < fp_->replan_thresh1_)
        {
          transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
          ROS_WARN("Replan: traj fully executed=================================");
          return;
        }
        // Replan if next frontier to be visited is covered
        if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered())
        {
          transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
          ROS_WARN("Replan: cluster covered=====================================");
          return;
        }
        // Replan after some time
        if (t_cur > fp_->replan_thresh3_ && !classic_)
        {
          transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
          ROS_WARN("Replan: periodic call=======================================");
        }
        break;
      }
      }
    }
    else
    {
      static int fsm_num = 0;
      fsm_num++;
      if (fsm_num == 100)
      {
        printFSMExecState();
        if (!trigger_)
          cout << "wait for goal." << endl;
        fsm_num = 0;
      }

      switch (exec_state_)
      {
      case INIT:
      {
        if (!fd_->have_odom_)
        {
          return;
        }
        if (!trigger_)
        {
          return;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
      }

      case WAIT_TARGET:
      {
        if (!have_target_)
          return;
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
      }

      case GEN_NEW_TRAJ:
      {
        start_pt_ = fd_->odom_pos_;
        start_vel_ = fd_->odom_vel_;
        start_acc_.setZero();

        Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
        start_yaw_(0) = atan2(rot_x(1), rot_x(0));
        start_yaw_(1) = start_yaw_(2) = 0.0;

        bool success = callKinodynamicReplan();
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          // have_target_ = false;
          // changeFSMExecState(WAIT_TARGET, "FSM");
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
      }

      case EXEC_TRAJ:
      {
        /* determine if need to replan */
        LocalTrajData *info = &planner_manager_->local_data_;
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - info->start_time_).toSec();
        t_cur = min(info->duration_, t_cur);

        Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

        /* && (end_pt_ - pos).norm() < 0.5 */
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
          return;
        }
        else if ((end_pt_ - pos).norm() < no_replan_thresh_)
        {
          // cout << "near end" << endl;
          return;
        }
        else if ((info->start_pos_ - pos).norm() < replan_thresh_)
        {
          // cout << "near start" << endl;
          return;
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        break;
      }

      case REPLAN_TRAJ:
      {
        LocalTrajData *info = &planner_manager_->local_data_;
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - info->start_time_).toSec();

        start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
        start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
        start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

        std_msgs::Empty replan_msg;
        replan_pub_.publish(replan_msg);

        bool success = callKinodynamicReplan();
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
      }
      }
    }
  }

  int FastExplorationFSM::callExplorationPlanner()
  {
    ROS_WARN("EXPLORATION PLANNER CALLED");
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

    int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                               fd_->start_yaw_);
    classic_ = false;

    if (res == SUCCEED)
    {
      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

      bspline::Bspline bspline;
      bspline.order = planner_manager_->pp_.bspline_degree_;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      for (int i = 0; i < pos_pts.rows(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(i, 0);
        pt.y = pos_pts(i, 1);
        pt.z = pos_pts(i, 2);
        bspline.pos_pts.push_back(pt);
      }
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
      Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i)
      {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
      fd_->newest_traj_ = bspline;
    }
    return res;
  }

  void FastExplorationFSM::visualize()
  {
    auto info = &planner_manager_->local_data_;
    auto plan_data = &planner_manager_->plan_data_;
    auto ed_ptr = expl_manager_->ed_;

    // Draw frontier
    static int last_ftr_num = 0;
    for (int i = 0; i < ed_ptr->frontiers_.size(); ++i)
    {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                                "frontier", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i)
    {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();
    visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                                Vector4d(1, 1, 0, 1));
  }

  void FastExplorationFSM::clearVisMarker()
  {
  }

  void FastExplorationFSM::frontierCallback(const ros::TimerEvent &e)
  {
    static int delay = 0;
    if (++delay < 5)
      return;

    if (state_ == PAUSED || state_ == FINISH)
    {
      auto ft = expl_manager_->frontier_finder_;
      auto ed = expl_manager_->ed_;
      ft->searchFrontiers();
      ft->computeFrontiersToVisit();
      ft->updateFrontierCostMatrix();

      ft->getFrontiers(ed->frontiers_);
      ft->getFrontierBoxes(ed->frontier_boxes_);

      // Draw frontier and bounding box
      for (int i = 0; i < ed->frontiers_.size(); ++i)
      {
        visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                  visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                  "frontier", i, 4);
      }
      for (int i = ed->frontiers_.size(); i < 50; ++i)
      {
        visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      }
    }
  }

  void FastExplorationFSM::startCallback(const std_msgs::Empty &msg)
  {
    fd_->is_running_ = true;
    cout << "Started!" << endl;
    transitState(EXPL_STATE::PLAN_TRAJ, "startCallback");
  }

  void FastExplorationFSM::pauseCallback(const std_msgs::Empty &msg)
  {
    fd_->is_running_ = false;
    fd_->static_state_ = true;
    cout << "Started!" << endl;
    transitState(EXPL_STATE::PAUSED, "pauseCallback");
  }

  void FastExplorationFSM::safetyCallback(const ros::TimerEvent &e)
  {
    if (use_fuel)
    {
      if (state_ == EXPL_STATE::EXEC_TRAJ)
      {
        // Check safety and trigger replan if necessary
        double dist;
        bool safe = planner_manager_->checkTrajCollision(dist);
        if (!safe)
        {
          ROS_WARN("Replan: collision detected==================================");
          transitState(EXPL_STATE::PLAN_TRAJ, "safetyCallback");
        }
      }
    }
    else
    {
      LocalTrajData *info = &planner_manager_->local_data_;

      if (have_target_)
      {
        auto edt_env = planner_manager_->edt_environment_;

        double dist = planner_manager_->pp_.dynamic_ ? edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) : edt_env->evaluateCoarseEDT(end_pt_, -1.0);

        if (dist <= 0.3)
        {
          /* try to find a max distance goal around */
          bool new_goal = false;
          const double dr = 0.5, dtheta = 30, dz = 0.3;
          double new_x, new_y, new_z, max_dist = -1.0;
          Eigen::Vector3d goal;

          for (double r = dr; r <= 5 * dr + 1e-3; r += dr)
          {
            for (double theta = -90; theta <= 270; theta += dtheta)
            {
              for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz)
              {
                new_x = end_pt_(0) + r * cos(theta / 57.3);
                new_y = end_pt_(1) + r * sin(theta / 57.3);
                new_z = end_pt_(2) + nz;

                Eigen::Vector3d new_pt(new_x, new_y, new_z);
                dist = planner_manager_->pp_.dynamic_ ? edt_env->evaluateCoarseEDT(new_pt,
                                                                                   /* time to program start+ */ info->duration_)
                                                      : edt_env->evaluateCoarseEDT(new_pt, -1.0);

                if (dist > max_dist)
                {
                  /* reset end_pt_ */
                  goal(0) = new_x;
                  goal(1) = new_y;
                  goal(2) = new_z;
                  max_dist = dist;
                }
              }
            }
          }

          if (max_dist > 0.3)
          {
            cout << "change goal, replan." << endl;
            end_pt_ = goal;
            have_target_ = true;
            end_vel_.setZero();

            if (exec_state_ == FAST_PLANNER_EXEC_STATE::EXEC_TRAJ)
            {
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }

            visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
          }
          else
          {
            // have_target_ = false;
            // cout << "Goal near collision, stop." << endl;
            // changeFSMExecState(WAIT_TARGET, "SAFETY");
            cout << "goal near collision, keep retry" << endl;
            changeFSMExecState(REPLAN_TRAJ, "FSM");

            std_msgs::Empty emt;
            replan_pub_.publish(emt);
          }
        }
      }

      /* ---------- check trajectory ---------- */
      if (exec_state_ == FAST_PLANNER_EXEC_STATE::EXEC_TRAJ)
      {
        double dist;
        bool safe = planner_manager_->checkTrajCollision(dist);

        if (!safe)
        {
          // cout << "current traj in collision." << endl;
          ROS_WARN("current traj in collision.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
      }
    }
  }

  bool FastExplorationFSM::useFuelCallback(std_srvs::SetBoolRequest &request, std_srvs::SetBoolResponse &response)
  {
    if (use_fuel == request.data)
    {
      return true;
    }
    auto x = std_msgs::Bool();
    x.data = request.data;
    remapper_mode_pub_.publish(x);
    use_fuel = request.data;
    if (use_fuel)
    {
      fd_->is_running_ = true;
      transitState(EXPL_STATE::PLAN_TRAJ, "startCallback");
    }
    else
    {
      fd_->is_running_ = false;
      fd_->static_state_ = true;
      transitState(EXPL_STATE::PAUSED, "pauseCallback");
    }
    return true;
  }

  void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    fd_->odom_pos_(0) = msg->pose.pose.position.x;
    fd_->odom_pos_(1) = msg->pose.pose.position.y;
    fd_->odom_pos_(2) = msg->pose.pose.position.z;

    fd_->odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->odom_vel_(2) = msg->twist.twist.linear.z;

    fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

    fd_->have_odom_ = true;
  }

  bool FastExplorationFSM::resetCallback(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response)
  {
    fp_.reset(new FSMParam);
    fd_.reset(new FSMData);

    /* Initialize main modules */
    expl_manager_.reset(new FastExplorationManager);
    expl_manager_->initialize(node_);
    visualization_.reset(new PlanningVisualization(node_));

    fd_->have_odom_ = false;
    fd_->state_str_ = {"INIT", "PAUSED", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH"};
    fd_->static_state_ = true;
    fd_->is_running_ = false;
    return true;
  }

  void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call)
  {
    int pre_s = int(state_);
    state_ = new_state;
    cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
         << endl;
  }
}
