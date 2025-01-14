/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2021-2022, Nokia
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         Mike Phillips (put the planner in its own thread)
 *********************************************************************/

#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define AC_TIMEOUT .5
#define HB_TIMEOUT 1.0
#define EPSILON 0.01

namespace move_base
{

  MoveBase::MoveBase(tf2_ros::Buffer &tf) : tf_(tf),
                                            as_(NULL),
                                            as_legacy_(NULL),
                                            ac_(NULL),
                                            current_vx_(0.0),
                                            current_vy_(0.0),
                                            current_omegaz_(0.0),
                                            brake_(false),
                                            handbrake_requested_(false),
                                            planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
                                            bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
                                            blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
                                            recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
                                            planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
                                            planner_waypoint_indices_(NULL), latest_waypoint_indices_(NULL), controller_waypoint_indices_(NULL),
                                            closest_plan_waypoint_index_(-1), pursued_plan_waypoint_index_(-1),
                                            runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false)
  {

    ac_ = new MoveBaseSWPActionClient("move_base_swp", true);
    as_ = new MoveBaseSWPActionServer(ros::NodeHandle(), "move_base_swp",
                                      boost::bind(&MoveBase::executeCb, this, _1), false);
    as_legacy_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base",
                                          boost::bind(&MoveBase::executeLegacyCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    // get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1); // disabled by default
    private_nh.param("max_replan_on_incomplete", max_replan_on_incomplete_, 8);

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    // set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    planner_waypoint_indices_ = new std::vector<int>();
    latest_waypoint_indices_ = new std::vector<int>();
    controller_waypoint_indices_ = new std::vector<int>();

    // set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));
    brake_thread_ = new boost::thread(boost::bind(&MoveBase::brakeThread, this));

    // for commanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
    current_waypoints_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("current_waypoints", 0);
    snapped_pose_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("snapped_pose", 0);
    pursued_plan_pub_ = private_nh.advertise<nav_msgs::Path>("pursued_plan", 0);

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_ = action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);
    handbrake_sub_ = action_nh.subscribe<move_base_swp::Handbrake>("handbrake", 1, boost::bind(&MoveBase::handbrakeCB, this, _1));

    // we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    // they won't get any useful information back about its status, but this is useful for tools
    // like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    // we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    private_nh.param("brake_slope", brake_slope_, 0.5);
    private_nh.param("brake_sample_rate", brake_sample_rate_, 20.0);
    if (brake_sample_rate_ < 1.0 || brake_sample_rate_ > 50.0)
    {
      ROS_WARN("Brake sample rate out of range, using default");
      brake_sample_rate_ = 20.0;
    }

    private_nh.param("plan_min_step_len", plan_min_step_len_, 0.025);
    private_nh.param("plan_buffer_size", plan_buffer_size_, 150);
    private_nh.param("plan_reload_threshold", plan_reload_threshold_, 100);

    // create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    // initialize the global planner
    try
    {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    // create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    // create a local planner
    try
    {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    // advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    // advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    // if we shutdown our costmaps when we're deactivated... we'll do that now
    if (shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    // load any user specified recovery behaviors, and if that fails load the defaults
    if (!loadRecoveryBehaviors(private_nh))
    {
      loadDefaultRecoveryBehaviors();
    }

    // initially, we'll need to make a plan
    state_ = PLANNING;

    // we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    // we're all set up now so we can start the action server
    as_->start();
    as_legacy_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void MoveBase::clearPlans()
  {
    planner_plan_->clear();
    planner_waypoint_indices_->clear();
    latest_plan_->clear();
    latest_waypoint_indices_->clear();
    controller_plan_->clear();
    controller_waypoint_indices_->clear();
    closest_plan_waypoint_index_ = -1;
    pursued_plan_waypoint_index_ = -1;
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    // The first time we're called, we just want to make sure we have the
    // original configuration
    if (!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if (config.restore_defaults)
    {
      config = default_config_;
      // if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if (planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if (controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    max_replan_on_incomplete_ = config.max_replan_on_incomplete;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    brake_slope_ = config.brake_slope;
    brake_sample_rate_ = config.brake_sample_rate;

    plan_min_step_len_ = config.plan_min_step_len;
    plan_buffer_size_ = config.plan_buffer_size;
    plan_reload_threshold_ = config.plan_reload_threshold;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if (config.base_global_planner != last_config_.base_global_planner)
    {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      // initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try
      {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        clearPlans();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      }
      catch (const pluginlib::PluginlibException &ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                  config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if (config.base_local_planner != last_config_.base_local_planner)
    {
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      // create a local planner
      try
      {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        clearPlans();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      }
      catch (const pluginlib::PluginlibException &ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                  config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }

  void MoveBase::handbrakeCB(const move_base_swp::Handbrake::ConstPtr &brake)
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    handbrake_requested_ = brake->data;
    std::cout << brake->data << std::endl;
    last_handbrake_msg_ = ros::Time::now();
  }

  // evaluate handbrake, return true if requested and not timed out
  bool MoveBase::handbrakeEngaged(void)
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    if (!handbrake_requested_)
      return false;
    if (ros::Time::now() > last_handbrake_msg_ + ros::Duration(HB_TIMEOUT))
    {
      handbrake_requested_ = false;
      return false;
    }
    return true;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal)
  {
    ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y)
  {
    geometry_msgs::PoseStamped global_pose;

    // clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    // clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    // clear the costmaps
    {
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
      controller_costmap_ros_->resetLayers();
    }
    {
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
      planner_costmap_ros_->resetLayers();
    }
    return true;
  }

  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
  {
    if (as_->isActive())
    {
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    // make sure we have a costmap for our planner
    if (planner_costmap_ros_ == NULL)
    {
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    // if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if (req.start.header.frame_id.empty())
    {
      geometry_msgs::PoseStamped global_pose;
      if (!getRobotPose(global_pose, planner_costmap_ros_))
      {
        ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
        return false;
      }
      start = global_pose;
    }
    else
    {
      start = req.start;
    }

    if (make_plan_clear_costmap_)
    {
      // update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    // first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if (!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
    {
      ROS_DEBUG_NAMED("move_base", "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                      req.goal.pose.position.x, req.goal.pose.position.y);

      // search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution * 3.0;
      if (req.tolerance > 0.0 && req.tolerance < search_increment)
        search_increment = req.tolerance;
      for (float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment)
      {
        for (float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment)
        {
          for (float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment)
          {

            // don't search again inside the current outer layer
            if (x_offset < max_offset - 1e-9 && y_offset < max_offset - 1e-9)
              continue;

            // search to both sides of the desired goal
            for (float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0)
            {

              // if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if (y_offset < 1e-9 && y_mult < -1.0 + 1e-9)
                continue;

              for (float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0)
              {
                if (x_offset < 1e-9 && x_mult < -1.0 + 1e-9)
                  continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if (planner_->makePlan(start, p, global_plan))
                {
                  if (!global_plan.empty())
                  {

                    if (make_plan_add_unreachable_goal_)
                    {
                      // adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else
                {
                  ROS_DEBUG_NAMED("move_base", "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    // copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for (unsigned int i = 0; i < global_plan.size(); ++i)
    {
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase()
  {
    recovery_behaviors_.clear();

    delete dsrv_;

    if (as_ != NULL)
      delete as_;

    if (as_legacy_ != NULL)
      delete as_legacy_;

    if (planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if (controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    brake_thread_->interrupt();
    brake_thread_->join();

    delete planner_thread_;
    delete brake_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;
    delete planner_waypoint_indices_;
    delete latest_waypoint_indices_;
    delete controller_waypoint_indices_;

    planner_.reset();
    tc_.reset();
  }

  /**
   * subsample the passed in plan and replace. Enforce a minimum cartesian distance
   * between waypoints. The first and last points in the original plan are always
   * preserved. min_step is in meters, and 0 disables the subsampling. */
  static void mbs_resample_plan(std::vector<geometry_msgs::PoseStamped> &plan, double min_step)
  {
    if (min_step > 1e-9 && plan.size() > 2)
    {
      // subsample the plan
      std::vector<geometry_msgs::PoseStamped> new_plan;
      const double min_dist_sq = (min_step * min_step) - 1e-9;
      size_t plan_len_minus_1 = plan.size() - 1;
      new_plan.push_back(plan[0]);
      geometry_msgs::PoseStamped a = plan[0];
      for (size_t i = 1; i < plan_len_minus_1; i++)
      {
        const geometry_msgs::PoseStamped &b = plan[i];
        double dx = b.pose.position.x - a.pose.position.x;
        double dy = b.pose.position.y - a.pose.position.y;
        double dist_sq = (dx * dx) + (dy * dy);
        if (dist_sq > min_dist_sq)
        {
          new_plan.push_back(b);
          a = b;
        }
      }
      new_plan.push_back(plan[plan_len_minus_1]);
      ROS_DEBUG("mbs: subsample global plan from %d to %d steps", (int)plan.size(),
                (int)new_plan.size());
      plan.swap(new_plan);
    }
    else
    {
      ROS_DEBUG("mbs: global plan has %d steps", (int)plan.size());
    }
  }

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    // make sure to set the plan to be empty initially
    plan.clear();

    // since this gets called on handle activate
    if (planner_costmap_ros_ == NULL)
    {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    // get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    if (!getRobotPose(global_pose, planner_costmap_ros_))
    {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped &start = global_pose;

    // if the planner fails or returns a zero length plan, planning failed
    if (!planner_->makePlan(start, goal, plan) || plan.empty())
    {
      ROS_DEBUG_NAMED("move_base", "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    // reduce waypoint density
    mbs_resample_plan(plan, plan_min_step_len_);

    return true;
  }

  void MoveBase::applyBrakes()
  {
    boost::unique_lock<boost::recursive_mutex> lock(brake_mutex_);
    brake_ = true;
    brake_cond_.notify_one();
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion &q)
  {
    // first we need to check if the quaternion has nan's or infs
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    {
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    // next, we need to check if the length of the quaternion is close to zero
    if (tf_q.length2() < 1e-6)
    {
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    // next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if (fabs(dot - 1) > 1e-3)
    {
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  void MoveBase::publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
  {
    nav_msgs::Path p;
    p.poses = plan;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = planner_costmap_ros_->getGlobalFrameID();
    pursued_plan_pub_.publish(p);
  }

  void MoveBase::publishWaypoints(const std::vector<geometry_msgs::PoseStamped> &waypoints)
  {
    geometry_msgs::PoseArray w;
    for (auto p = waypoints.begin(); p < waypoints.end(); p++)
      w.poses.push_back(p->pose);
    w.header.stamp = ros::Time::now();
    w.header.frame_id = planner_costmap_ros_->getGlobalFrameID();
    current_waypoints_pub_.publish(w);
  }

  bool MoveBase::loadWaypoints(const move_base_swp::MoveBaseSWPGoalConstPtr &swp_goal, std::vector<geometry_msgs::PoseStamped> &waypoints)
  {
    for (auto g = swp_goal->waypoint_poses.begin(); g < swp_goal->waypoint_poses.end(); g++)
    {
      if (!isQuaternionValid(g->pose.orientation))
      {
        as_->setAborted(move_base_swp::MoveBaseSWPResult(),
                        "SWP goal contains an invalid quaternion");
        return false;
      }
      logPose("loading waypoint", *g);
      waypoints.push_back(goalToGlobalFrame(*g));
    }
    if (waypoints.size() == 0)
    {
      ROS_WARN("Received empty waypoint list");
      as_->setAborted(move_base_swp::MoveBaseSWPResult(),
                      "SWP goal contains no waypoints");
      return false;
    }
    current_goal_pub_.publish(waypoints.back());
    publishWaypoints(waypoints);
    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg)
  {
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    // just get the latest available transform... for accuracy they should send
    // goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try
    {
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
               goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent &event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  void MoveBase::planThread()
  {
    ROS_DEBUG_NAMED("move_base_plan_thread", "Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while (n.ok())
    {
      // check if we should run the planner (the mutex is locked)
      while (wait_for_wake || !runPlanner_)
      {
        // if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      // time to plan! get a copy of the goal and unlock the mutex
      std::vector<geometry_msgs::PoseStamped> temp_waypoints = planner_waypoints_;
      lock.unlock();

      // run planner for each waypoint
      planner_plan_->clear();
      planner_waypoint_indices_->clear();
      double planner_frequency;
      {
        boost::recursive_mutex::scoped_lock cl(configuration_mutex_);
        planner_frequency = planner_frequency_;
      }
      bool gotPlan = false;
      if (n.ok())
      {
        for (auto w = temp_waypoints.begin(); w < temp_waypoints.end(); w++)
        {
          std::vector<geometry_msgs::PoseStamped> temp_plan;
          temp_plan.clear();
          if (w == temp_waypoints.begin())
          {
            logPose("planning to first waypoint", *w);
            gotPlan = makePlan(*w, temp_plan);
          }
          else
          {
            logPose("planning to waypoint", *w);
            gotPlan = planner_->makePlan(*(w - 1), *w, temp_plan);
          }
          if (gotPlan)
          {
            planner_plan_->insert(planner_plan_->end(), temp_plan.begin(), temp_plan.end());
            int wp_index = planner_plan_->size() - 1;
            ROS_INFO("planning succeeded, wp_index=%d", wp_index);
            planner_waypoint_indices_->push_back(wp_index);
          }
          else
          {
            ROS_WARN("plan failed");
            planner_plan_->clear();
            planner_waypoint_indices_->clear();
            break;
          }
        }
      }
      if (gotPlan)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread", "Got Plan with %zu points!", planner_plan_->size());
        // pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;
        std::vector<int> *temp_waypoint_indices = planner_waypoint_indices_;

        lock.lock();
        planner_plan_ = latest_plan_;
        planner_waypoint_indices_ = latest_waypoint_indices_;
        latest_plan_ = temp_plan;
        latest_waypoint_indices_ = temp_waypoint_indices;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

        // make sure we only start the controller if we still haven't reached the goal
        if (runPlanner_)
          state_ = CONTROLLING;
        if (planner_frequency <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      // if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if (state_ == PLANNING)
      {
        uint32_t max_planning_retries;
        double planner_patience;
        {
          boost::recursive_mutex::scoped_lock cl(configuration_mutex_);
          max_planning_retries = (uint32_t)max_planning_retries_;
          planner_patience = planner_patience_;
        }
        ROS_DEBUG_NAMED("move_base_plan_thread", "No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience);

        // check if we've tried to make a plan for over our time limit or our maximum number of retries
        // issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        // is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if (runPlanner_ &&
            (ros::Time::now() > attempt_end || planning_retries_ > max_planning_retries))
        {
          // we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false; // proper solution for issue #523
          applyBrakes();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      // take the mutex for the next iteration
      lock.lock();

      // setup sleep interface if needed
      if (planner_frequency > 0)
      {
        ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_frequency)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0))
        {
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  bool MoveBase::rampDownVelocity(double &vx, double &vy, double &omegaz, double delta)
  {
    double v = hypot(vx, vy);
    // TODO: dehardcode brake sample rate
    double new_v = v - delta;
    if (new_v < 0.0)
    {
      vx = 0.0;
      vy = 0.0;
      omegaz = 0.0;
      return true;
    }
    vx *= new_v / v;
    vy *= new_v / v;
    if (fabs(omegaz) <= EPSILON)
    {
      omegaz = 0.0;
    }
    else
    {
      double turn_radius = v / omegaz;
      omegaz = (omegaz < 0.0) ? -new_v / v : new_v / v;
    }
    return false;
  }

  void MoveBase::brakeThread()
  {
    ros::NodeHandle n;
    double brake_delta;
    double brake_sample_rate;
    geometry_msgs::Twist cmd_vel;
    boost::unique_lock<boost::recursive_mutex> lock(brake_mutex_);

    while (n.ok())
    {
      if (!brake_)
      {
        ROS_DEBUG_NAMED("move_base", "brake thread going to sleep");
        brake_cond_.wait(lock);
      }
      else
      {
        lock.unlock();
        // check if parameters have changed with lock released to
        // avoid the deadlock
        {
          boost::recursive_mutex::scoped_lock cl(configuration_mutex_);
          brake_sample_rate = brake_sample_rate_;
          brake_delta = brake_slope_ / brake_sample_rate;
        }
        ros::Rate r(brake_sample_rate);
        lock.lock();
        if (rampDownVelocity(current_vx_, current_vy_, current_omegaz_, brake_delta))
          brake_ = false;
        cmd_vel.linear.x = current_vx_;
        cmd_vel.linear.y = current_vy_;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = current_omegaz_;
        ROS_DEBUG_NAMED("move_base", "brakes active vx=%.2f, vy=%.2f, omega=%.2f",
                        current_vx_, current_vy_, current_omegaz_);
        lock.unlock();
        vel_pub_.publish(cmd_vel);
        r.sleep();
        lock.lock();
      }
    }
  }

  void MoveBase::logPose(const char *msg, const geometry_msgs::PoseStamped &p)
  {

    ROS_INFO("%s: p=(%.2f, %.2f, %2f), q=(%2f, %2f, %2f, %2f)", msg,
             p.pose.position.x, p.pose.position.y, p.pose.position.z,
             p.pose.orientation.x, p.pose.orientation.y,
             p.pose.orientation.z, p.pose.orientation.w);
  }

  void MoveBase::executeLegacyCb(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal)
  {
    logPose("Got goal on legacy interface", move_base_goal->target_pose);
    if (!ac_->waitForServer(ros::Duration(AC_TIMEOUT)))
    {
      as_legacy_->setAborted(move_base_msgs::MoveBaseResult(),
                             "Primary interface not responding");
      ROS_ERROR("Primary interface not responding, aborting");
      return;
    }

    move_base_swp::MoveBaseSWPGoal swp_goal;
    swp_goal.waypoint_poses.push_back(move_base_goal->target_pose);
    ac_->sendGoal(swp_goal);
    ros::NodeHandle n;
    while (n.ok())
    {
      if (as_legacy_->isPreemptRequested())
      {
        if (as_legacy_->isNewGoalAvailable())
        {
          move_base_msgs::MoveBaseGoalConstPtr new_goal = as_legacy_->acceptNewGoal();
          logPose("Preempted by new goal on legacy interface", new_goal->target_pose);
          move_base_swp::MoveBaseSWPGoal swp_goal;
          swp_goal.waypoint_poses.push_back(new_goal->target_pose);
          ac_->sendGoal(swp_goal);
        }
        else
        {
          ROS_INFO("Preempted by cancel request on legacy interface");
          ac_->cancelGoal();
        }
      }
      double controller_frequency;
      {
        boost::recursive_mutex::scoped_lock cl(configuration_mutex_);
        controller_frequency = controller_frequency_;
      }
      bool done = ac_->waitForResult(ros::Duration(1.0 / controller_frequency));
      actionlib::SimpleClientGoalState state = ac_->getState();
      if (done && state.isDone())
      {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Primary interface succeeded");
          as_legacy_->setSucceeded(move_base_msgs::MoveBaseResult(),
                                   state.getText());
        }
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
          ROS_INFO("Primary interface aborted");
          as_legacy_->setAborted(move_base_msgs::MoveBaseResult(),
                                 state.getText());
        }
        else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
        {
          ROS_INFO("Primary interface preempted");
          as_legacy_->setPreempted(move_base_msgs::MoveBaseResult(),
                                   state.getText());
        }
        else
        {
          ROS_WARN("Primary interface recall or reject treated as abort");
          // RECALLED and REJECTED will match this but the only thing
          // we can do is abort, because legacy interface has accepted
          // the goal before it new that primary interface would reject
          // or recall it
          as_legacy_->setAborted(move_base_msgs::MoveBaseResult(),
                                 state.toString() + ':' + state.getText());
        }
        return;
      }
      geometry_msgs::PoseStamped global_pose;
      getRobotPose(global_pose, planner_costmap_ros_);
      const geometry_msgs::PoseStamped &current_position = global_pose;
      move_base_msgs::MoveBaseFeedback feedback;
      feedback.base_position = current_position;
      as_legacy_->publishFeedback(feedback);
    }
  }

  void MoveBase::setVelocity(const geometry_msgs::Twist &cmd_vel)
  {
    boost::unique_lock<boost::recursive_mutex> lock(brake_mutex_);
    vel_pub_.publish(cmd_vel);
    current_vx_ = cmd_vel.linear.x;
    current_vy_ = cmd_vel.linear.y;
    current_omegaz_ = cmd_vel.angular.y;
    brake_ = false;
  }

  void MoveBase::startPlanner(const std::vector<geometry_msgs::PoseStamped> &waypoints)
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_waypoints_ = waypoints;
    replan_on_incomplete_counter_ = 0;
    runPlanner_ = true;
    planner_cond_.notify_one();
  }

  void MoveBase::startPlanner()
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = true;
    planner_cond_.notify_one();
  }

  void MoveBase::stopPlanner()
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
  }

  void MoveBase::executeCb(const move_base_swp::MoveBaseSWPGoalConstPtr &swp_goal)
  {
    ROS_INFO("Got goal on primary interface");
    std::vector<geometry_msgs::PoseStamped> waypoints;
    if (!loadWaypoints(swp_goal, waypoints))
      return;
    applyBrakes();
    startPlanner(waypoints);

    std::vector<geometry_msgs::PoseStamped> global_plan;

    double controller_frequency;
    bool shutdown_costmaps;
    {
      boost::recursive_mutex::scoped_lock cl(configuration_mutex_);
      controller_frequency = controller_frequency_;
      shutdown_costmaps = shutdown_costmaps_;
    }
    ros::Rate r(controller_frequency);
    if (shutdown_costmaps)
    {
      ROS_DEBUG_NAMED("move_base", "Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    // we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while (n.ok())
    {
      if (c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency);
        r = ros::Rate(controller_frequency);
        c_freq_change_ = false;
      }

      if (as_->isPreemptRequested())
      {
        if (as_->isNewGoalAvailable())
        {
          // if we're active and a new goal is available,
          // we'll accept it, but we won't shut anything down
          ROS_INFO("Preempted by the new goal on primary interface");
          move_base_swp::MoveBaseSWPGoalConstPtr new_swp_goal = as_->acceptNewGoal();
          std::vector<geometry_msgs::PoseStamped> waypoints;
          if (!loadWaypoints(new_swp_goal, waypoints))
            return;

          // we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          // we have a new goal so make sure the planner is awake
          startPlanner(waypoints);

          // make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else
        {
          // if we've been preempted explicitly we need to shut things down
          resetState();

          // notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");
          as_->setPreempted();

          // we'll actually return from execute after preempting
          return;
        }
      }

      // we also want to check if we've changed global frames because we need to transform our goal pose
      bool global_frame_changed = false;
      for (auto g = waypoints.begin(); g < waypoints.end(); g++)
      {
        if (g->header.frame_id != planner_costmap_ros_->getGlobalFrameID())
        {
          logPose("global frame changed for waypoint", *g);
          *g = goalToGlobalFrame(*g);
          global_frame_changed = true;
        }
      }
      if (global_frame_changed)
      {
        // we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;
        startPlanner(waypoints);
        current_goal_pub_.publish(waypoints.back());
        publishWaypoints(waypoints);

        // make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      // for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      // the real work on pursuing a goal is done here
      bool done = executeCycle();

      // if we're done, then we'll return from execute
      if (done)
        return;

      // check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base", "Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      // make sure to sleep for the remainder of our cycle time
      if (r.cycleTime() > ros::Duration(1 / controller_frequency) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency, r.cycleTime().toSec());
    }

    // wake up the planner thread so that it can exit cleanly
    startPlanner();

    // if the node is killed then we'll abort and return
    as_->setAborted(move_base_swp::MoveBaseSWPResult(),
                    "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  geometry_msgs::PoseStamped MoveBase::updateClosestWaypointIndex(int &cwpi, const std::vector<geometry_msgs::PoseStamped> &plan)
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    geometry_msgs::PoseStamped current_pose;
    getRobotPose(current_pose, planner_costmap_ros_);
    double lowest_d = distance(current_pose, plan[cwpi]);
    double search_radius = lowest_d;
    double covered_distance = 0.0;
    int lowest_i = cwpi;
    for (int i = cwpi + 1; i < plan.size(); i++)
    {
      double d = distance(current_pose, plan[i]);
      if (d < lowest_d)
      {
        lowest_d = d;
        lowest_i = i;
      }
      covered_distance += distance(plan[i - 1], plan[i]);
      // early-termination heuristics: points that are further
      // away than the circumference of a circle determined by the
      // initial distance are unlikely to be of interest
      if (covered_distance > search_radius * 2 * M_PI)
        break;
    }
    geometry_msgs::PoseStamped snapped_pose = plan[lowest_i];
    // overwrite orientation because some planners don't set it
    snapped_pose.pose.orientation = current_pose.pose.orientation;
    // side-effect: update the index that caller passed
    cwpi = lowest_i;
    return snapped_pose;
  }

  bool MoveBase::updateNearTermPlan(int cwpi, const std::vector<geometry_msgs::PoseStamped> &full_plan, int &pwpi, std::vector<geometry_msgs::PoseStamped> &near_term_plan)
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    // check if under the reload threshold or if nothing left to load
    if (pwpi - cwpi > plan_reload_threshold_ || pwpi == full_plan.size())
      return false;
    int new_pwpi = cwpi + plan_buffer_size_;
    if (new_pwpi > full_plan.size())
      new_pwpi = full_plan.size();
    near_term_plan =
        std::vector<geometry_msgs::PoseStamped>(full_plan.begin() + cwpi, full_plan.begin() + new_pwpi);
    pwpi = new_pwpi;
    return true;
  }

  void MoveBase::pruneWaypoints(int cwpi, std::vector<geometry_msgs::PoseStamped> &waypoints, std::vector<int> &waypoint_indices)
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    if (waypoints.size() != waypoint_indices.size())
      ROS_FATAL("BUG! inconsistent waypoint list %lu!=%lu",
                waypoints.size(), waypoint_indices.size());
    // never prune the last waypoint (it can only drop after we reach the goal)
    while (waypoint_indices[0] <= cwpi && waypoint_indices.size() > 1)
    {
      ROS_INFO("pruned waypoint at %d", waypoint_indices[0]);
      waypoint_indices.erase(waypoint_indices.begin());
      waypoints.erase(waypoints.begin());
    }
  }

  void MoveBase::pruneFirstWaypoint(std::vector<geometry_msgs::PoseStamped> &waypoints, std::vector<int> &waypoint_indices)
  {
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    if (waypoints.size() != waypoint_indices.size())
      ROS_FATAL("BUG! inconsistent waypoint list %lu!=%lu",
                waypoints.size(), waypoint_indices.size());
    // never prune the last waypoint (it can only drop after we reach the goal)
    if (waypoint_indices.size() > 1)
    {
      ROS_INFO("pruned waypoint at %d", waypoint_indices[0]);
      waypoint_indices.erase(waypoint_indices.begin());
      waypoints.erase(waypoints.begin());
    }
    else
    {
      ROS_ERROR("BUG?: prineFirstWaypoint called with single-element waypoint list");
    }
  }

  bool MoveBase::executeCycle()
  {
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    // we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    // update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped &current_position = global_pose;

    // push the feedback out
    move_base_swp::MoveBaseSWPFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    // check to see if we've moved far enough to reset our oscillation timeout
    if (distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      // if our last recovery was caused by oscillation, we want to reset the recovery index
      if (recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    // check that the observation buffers for the costmap are current, we don't want to drive blind
    if (!controller_costmap_ros_->isCurrent())
    {
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety", ros::this_node::getName().c_str());
      applyBrakes();
      return false;
    }

    // if we have a new plan then grab it and give it to the controller
    if (new_global_plan_)
    {
      // make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

      // do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped> *temp_plan = controller_plan_;
      std::vector<int> *temp_waypoint_indices = controller_waypoint_indices_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      controller_waypoint_indices_ = latest_waypoint_indices_;
      closest_plan_waypoint_index_ = 0;
      pursued_plan_waypoint_index_ = -1;
      latest_plan_ = temp_plan;
      latest_waypoint_indices_ = temp_waypoint_indices;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base", "pointers swapped!");
      bool plan_set;
      if (plan_buffer_size_ > 0)
      {
        updateNearTermPlan(closest_plan_waypoint_index_, *controller_plan_, pursued_plan_waypoint_index_, near_term_plan_segment_);
        plan_set = tc_->setPlan(near_term_plan_segment_);
      }
      else
        plan_set = tc_->setPlan(*controller_plan_);
      if (!plan_set)
      {
        // ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();
        as_->setAborted(move_base_swp::MoveBaseSWPResult(),
                        "Failed to pass global plan to the controller.");
        return true;
      }
      publishPlan(near_term_plan_segment_);
      // make sure to reset recovery_index_ since we were able to find a valid plan
      if (recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    // the move_base state machine, handles the control logic for navigation
    switch (state_)
    {
    // if we are in a planning state, then we'll attempt to make a plan
    case PLANNING:
      startPlanner();
      ROS_DEBUG_NAMED("move_base", "Waiting for plan, in the planning state.");
      break;

    // if we're controlling, we'll attempt to find valid velocity commands
    case CONTROLLING:
      ROS_DEBUG_NAMED("move_base", "In controlling state.");

      // check to see if we've reached our goal
      if (tc_->isGoalReached())
      {
        ROS_DEBUG_NAMED("move_base", "Goal (probably) reached!");
        if (pursued_plan_waypoint_index_ < 0 || pursued_plan_waypoint_index_ == controller_plan_->size())
        {
          resetState();
          as_->setSucceeded(move_base_swp::MoveBaseSWPResult(), "Goal reached.");
          replan_on_incomplete_counter_ = 0;
          return true;
        }
        else
        {
          replan_on_incomplete_counter_++;
          ROS_INFO("Plan not fully executed, re-planning retry=%u", replan_on_incomplete_counter_);
          if (replan_on_incomplete_counter_ > max_replan_on_incomplete_)
          {
            ROS_WARN("Detected infeasable waypoint, pruning");
            pruneFirstWaypoint(planner_waypoints_, *controller_waypoint_indices_);
            replan_on_incomplete_counter_++;
          }
          planning_retries_ = 0;
          last_valid_plan_ = ros::Time::now();
          state_ = PLANNING;
          applyBrakes();
          return false;
        }
      }

      // check for an oscillation condition
      if (oscillation_timeout_ > 0.0 &&
          last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
      {
        applyBrakes();
        state_ = CLEARING;
        recovery_trigger_ = OSCILLATION_R;
      }

      if (handbrakeEngaged())
      {
        applyBrakes();
        state_ = HANDBRAKE;
      }
      else
      {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        bool plan_ok = true;
        if (plan_buffer_size_ > 0)
        {
          if (updateNearTermPlan(closest_plan_waypoint_index_, *controller_plan_, pursued_plan_waypoint_index_, near_term_plan_segment_))
          {
            // near-term plan needed update, so pass it to the local planner
            plan_ok = tc_->setPlan(near_term_plan_segment_);
          }
        }
        if (plan_ok && tc_->computeVelocityCommands(cmd_vel))
        {
          publishPlan(near_term_plan_segment_);
          ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
          last_valid_control_ = ros::Time::now();
          geometry_msgs::PoseStamped snapped_pose = updateClosestWaypointIndex(closest_plan_waypoint_index_, *controller_plan_);
          snapped_pose_pub_.publish(snapped_pose);
          pruneWaypoints(closest_plan_waypoint_index_, planner_waypoints_, *controller_waypoint_indices_);
          publishWaypoints(planner_waypoints_);
          // make sure that we send the velocity command to the base
          setVelocity(cmd_vel);
          if (recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else
        {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end;
          {
            boost::recursive_mutex::scoped_lock cl(configuration_mutex_);
            attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
          }
          // check if we've tried to find a valid control for longer than our time limit
          if (ros::Time::now() > attempt_end)
          {
            // we'll move into our obstacle clearing mode
            applyBrakes();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else
          {
            // otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            applyBrakes();
            startPlanner();
          }
        }
      }

      break;

    // we'll try to clear out space with any user-provided recovery behaviors
    case CLEARING:
      ROS_DEBUG_NAMED("move_base", "In clearing/recovery state");
      // we'll invoke whatever recovery behavior we're currently on if they're enabled
      if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
      {
        ROS_DEBUG_NAMED("move_base_recovery", "Executing behavior %u of %zu", recovery_index_ + 1, recovery_behaviors_.size());

        move_base_msgs::RecoveryStatus msg;
        msg.pose_stamped = current_position;
        msg.current_recovery_number = recovery_index_;
        msg.total_number_of_recoveries = recovery_behaviors_.size();
        msg.recovery_behavior_name = recovery_behavior_names_[recovery_index_];

        recovery_status_pub_.publish(msg);

        recovery_behaviors_[recovery_index_]->runBehavior();

        // we at least want to give the robot some time to stop oscillating after executing the behavior
        last_oscillation_reset_ = ros::Time::now();

        // we'll check if the recovery behavior actually worked
        ROS_DEBUG_NAMED("move_base_recovery", "Going back to planning state");
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        state_ = PLANNING;

        // update the index of the next recovery behavior that we'll try
        recovery_index_++;
      }
      else
      {
        ROS_DEBUG_NAMED("move_base_recovery", "All recovery behaviors have failed, locking the planner and disabling it.");
        stopPlanner();

        ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");

        if (recovery_trigger_ == CONTROLLING_R)
        {
          ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
          as_->setAborted(move_base_swp::MoveBaseSWPResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
        }
        else if (recovery_trigger_ == PLANNING_R)
        {
          ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
          as_->setAborted(move_base_swp::MoveBaseSWPResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
        }
        else if (recovery_trigger_ == OSCILLATION_R)
        {
          ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
          as_->setAborted(move_base_swp::MoveBaseSWPResult(), "Robot is oscillating. Even after executing recovery behaviors.");
        }
        resetState();
        return true;
      }
      break;

    case HANDBRAKE:
      if (!handbrakeEngaged())
        state_ = CONTROLLING;
      break;

    default:
      ROS_ERROR("This case should never be reached, something is wrong, aborting");
      resetState();
      as_->setAborted(move_base_swp::MoveBaseSWPResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
      return true;
    }

    // we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
  {
    XmlRpc::XmlRpcValue behavior_list;
    if (node.getParam("recovery_behaviors", behavior_list))
    {
      if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int i = 0; i < behavior_list.size(); ++i)
        {
          if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            if (behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type"))
            {
              // check for recovery behaviors with the same name
              for (int j = i + 1; j < behavior_list.size(); j++)
              {
                if (behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if (behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type"))
                  {
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if (name_i == name_j)
                    {
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else
            {
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else
          {
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                      behavior_list[i].getType());
            return false;
          }
        }

        // if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for (int i = 0; i < behavior_list.size(); ++i)
        {
          try
          {
            // check if a non fully qualified name has potentially been passed in
            if (!recovery_loader_.isClassAvailable(behavior_list[i]["type"]))
            {
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for (unsigned int i = 0; i < classes.size(); ++i)
              {
                if (behavior_list[i]["type"] == recovery_loader_.getName(classes[i]))
                {
                  // if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                           std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            // shouldn't be possible, but it won't hurt to check
            if (behavior.get() == NULL)
            {
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            // initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch (pluginlib::PluginlibException &ex)
          {
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else
      {
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                  behavior_list.getType());
        return false;
      }
    }
    else
    {
      // if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    // if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  // we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors()
  {
    recovery_behaviors_.clear();
    try
    {
      // we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      // first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      // next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if (clearing_rotation_allowed_)
      {
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      // next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(ags_clear);

      // we'll rotate in-place one more time
      if (clearing_rotation_allowed_)
      {
        recovery_behaviors_.push_back(rotate);
        recovery_behavior_names_.push_back("rotate_recovery");
      }
    }
    catch (pluginlib::PluginlibException &ex)
    {
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState()
  {
    stopPlanner();
    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    applyBrakes();

    // if we shutdown our costmaps when we're deactivated... we'll do that now
    bool shutdown_costmaps;
    {
      boost::recursive_mutex::scoped_lock cl(configuration_mutex_);
      shutdown_costmaps = shutdown_costmaps_;
    }
    if (shutdown_costmaps)
    {
      ROS_DEBUG_NAMED("move_base", "Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped &global_pose, costmap_2d::Costmap2DROS *costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time();     // latest available
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. "
                             "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f",
                        costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
