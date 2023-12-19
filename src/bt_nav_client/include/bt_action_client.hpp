#ifndef BT_NAV_CLIENT
#define BT_NAV_CLIENT

#include <ros/ros.h>
#include "bt_action_node.h"
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>

std::queue<geometry_msgs::PoseStamped> goal_queue;

class GoalSubscriber
{
public:
  GoalSubscriber(ros::NodeHandle& nh, std::string topic = "move_base_simple/goal") : nh_(nh)
  {
    goal_sub_=nh_.subscribe<geometry_msgs::PoseStamped>(topic, 1, &RVizGoalSubscriber::goalCallback, this)
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& ip_goal)
  {
    goal_queue.push(*ip_goal);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
};

#endif  // BT_NAV_CLIENT