#ifndef BT_NAV_CLIENT
#define BT_NAV_CLIENT

#include <ros/ros.h>
#include "bt_action_node.h"
#include <geometry_msgs/PoseStamped.h>
#include <functional>
#include <queue>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <behaviortree_cpp_v3/bt_factory.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

extern std::queue<geometry_msgs::PoseStamped> goal_queue;
extern MoveBaseClient* action_client;

class GoalSubscriber
{
public:
  GoalSubscriber(ros::NodeHandle& nh, std::string topic = "move_base_simple/goal") : nh_(nh), topic_(topic_)
  {
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        topic, 10, std::bind(&GoalSubscriber::goalCallback, this, std::placeholders::_1));
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& ip_goal)
  {
    goal_queue.push(*ip_goal);
  }

private:
  std::string topic_;
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
};

class CheckGoalsQueueNode : public BT::SyncActionNode
{
public:
  CheckGoalsQueueNode(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  BT::NodeStatus tick() override
  {
    // Check if goals queue is not empty
    bool isQueueNotEmpty = !goal_queue.empty();

    return isQueueNotEmpty ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }
};

// class SendGoalActionNode : public BT::RosActionNode<move_base_msgs::MoveBaseAction>
// {
// public:
//   SendGoalActionNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& config)
//     : BT::RosActionNode<move_base_msgs::MoveBaseAction>(nh, name, config)
//     , rviz_goal_subscriber_(nh, "/move_base_simple/goal")
//   {
//     action_client = new MoveBaseClient("move_base", true);

//     // Wait for the action server to come up
//     if (!action_client->isServerConnected())
//       action_client->waitForServer(ros::Duration(10.0));
//   }

//   virtual bool sendGoal(GoalType& goal) override
//   {
//     if (!goal_queue.empty())
//     {
//       goal.target_pose = goal_queue.front();
//       goal_queue.pop();
//       ROS_INFO("Sending goal to MoveBase");
//       action_client->sendGoal(goal);
//       return true;
//     }
//     else
//     {
//       ROS_WARN("No goal received from RViz");
//       return false;
//     }
//   }

//   virtual BT::NodeStatus onResult(const ResultType& res) override
//   {
//     // Process the result (print if the goal is reached or not)
//     if (action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     {
//       ROS_INFO("MoveBase goal achieved");
//       return BT::NodeStatus::SUCCESS;
//     }
//     else
//     {
//       ROS_INFO("MoveBase goal not achieved");
//       return BT::NodeStatus::FAILURE;
//     }
//   }

//   static BT::PortsList providedPorts()
//   {
//     return {};
//   }

//   ~SendGoalActionNode()
//   {
//     delete action_client;
//   }

// private:

//   GoalSubscriber rviz_goal_subscriber_;
//   MoveBaseClient* action_client;
// };

class SendGoalActionNode : public BT::SyncActionNode
{
public:
  SendGoalActionNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    action_client = new MoveBaseClient("move_base", true);

    // Wait for the action server to come up
    if (!action_client->isServerConnected())
      action_client->waitForServer(ros::Duration(10.0));
  }

  BT::NodeStatus tick() override
  {
    // Check if there is a goal in the queue
    if (!goal_queue.empty())
    {
      // Extract the goal from the queue
      geometry_msgs::PoseStamped goal = goal_queue.front();
      goal_queue.pop();

      // Construct the MoveBase goal
      move_base_msgs::MoveBaseGoal move_base_goal;
      move_base_goal.target_pose = goal;

      // Send the goal to MoveBase
      action_client->sendGoal(move_base_goal);

      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_WARN("No goal received from RViz");
      return BT::NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  ~SendGoalActionNode()
  {
    delete action_client;
  }

private:
  
  MoveBaseClient* action_client;
};
#endif
