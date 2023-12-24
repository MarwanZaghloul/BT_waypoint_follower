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
#include <behaviortree_cpp_v3/behavior_tree.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

extern std::queue<geometry_msgs::PoseStamped> goal_queue;
extern MoveBaseClient* action_client;

class GoalSubscriber
{
public:
  GoalSubscriber(std::string topic = "move_base_simple/goal") : nh_("~"), topic_(topic_)
  {
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "move_base_simple/goal", 10, std::bind(&GoalSubscriber::goalCallback, this, std::placeholders::_1));
    std::cout << "subscriber created " << std::endl;
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

class SimpleSubscriber : public BT::AsyncActionNode
{
public:
  SimpleSubscriber(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config)
  {
    ros::NodeHandle nh;
    sub_ = nh.subscribe("goals", 1, &SimpleSubscriber::messageCallback, this);
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    ros::spinOnce();
    return BT::NodeStatus::SUCCESS;
  }

private:
  void messageCallback(const geometry_msgs::PoseStamped::ConstPtr& ip_goal)
  {
    goal_queue.push(*ip_goal);
  }

  ros::Subscriber sub_;
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
    bool isQueueNotEmpty = goal_queue.size() < 5;

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
//********************************
// class SendGoalActionNode : public BT::SyncActionNode
// {
// public:
//   SendGoalActionNode(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
//   {
//     action_client = new MoveBaseClient("move_base", true);

//     // Wait for the action server to come up
//     if (!action_client->isServerConnected())
//     {
//       action_client->waitForServer(ros::Duration(10.0));
//       std::cout << "server connected" << std::endl;
//     }
//   }

//   BT::NodeStatus tick() override
//   {
//     // Check if there is a goal in the queue
//     if (!goal_queue.empty())
//     {
//       // Extract the goal from the queue
//       geometry_msgs::PoseStamped goal = goal_queue.front();
//       goal_queue.pop();

//       // Construct the MoveBase goal

//       move_base_msgs::MoveBaseGoal move_base_goal;
//       move_base_goal.target_pose = goal;

//       // Send the goal to MoveBase
//       action_client->sendGoal(move_base_goal);
//       action_client->waitForResult();

//       return BT::NodeStatus::SUCCESS;
//     }
//     else
//     {
//       ROS_WARN("No goal received from RViz");
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
//   MoveBaseClient* action_client;
// };
//*******************************
// Existing includes...
class SendGoalActionNode : public BT::SyncActionNode
{
public:
  SendGoalActionNode(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
    action_client = new MoveBaseClient("move_base", true);

    // Wait for the action server to come up
    if (!action_client->isServerConnected())
    {
      action_client->waitForServer(ros::Duration(10.0));
      std::cout << "server connected" << std::endl;
    }
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    // Move goals from global goal_queue to local_gq until local_gq has 5 goals
    while (local_gq.size() < 5 && !goal_queue.empty())
    {
      local_gq.push(goal_queue.front());
      goal_queue.pop();
    }

    // Check if local_gq has accumulated 5 goals
    if (local_gq.size() == 5)
    {
      // Send all goals from local_gq
      while (!local_gq.empty())
      {
        // Extract the goal from the local queue
        geometry_msgs::PoseStamped goal = local_gq.front();
        local_gq.pop();

        // Construct the MoveBase goal
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose = goal;

        // Send the goal to MoveBase
        action_client->sendGoal(move_base_goal);
        action_client->waitForResult();

        // Handle goal completion (you can add custom logic here)
        const actionlib::SimpleClientGoalState& state = action_client->getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("MoveBase goal achieved");
        }
        else
        {
          ROS_INFO("MoveBase goal not achieved");
        }
      }

      // Reset the local_gq for the next iteration
      local_gq = std::queue<geometry_msgs::PoseStamped>();
    }

    return BT::NodeStatus::SUCCESS;
  }

  ~SendGoalActionNode()
  {
    delete action_client;
  }

private:
  std::queue<geometry_msgs::PoseStamped> local_gq;
  MoveBaseClient* action_client;
};

#endif

