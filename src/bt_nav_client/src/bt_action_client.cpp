#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string_view>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include "bt_action_client.hpp"

std::queue<geometry_msgs::PoseStamped> goal_queue;

void registerMoveBaseNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& node_handle)
{
  // Create instances of your custom nodes
  // CheckGoalsQueueNode checkGoalsQueueNode("CheckGoals", BT::NodeConfiguration());
  //÷SendGoalActionNode sendGoalActionNode(node_handle, "SendGoal", BT::NodeConfiguration());
  // Register the nodes with the factory
  factory.registerNodeType<CheckGoalsQueueNode>("CheckGoals");
  factory.registerNodeType<SendGoalActionNode>("SendGoal");
  // BT::RegisterRosAction<SendGoalActionNode>(factory, "SendGoal", node_handle);
}

// std::string xml_text =
//     R"(
//     <root main_tree_to_execute="MainTree">
//       <BehaviorTree ID="MainTree">
//         <Sequence name="main">
//           <Action ID="CheckGoalsQueue"/>
//           <Action ID="SendGoal"/>
//         </Sequence>
//       </BehaviorTree>
//     </root>)";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_bt_client");
  ros::NodeHandle nh;

  // Print the XML content
  // std::cout << "XML content:\n" << xml_text << std::endl;

  // Create a Behavior Tree factory
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SimpleSubscriber>("GoalSubscriber");
  factory.registerNodeType<SendGoalActionNode>("SendGoal");
  factory.registerNodeType<CheckGoalsQueueNode>("CheckGoals");

  // Register MoveBase nodes
  // registerMoveBaseNodes(factory, nh);
  // auto registeredNodes = factory.manifests();
  // for (const auto& manifest : registeredNodes)
  // {
  //   std::cout << "Node Name: " << manifest.first << std::endl;
  //   std::cout << "Category: " << manifest.second.registration_ID << std::endl;
  //   std::cout << "Description: " << manifest.second.type << std::endl;
  //   std::cout << "------------------------" << std::endl;
  // }

  std::string pkg_path = ros::package::getPath("bt_nav_client");
  std::string file_path = pkg_path + "/tree.xml";

  BT::Tree tree;

  //std::cout << "Marco." << std::endl;
  tree = factory.createTreeFromFile(file_path);
  try
  {
    std::string xml_content = R"(
        <root main_tree_to_execute="MainTree">

          <BehaviorTree ID="MainTree">
            <Sequence name="main">
              <GoalSubscriber/>
              <CheckGoals/>
              <SendGoal/>
            </Sequence>
          </BehaviorTree>

        </root>)";

    std::cout << "Creating Tree..." << std::endl;
    tree = factory.createTreeFromText(xml_content);
    std::cout << "Tree created successfully." << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error creating the tree: " << e.what() << std::endl;
    return 1;  // Exit the program with an error code
  }

  // Create a logger for console output

  BT::PublisherZMQ publisher_zmq(tree);
  // Run the Behavior Tree using a loop (you might replace this with a ROS rate loop)
  while (ros::ok())
  {
    // Execute one tick of the Behavior Tree
    BT::NodeStatus status = tree.rootNode()->executeTick();
    ros::spinOnce();
    // Sleep for a short duration (you might replace this with a ROS sleep or other logic)
    ros::Duration(0.5).sleep();
  }

  return 0;
}
