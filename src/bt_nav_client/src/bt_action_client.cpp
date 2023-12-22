#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include "bt_action_client.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_bt_client");
  ros::NodeHandle nh;

  // Create a Behavior Tree factory
  BT::BehaviorTreeFactory factory;

  // Register MoveBase nodes
  registerMoveBaseNodes(factory, nh);

  // Create the Behavior Tree from a string
  const std::string bt_xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <CheckMoveBaseServer />
        <Fallback>
          <Sequence>
            <CheckGoalsQueueNode />
            <SendGoal />
          </Sequence>
          <Idle />
        </Fallback>
      </BehaviorTree>
    </root>
  )";

  // Create a Behavior Tree from the XML string
  auto tree = factory.createTreeFromText(bt_xml);

  // Create a logger for console output
  BT::StdCoutLogger logger_cout(tree);
  BT::PublisherZMQ publisher_zmq(tree);

  // Run the Behavior Tree using a loop (you might replace this with a ROS rate loop)
  while (ros::ok())
  {
    // Execute one tick of the Behavior Tree
    BT::NodeStatus status = tree.rootNode()->executeTick();

    ros::spinOnce();

    // Sleep for a short duration (you might replace this with a ROS sleep or other logic)
    ros::Duration(0.1).sleep();
  }

  return 0;
}