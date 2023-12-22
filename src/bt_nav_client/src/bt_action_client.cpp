#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include "bt_action_client.hpp"



std::queue<geometry_msgs::PoseStamped> goal_queue;

static std::string xml_text =
    R"(<root main_tree_to_execute="Main_Tree">
          <BehaviorTree ID="Main_Tree">
              <Sequence name="main">
                  <CheckGoalsQueue name="CheckGoalsQueue" />
                  <SendGoal name="SendGoal" />
              </Sequence>
          </BehaviorTree>
      </root>)";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_bt_client");
    ros::NodeHandle nh;

    // Print the XML content
    std::cout << "XML content:\n" << xml_text << std::endl;

    // Create a Behavior Tree factory
    BT::BehaviorTreeFactory factory;
    factory.clearRegisteredBehaviorTrees();

    // Register MoveBase nodes
    registerMoveBaseNodes(factory, nh);
    auto registeredNodes = factory.manifests();
    for (const auto& manifest : registeredNodes)
    {
        std::cout << "Node Name: " << manifest.first << std::endl;
        std::cout << "Category: " << manifest.second.registration_ID << std::endl;
        std::cout << "Description: " << manifest.second.type << std::endl;
        std::cout << "------------------------" << std::endl;
    }

    // Create the Behavior Tree from a string
    // Create a Behavior Tree from the XML string

    BT::Tree tree;

    try
    {
       // std::cout << "Loading XML file: " << xml_filename << std::endl;
        tree = factory.createTreeFromText(xml_text);
        std::cout << "Tree created successfully." << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error creating the tree: " << e.what() << std::endl;
        return 1; // Exit the program with an error code
    }

    // Create a logger for console output

    BT::PublisherZMQ publisher_zmq(tree);
    BT::NodeStatus status = BT::NodeStatus::IDLE;

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
