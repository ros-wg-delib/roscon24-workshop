#include <fstream>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pyrobosim_msgs/msg/robot_state.hpp>

// BTCPP includes
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

// BTCPP nodes in this package
#include "pyrobosim_btcpp/nodes/battery_nodes.hpp"
#include "pyrobosim_btcpp/nodes/get_current_location_node.hpp"
#include "pyrobosim_btcpp/nodes/open_node.hpp"
#include "pyrobosim_btcpp/nodes/close_node.hpp"
#include "pyrobosim_btcpp/nodes/detect_object_node.hpp"
#include "pyrobosim_btcpp/nodes/navigate_node.hpp"
#include "pyrobosim_btcpp/nodes/pick_object_node.hpp"
#include "pyrobosim_btcpp/nodes/place_object_node.hpp"

// TO_WORKSHOP_USER: add here the include to your custom actions, if you have any

std::filesystem::path GetFilePath(const std::string& filename)
{
  // check first the given path
  if(std::filesystem::exists(filename))
  {
    return filename;
  }
  // try appending the package directory
  const std::string package_dir = ament_index_cpp::get_package_share_directory("pyrobosim_btcpp");
  const auto package_path = std::filesystem::path(package_dir) / filename;
  if(std::filesystem::exists(package_path))
  {
    return package_path;
  }
  throw std::runtime_error("File not found: " + filename);
}

int main(int argc, char** argv)
{
  // Create a ROS Node
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("btcpp_executor");

  nh->declare_parameter("tree", rclcpp::PARAMETER_STRING);
  nh->declare_parameter("save_model_only", false);

  std::string tree_filename;
  nh->get_parameter("tree", tree_filename);
  bool save_model = false;
  nh->get_parameter("save_model_only", save_model);

  if(tree_filename.empty() && !save_model)
  {
    RCLCPP_FATAL(nh->get_logger(), "Missing parameter 'tree' with the path to the Behavior Tree "
                                   "XML file");
    return 1;
  }

  //----------------------------------
  // register all the actions in the factory
  BT::BehaviorTreeFactory factory;

  // all the actions are done using the same Action Server.
  // Therefore  single RosNodeParams will do.
  BT::RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "execute_action";
  params.wait_for_server_timeout = std::chrono::seconds(5);

  factory.registerNodeType<BT::IsBatteryLow>("IsBatteryLow", nh->get_logger());
  factory.registerNodeType<BT::IsBatteryFull>("IsBatteryFull", nh->get_logger());
  factory.registerNodeType<BT::GetCurrentLocation>("GetCurrentLocation", nh->get_logger());
  factory.registerNodeType<BT::CloseAction>("Close", params);
  factory.registerNodeType<BT::DetectObject>("DetectObject", params);
  factory.registerNodeType<BT::NavigateAction>("Navigate", params);
  factory.registerNodeType<BT::OpenAction>("Open", params);
  factory.registerNodeType<BT::PickObject>("PickObject", params);
  factory.registerNodeType<BT::PlaceObject>("PlaceObject", params);

  // TO_WORKSHOP_USER: register here more Nodes, if you decided to implement your own

  // Optionally, we can save the model of the Nodes registered in the factory
  if(save_model)
  {
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream of("pyrobosim_btcpp_model.xml");
    of << xml_models;
    RCLCPP_INFO(nh->get_logger(), "XML model of the tree saved in pyrobosim_btcpp_model.xml");
    return 0;
  }
  const std::filesystem::path filepath = GetFilePath(tree_filename);

  //----------------------------------
  // load a tree and execute
  factory.registerBehaviorTreeFromFile(filepath.string());

  // the global blackboard patterns is explained here:
  // https://www.behaviortree.dev/docs/tutorial-advanced/tutorial_16_global_blackboard
  auto global_blackboard = BT::Blackboard::create();
  BT::Tree tree = factory.createTree("MainTree", BT::Blackboard::create(global_blackboard));

  // This will add console messages for each action and condition executed
  BT::StdCoutLogger console_logger(tree);
  console_logger.enableTransitionToIdle(false);

  BT::Groot2Publisher groot2_publisher(tree, 5555);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nh);

  bool state_received = false;

  // create a subscriber to /robot/robot_state to update the blackboard
  // Note that the prefix/namespace used here is "/robot", for the purpose of the
  // workshop, but this identifier may change in pyrobosim.
  auto robot_state_subscriber = nh->create_subscription<pyrobosim_msgs::msg::RobotState>(
      "/robot/robot_state", 10,
      [global_blackboard, &state_received](const pyrobosim_msgs::msg::RobotState::SharedPtr msg) {
        global_blackboard->set("battery_level", msg->battery_level);
        global_blackboard->set("holding_object", msg->holding_object);
        global_blackboard->set("last_visited_location", msg->last_visited_location);
        global_blackboard->set("executing_action", msg->executing_action);
        state_received = true;
      });

  // wait for the first message to be received by robot_state_subscriber
  while(!state_received)
  {
    executor.spin_once(std::chrono::milliseconds(10));
  }

  // This is the "main loop". Execution is completed once the tick() method returns SUCCESS of FAILURE
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while(rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    // tick once the tree
    status = tree.tickOnce();
    // Spin to update robot_state_subscriber
    executor.spin_once(std::chrono::milliseconds(1));
  }

  std::cout << "Execution completed. Result: " << BT::toStr(status) << std::endl;

  return 0;
}
