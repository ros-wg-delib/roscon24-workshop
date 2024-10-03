#include <fstream>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// BTCPP includes
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/xml_parsing.h>

// BTCPP nodes in this package
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
  nh->declare_parameter("save-model", false);

  const std::string tree_filename = nh->get_parameter("tree").as_string();
  const bool save_model = nh->get_parameter("save-model").as_bool();

  if(tree_filename.empty())
  {
    RCLCPP_FATAL(nh->get_logger(), "Missing parameter 'tree' with the path to the Behavior Tree "
                                   "XML file");
    return 1;
  }
  const std::filesystem::path filepath = GetFilePath(tree_filename);

  //----------------------------------
  // register all the actions in the factory
  BT::BehaviorTreeFactory factory;

  // all the actions are done using the same Action Server.
  // Therefore  single RosNodeParams will do.
  BT::RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "execute_action";

  factory.registerNodeType<BT::CloseAction>("Close", params);
  factory.registerNodeType<BT::DetectObject>("DetectObject", params);
  factory.registerNodeType<BT::NavigateAction>("Navigate", params);
  factory.registerNodeType<BT::OpenAction>("Open", params);
  factory.registerNodeType<BT::PickObject>("PickObject", params);
  factory.registerNodeType<BT::PlaceObject>("PlaceObject", params);

  // TO_WORKSHOP_USER: add here more rgistration, if you decided to implement your own nodes

  // optionally we can display and save the model of the tree
  if(save_model)
  {
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::cout << "--------- Node Models ---------:\n" << xml_models << std::endl;
    std::ofstream of("tree_nodes_model.xml");
    of << xml_models;
    std::cout << "\nXML model of the tree saved in tree_nodes_model.xml\n" << std::endl;
  }

  //----------------------------------
  // load a tree and execute
  BT::Tree tree = factory.createTreeFromFile(filepath.string());

  // This will add console messages for each action and condition executed
  BT::StdCoutLogger console_logger(tree);
  console_logger.enableTransitionToIdle(false);

  // This is the "main loop":xecution is completed once the tick() method returns SUCCESS of FAILURE
  BT::NodeStatus res = tree.tickWhileRunning();

  std::cout << "Execution completed. Result: " << BT::toStr(res) << std::endl;

  return 0;
}
