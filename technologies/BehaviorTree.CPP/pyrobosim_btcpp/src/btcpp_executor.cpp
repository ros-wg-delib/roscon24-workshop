#include <CLI11/CLI11.hpp>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// BTCPP includes
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/xml_parsing.h>

// BTCPP nodes in this package
#include "pyrobosim_btcpp/nodes/navigate_node.hpp"

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
  // Parse command line arguments
  CLI::App app("BehaviorTree.CPP executor for pyrobosim");

  std::string tree_filename;
  app.add_option("-t,--tree", tree_filename, "Path to the XML containing the tree")->required();

  bool save_model = false;
  app.add_flag("--save-model", save_model, "Save the XML model of the tree");

  CLI11_PARSE(app, argc, argv);

  const std::filesystem::path filepath = GetFilePath(tree_filename);

  //----------------------------------
  // Create a ROS Node
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("btcpp_executor");

  //----------------------------------
  // register all the actions in the factory
  BT::BehaviorTreeFactory factory;

  // all the actions are done using the same Action Server.
  // Therefore  single RosNodeParams will do.
  BT::RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "execute_action";

  factory.registerNodeType<BT::NavigateAction>("Navigate", params);

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
