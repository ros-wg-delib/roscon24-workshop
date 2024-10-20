
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace BT
{

class GetCurrentLocation : public SyncActionNode
{
public:
  GetCurrentLocation(const std::string& name, const NodeConfig& config, rclcpp::Logger logger)
    : SyncActionNode(name, config), logger_(logger)
  {}

  static PortsList providedPorts()
  {
    return { OutputPort<std::string>("location", "The robot's current location.") };
  }

  // You must override the virtual function tick()
  NodeStatus tick() override
  {
    // Read a value from the blackboard
    std::string current_location = "";
    if(!config().blackboard->get("@last_visited_location", current_location))
    {
      RCLCPP_ERROR(logger_, "Robot location not available yet");
      return NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger_, "Robot location level: %s", current_location.c_str());
    setOutput("location", current_location);
    return NodeStatus::SUCCESS;
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace BT
