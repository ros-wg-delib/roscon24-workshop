
#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

namespace BT
{

class IsBatteryLow : public ConditionNode
{
public:
  IsBatteryLow(const std::string& name, const NodeConfig& config, rclcpp::Logger logger)
    : ConditionNode(name, config), logger_(logger)
  {}

  static PortsList providedPorts()
  {
    return { InputPort<double>("low_threshold", 20.0,
                               "Return SUCCESS if {@battery_level} below this value") };
  }

  // You must override the virtual function tick()
  NodeStatus tick() override
  {
    // Read a value from the blackboard
    double battery_level = -1;
    double low_threshold = 0;
    if(!config().blackboard->get("@battery_level", battery_level))
    {
      RCLCPP_ERROR(logger_, "Battery level not available yet");
      return NodeStatus::FAILURE;
    }
    if(!getInput("low_threshold", low_threshold))
    {
      throw RuntimeError("Missing parameter [low_threshold] in IsBatteryLow BT node");
    }
    RCLCPP_INFO(logger_, "Battery level: %.1f", battery_level);
    return (battery_level < low_threshold) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

private:
  rclcpp::Logger logger_;
};

//----------------------------------------------------------------------------------------------------

class IsBatteryFull : public ConditionNode
{
public:
  IsBatteryFull(const std::string& name, const NodeConfig& config, rclcpp::Logger logger)
    : ConditionNode(name, config), logger_(logger)
  {}

  static PortsList providedPorts()
  {
    return { InputPort<double>("full_threshold", 90.0,
                               "Return SUCCESS if {@battery_level} above this value") };
  }

  // You must override the virtual function tick()
  NodeStatus tick() override
  {
    // Read a value from the blackboard
    double battery_level = -1;
    double full_threshold = 0;
    if(!config().blackboard->get("@battery_level", battery_level))
    {
      RCLCPP_ERROR(logger_, "Battery level not available yet");
      return NodeStatus::FAILURE;
    }
    if(!getInput("full_threshold", full_threshold))
    {
      throw RuntimeError("Missing parameter [full_threshold] in IsBatteryFull BT node");
    }
    RCLCPP_INFO(logger_, "Battery level: %.1f", battery_level);
    return (battery_level >= full_threshold) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace BT
