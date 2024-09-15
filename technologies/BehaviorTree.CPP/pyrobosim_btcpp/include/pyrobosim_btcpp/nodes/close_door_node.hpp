#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <pyrobosim_msgs/action/execute_task_action.hpp>
#include "execute_task_node.hpp"

namespace BT
{

class CloseDoorAction : public ExecuteTaskNode
{
public:
  CloseDoorAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : ExecuteTaskNode(name, conf, params)
  {}

  // specify the ports offered by this node
  static BT::PortsList providedPorts()
  {
    return ExecuteTaskNode::appendProvidedPorts({});
  }

  // Implement the method that sends the goal
  bool setGoal(TaskAction& action) override
  {
    // prepare the goal message
    action.type = "close";
    action.target_location = "door";
    return true;
  }
};

}  // namespace BT
