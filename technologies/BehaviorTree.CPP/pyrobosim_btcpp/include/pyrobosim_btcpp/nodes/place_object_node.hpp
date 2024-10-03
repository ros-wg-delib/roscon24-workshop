#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <pyrobosim_msgs/action/execute_task_action.hpp>
#include "execute_task_node.hpp"

namespace BT
{

class PlaceObject : public ExecuteTaskNode
{
public:
  PlaceObject(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : ExecuteTaskNode(name, conf, params)
  {}

  // specify the ports offered by this node
  static BT::PortsList providedPorts()
  {
    return ExecuteTaskNode::appendProvidedPorts({ BT::InputPort<std::string>("object") });
  }

  // Implement the method that sends the goal
  bool setGoal(TaskAction& action) override
  {
    std::string object;
    if(!getInput("object", object) || object.empty())
    {
      throw BT::RuntimeError("missing required input [object]");
    }
    // prepare the goal message
    action.type = "place";
    action.object = object;
    return true;
  }
};

}  // namespace BT
