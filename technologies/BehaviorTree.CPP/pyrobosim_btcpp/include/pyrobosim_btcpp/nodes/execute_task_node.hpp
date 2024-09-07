#include <behaviortree_ros2/bt_action_node.hpp>
#include <pyrobosim_msgs/action/execute_task_action.hpp>

namespace BT
{

inline const char* resultToStr(pyrobosim_msgs::msg::ExecutionResult result)
{
  switch(result.status)
  {
    case pyrobosim_msgs::msg::ExecutionResult::UNKNOWN:
      return "UNKNOWN";
    case pyrobosim_msgs::msg::ExecutionResult::SUCCESS:
      return "SUCCESS";
    case pyrobosim_msgs::msg::ExecutionResult::PRECONDITION_FAILURE:
      return "PRECONDITION_FAILURE";
    case pyrobosim_msgs::msg::ExecutionResult::PLANNING_FAILURE:
      return "PLANNING_FAILURE";
    case pyrobosim_msgs::msg::ExecutionResult::EXECUTION_FAILURE:
      return "EXECUTION_FAILURE";
    case pyrobosim_msgs::msg::ExecutionResult::POSTCONDITION_FAILURE:
      return "POSTCONDITION_FAILURE";
    case pyrobosim_msgs::msg::ExecutionResult::INVALID_ACTION:
      return "INVALID_ACTION";
    case pyrobosim_msgs::msg::ExecutionResult::CANCELED:
      return "CANCELED";
  }
  return "UNKNOWN";
}

/**
 * @brief Base class for all the nodes that execute a task using the ExecuteTaskAction
 */
class ExecuteTaskNode : public RosActionNode<pyrobosim_msgs::action::ExecuteTaskAction>
{
public:
  ExecuteTaskNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<pyrobosim_msgs::action::ExecuteTaskAction>(name, conf, params)
  {}

  using TaskAction = pyrobosim_msgs::msg::TaskAction;
  using ExecutionResult = pyrobosim_msgs::msg::ExecutionResult;

  // method that sends the goal. Must be implemented by the derived class
  virtual bool setGoal(TaskAction& action) = 0;

  // virtual method processing the result. Can be overridden by the derived class
  virtual NodeStatus onResultReceived(const ExecutionResult& execution_result);

  // default implementation of the onFailure callback. Can be overridden by the derived class
  NodeStatus onFailure(ActionNodeErrorCode error) override;

private:
  bool setGoal(Goal& goal) override final
  {
    goal.action.robot = "robot";  // default name
    return setGoal(goal.action);
  }

  NodeStatus onResultReceived(const WrappedResult& wr) override final
  {
    return onResultReceived(wr.result->execution_result);
  }
};

//------------------------------------------------------------
//------------------------------------------------------------
//------------------------------------------------------------

NodeStatus ExecuteTaskNode::onResultReceived(const ExecutionResult& execution_result)
{
  if(execution_result.status != ExecutionResult::SUCCESS)
  {
    RCLCPP_ERROR(logger(), "[%s] failed with error: %s. Message: %s", name().c_str(),
                 resultToStr(execution_result), execution_result.message.c_str());
    return NodeStatus::FAILURE;
  }
  return NodeStatus::SUCCESS;
}

NodeStatus ExecuteTaskNode::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "[%s] failed with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

}  // namespace BT
