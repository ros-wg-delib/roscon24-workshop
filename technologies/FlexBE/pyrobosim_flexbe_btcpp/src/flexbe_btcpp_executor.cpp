// Copyright 2024 David Conner
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
// TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Based on pyrobosim_btcpp Copyright 2024 Davide Faconti
// and sample_bt_executor.cpp Copyright 2024 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <fstream>

#include <std_msgs/msg/float32.hpp>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// BTCPP includes
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_ros2/tree_execution_server.hpp>

// BTCPP nodes used in this package
#include "pyrobosim_btcpp/nodes/battery_nodes.hpp"
#include "pyrobosim_btcpp/nodes/close_node.hpp"
#include "pyrobosim_btcpp/nodes/detect_object_node.hpp"
#include "pyrobosim_btcpp/nodes/execute_task_node.hpp"
#include "pyrobosim_btcpp/nodes/get_current_location_node.hpp"
#include "pyrobosim_btcpp/nodes/navigate_node.hpp"
#include "pyrobosim_btcpp/nodes/open_node.hpp"
#include "pyrobosim_btcpp/nodes/pick_object_node.hpp"
#include "pyrobosim_btcpp/nodes/place_object_node.hpp"

// TO_WORKSHOP_USER: add here the include to your custom actions, if you have any

#include <pyrobosim_msgs/msg/robot_state.hpp>

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

static const auto kLogger = rclcpp::get_logger("bt_action_server");

class FlexibleBTActionServer : public BT::TreeExecutionServer
{
public:
  FlexibleBTActionServer(const rclcpp::NodeOptions& options) : TreeExecutionServer(options)
  {
    // here we assume that the battery voltage is published as a std_msgs::msg::Float32
    auto global_blackboard = globalBlackboard();
    robot_state_sub_ = node()->create_subscription<pyrobosim_msgs::msg::RobotState>(
        "/robot/robot_state", 10,
        [global_blackboard](const pyrobosim_msgs::msg::RobotState::SharedPtr msg) {
          global_blackboard->set("battery_level", msg->battery_level);
          global_blackboard->set("holding_object", msg->holding_object);
          global_blackboard->set("last_visited_location", msg->last_visited_location);
          global_blackboard->set("executing_action", msg->executing_action);
        });

    // Note that the callback above and the execution of the tree accessing the
    // global blackboard happen in two different threads.
    // The former runs in the MultiThreadedExecutor, while the latter in the thread created
    // by TreeExecutionServer. But this is OK because the blackboard is thread-safe.
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    RCLCPP_INFO(kLogger, "onTreeCreated ...");
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
    tick_count_ = 0;
  }

  /**
   * @brief onTreeExecutionCompleted is a callback invoked after the tree execution is completed,
   * i.e. if it returned SUCCESS/FAILURE or if the action was cancelled by the Action Client.
   *
   * @param status The status of the tree after the last tick
   * @param was_cancelled True if the action was cancelled by the Action Client
   *
   * @return if not std::nullopt, the string will be sent as [return_message] to the Action Client.
  */
  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                      bool was_cancelled) override
  {
    RCLCPP_INFO(kLogger, "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
                int(status), was_cancelled, tick_count_);
    logger_cout_.reset();
    std::string result =
        "pyrobosim_flexbe_btcpp tree completed with status=" + std::to_string(int(status)) +
        " after " + std::to_string(tick_count_) + " ticks";

    return result;
  }

  /**
   * @brief Callback invoked when a goal is received and before the tree is created.
   * If it returns false, the goal will be rejected.
  */
  bool onGoalReceived(const std::string& tree_name, const std::string& payload)
  {
    RCLCPP_INFO(kLogger, "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
                payload.c_str());
    return true;
  }

  /**
   * @brief registerNodesIntoFactory is a callback invoked after the
   * plugins were registered into the BT::BehaviorTreeFactory.
   * It can be used to register additional custom nodes manually.
   *
   * @param factory The factory to use to register nodes
  */
  void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)
  {
    RCLCPP_INFO(kLogger, "registerNodesIntoFactory - load custom nodes ...");
    // all the actions are done using the same Action Server.
    // Therefore  single RosNodeParams will do.
    BT::RosNodeParams params;
    params.nh = this->node();
    params.default_port_value = "execute_action";

    factory.registerNodeType<BT::CloseAction>("Close", params);
    factory.registerNodeType<BT::DetectObject>("DetectObject", params);
    factory.registerNodeType<BT::IsBatteryLow>("IsBatteryLow", this->node()->get_logger());
    factory.registerNodeType<BT::IsBatteryFull>("IsBatteryFull", this->node()->get_logger());
    factory.registerNodeType<BT::NavigateAction>("Navigate", params);
    factory.registerNodeType<BT::OpenAction>("Open", params);
    factory.registerNodeType<BT::PickObject>("PickObject", params);
    factory.registerNodeType<BT::PlaceObject>("PlaceObject", params);
    RCLCPP_INFO(kLogger, "registerNodesIntoFactory - registered custom nodes.");
  }

  /**
   * @brief onLoopAfterTick invoked at each loop, after tree.tickOnce().
   * If it returns a valid NodeStatus, the tree will stop and return that status.
   * Return std::nullopt to continue the execution.
   *
   * @param status The status of the tree after the last tick
  */
  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus /*status*/)
  {
    //RCLCPP_INFO(kLogger, "pyrobosim_flexbe_btcpp - onLoopAfterTick.");
    ++tick_count_;
    return std::nullopt;
  }

  /**
   * @brief onLoopFeedback is a callback invoked at each loop, after tree.tickOnce().
   * If it returns a valid string, it will be sent as feedback to the Action Client.
   *
   * If you don't want to return any feedback, return std::nullopt.
  */
  std::optional<std::string> onLoopFeedback()
  {
    return std::nullopt;
  }

private:
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
  rclcpp::Subscription<pyrobosim_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  uint32_t tick_count_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<FlexibleBTActionServer>(options);

  RCLCPP_INFO(kLogger, "starting pyrobosim_flexbe_btcpp server ...");

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  RCLCPP_INFO(kLogger, "begin spinning the pyrobosim_flexbe_btcpp server ...");
  exec.spin();

  RCLCPP_INFO(kLogger, "done spinning the pyrobosim_flexbe_btcpp server ...");
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}
