#pragma once

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace curobo_rviz
{
  // Spins a node on a dedicated background thread for the lifetime of this object.
  //
  // RViz panels run their Qt slots/timers on the GUI thread and never spin the
  // rclcpp::Node they own. Without something spinning the node, async service/action/
  // parameter callbacks never fire, and the ROS "synchronous" helper APIs
  // (SyncParametersClient::get_parameters, rclcpp::spin_until_future_complete, ...)
  // each build their own temporary executor and block the calling thread until the
  // call resolves -- which freezes the whole Qt event loop if the target service is
  // slow to respond (e.g. cuRobo GPU warmup). NodeSpinner replaces that pattern: the
  // node is spun continuously in the background, so panels only ever need to use the
  // fully-async APIs (async_send_request, AsyncParametersClient) and marshal results
  // back to the GUI thread themselves.
  class NodeSpinner
  {
  public:
    explicit NodeSpinner(const rclcpp::Node::SharedPtr & node)
      : executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {
      executor_->add_node(node);
      thread_ = std::thread([this]() { executor_->spin(); });
    }

    ~NodeSpinner()
    {
      executor_->cancel();
      if (thread_.joinable()) {
        thread_.join();
      }
    }

    NodeSpinner(const NodeSpinner &) = delete;
    NodeSpinner & operator=(const NodeSpinner &) = delete;

  private:
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread thread_;
  };
} // namespace curobo_rviz
