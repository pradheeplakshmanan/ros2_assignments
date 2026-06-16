#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include "question_3_charger_interfaces/action/go_to_charger.hpp"

/*
 * Action Server -- Version B
 *
 * Same 30 second trip, but done properly using an action.
 * Every second we publish feedback so the client knows the progress.
 * The execution runs in a separate thread so the main thread is never blocked.
 *
 * This is the right way to handle long running tasks in ROS2.
 */

using GoToCharger      = question_3_charger_interfaces::action::GoToCharger;
using GoalHandleCharger = rclcpp_action::ServerGoalHandle<GoToCharger>;

class ChargerActionServer : public rclcpp::Node
{
public:
  ChargerActionServer() : Node("charger_action_server")
  {
    server_ = rclcpp_action::create_server<GoToCharger>(
      this,
      "go_to_charger",
      std::bind(&ChargerActionServer::handleGoal,     this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&ChargerActionServer::handleCancel,   this, std::placeholders::_1),
      std::bind(&ChargerActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action server ready. Waiting for goals...");
  }

private:

  // accept every goal that comes in
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const GoToCharger::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received goal: go to %s", goal->destination.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // accept cancel requests
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleCharger> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // spin up a thread so execution never blocks the main thread
  void handleAccepted(const std::shared_ptr<GoalHandleCharger> goal_handle)
  {
    std::thread([this, goal_handle]() {
      execute(goal_handle);
    }).detach();
  }

  void execute(const std::shared_ptr<GoalHandleCharger> goal_handle)
  {
    const float total_distance = 30.0f;
    float distance_remaining   = total_distance;

    auto feedback = std::make_shared<GoToCharger::Feedback>();
    auto result   = std::make_shared<GoToCharger::Result>();

    RCLCPP_INFO(this->get_logger(), "Starting 30 second trip...");

    // send feedback every second for 30 seconds
    for (int i = 0; i < 30; ++i) {

      // stop immediately if client cancelled
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Trip cancelled";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        return;
      }

      distance_remaining = total_distance - static_cast<float>(i + 1);

      feedback->distance_remaining = distance_remaining;
      feedback->elapsed_time       = static_cast<float>(i + 1);

      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(),
        "[feedback] Distance remaining: %.1f m", distance_remaining);

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // trip finished -- send the final result
    result->success     = true;
    result->travel_time = 30.0f;
    result->message     = "Arrived at charging station!";
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Trip complete. Result sent.");
  }

  rclcpp_action::Server<GoToCharger>::SharedPtr server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargerActionServer>());
  rclcpp::shutdown();
  return 0;
}