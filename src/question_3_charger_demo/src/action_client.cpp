#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "question_3_charger_interfaces/action/go_to_charger.hpp"

/*
 * Action Client -- Version B
 *
 * Sends a goal and stays connected for the full 30 seconds.
 * No timeout issue because action goals are async -- the client
 * just waits for feedback and result callbacks instead of blocking.
 *
 * Three callbacks handle everything:
 *   goal_response_callback -- did server accept or reject the goal?
 *   feedback_callback      -- progress update every second
 *   result_callback        -- final result when trip is done
 */

using GoToCharger       = question_3_charger_interfaces::action::GoToCharger;
using GoalHandleCharger = rclcpp_action::ClientGoalHandle<GoToCharger>;

class ChargerActionClient : public rclcpp::Node
{
public:
  ChargerActionClient() : Node("charger_action_client")
  {
    client_ = rclcpp_action::create_client<GoToCharger>(this, "go_to_charger");

    // send the goal once the node is up
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ChargerActionClient::sendGoal, this));
  }

private:

  void sendGoal()
  {
    timer_->cancel();  // only send once

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available. Exiting.");
      rclcpp::shutdown();
      return;
    }

    GoToCharger::Goal goal_msg;
    goal_msg.destination = "charging_station_A";

    rclcpp_action::Client<GoToCharger>::SendGoalOptions options;

    // called once when server accepts or rejects the goal
    options.goal_response_callback =
      [this](const GoalHandleCharger::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted -- receiving feedback...");
        }
      };

    // called every second with distance remaining
    options.feedback_callback =
      [this](GoalHandleCharger::SharedPtr,
             const std::shared_ptr<const GoToCharger::Feedback> fb) {
        RCLCPP_INFO(this->get_logger(),
          "[feedback] Distance remaining: %.1f m", fb->distance_remaining);
      };

    // called once when the trip is fully complete
    options.result_callback =
      [this](const GoalHandleCharger::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(),
            "Result: %s  Travel time: %.1fs",
            result.result->message.c_str(),
            result.result->travel_time);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Goal did not succeed");
        }
        rclcpp::shutdown();
      };

    RCLCPP_INFO(this->get_logger(), "Sending goal to action server...");

    // async -- returns immediately, no blocking, no timeout issue
    client_->async_send_goal(goal_msg, options);
  }

  rclcpp_action::Client<GoToCharger>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargerActionClient>());
  rclcpp::shutdown();
  return 0;
}