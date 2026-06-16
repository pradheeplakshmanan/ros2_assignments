#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "question_3_charger_interfaces/srv/go_to_charger.hpp"

/*
 * Service Client -- Version A
 *
 * Sends a request to the charger service and waits up to 5 seconds for a reply.
 * The server takes 30 seconds to respond, so the client always times out.
 *
 * Key point: once the client times out, the server is still running with no way
 * to cancel it. The response goes nowhere -- nobody is listening anymore.
 */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("charger_service_client");

  auto client = node->create_client<question_3_charger_interfaces::srv::GoToCharger>(
    "go_to_charger");

  // wait until the service is available
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node->get_logger(), "Waiting for service to come online...");
  }

  auto request = std::make_shared<
    question_3_charger_interfaces::srv::GoToCharger::Request>();
  request->destination = "charging_station_A";

  RCLCPP_INFO(node->get_logger(), "Requesting charging station trip via SERVICE...");
  RCLCPP_INFO(node->get_logger(), "Waiting for response (5s timeout)...");

  auto future = client->async_send_request(request);

  // wait up to 5 seconds for a reply
  // server takes 30 seconds so this will always return TIMEOUT
  auto status = rclcpp::spin_until_future_complete(
    node, future, std::chrono::seconds(5));

  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    RCLCPP_INFO(node->get_logger(),
      "Result: %s  travel time: %.1fs",
      response->message.c_str(), response->travel_time);
  }
  else if (status == rclcpp::FutureReturnCode::TIMEOUT) {
    // client gives up here -- but server is still sleeping for 25 more seconds
    // there is no way to cancel the server from here
    RCLCPP_ERROR(node->get_logger(),
      "TIMED OUT -- service did not respond within 5 seconds!");
    RCLCPP_WARN(node->get_logger(),
      "Server is still running but client already quit. Response will be lost.");
  }
  else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}