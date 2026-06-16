#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include "question_3_charger_interfaces/srv/go_to_charger.hpp"

/*
 * Service Server -- Version A
 *
 * Simulates a 30 second trip to the charging station.
 * The problem: this server blocks for 30 seconds before sending a response.
 * If the client has a 5 second timeout, it gives up and never gets the result.
 *
 * This shows why services are a bad choice for long running tasks.
 */

class ChargerServiceServer : public rclcpp::Node
{
public:
  ChargerServiceServer() : Node("charger_service_server")
  {
    server_ = this->create_service<question_3_charger_interfaces::srv::GoToCharger>(
      "go_to_charger",
      std::bind(&ChargerServiceServer::handleRequest, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service server ready. Waiting for requests...");
  }

private:

  void handleRequest(
    const question_3_charger_interfaces::srv::GoToCharger::Request::SharedPtr req,
    question_3_charger_interfaces::srv::GoToCharger::Response::SharedPtr res)
  {
    RCLCPP_INFO(this->get_logger(),
      "Got request for: %s. Starting 30 second trip...", req->destination.c_str());

    // blocks here for 30 seconds -- client will timeout waiting for this
    std::this_thread::sleep_for(std::chrono::seconds(30));

    res->success     = true;
    res->travel_time = 30.0f;
    res->message     = "Arrived at charging station!";

    // by the time we get here, the client has already given up
    RCLCPP_INFO(this->get_logger(), "Trip done. Sending response (client may have already timed out).");
  }

  rclcpp::Service<question_3_charger_interfaces::srv::GoToCharger>::SharedPtr server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChargerServiceServer>());
  rclcpp::shutdown();
  return 0;
}