// ALL DESCRIPTIONS ARE SUBJECT TO CHANGE AS UNDERSTANDING IS OBTAINED //
/*
 *   Drone target node to run on target drone. Simply publishes
 *   target drone location repeatedly.
 *
 *   File: drone_target.cpp
 *   Date: 10-7-2024
 *   Author: Benjamin Ireland
 */

#include <chrono>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "target/target_node.hpp"
#include "logging/logging.hpp"
#include "shared_packages/throttled_subscriber.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

#ifndef SIM_BUILD
#define SIM_BUILD 0
#endif

#define TARGET_QUEUE_SIZE 10

class DroneTarget : public Target {
public:
	DroneTarget() : Target() {

		// Declare the "spin_rate" parameter with a default value of 20.0.
		this->declare_parameter<double>("spin_rate", 60.0);

		// Declare the "throttle_rate" parameter with a default value of 10.0.
		this->declare_parameter<double>("throttle_rate", 15.0);


		/* Quality of Service set-up for PX4 topics. */
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		// Create the publisher for target position.
		position_publisher_ = this->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
			"/target/global_position", TARGET_QUEUE_SIZE);

		// Create the throttled subscriber using our wrapper.
		if (SIM_BUILD) {
			target_drone_pos_sub_ =
				shared_packages::ThrottledSubscriber<px4_msgs::msg::VehicleGlobalPosition>::create(
					this,
					"/px4_250/fmu/out/vehicle_global_position",
					qos);
		} else {
			target_drone_pos_sub_ =
				shared_packages::ThrottledSubscriber<px4_msgs::msg::VehicleGlobalPosition>::create(
					this,
					"/px4_250/fmu/out/vehicle_global_position",
					qos);
		}

		// Set the throttle period. For a throttle frequency of 10 Hz, period = 0.1 seconds.
		target_drone_pos_sub_->set_throttle_period(0.1);

		// Set the user callback. The callback expects a const reference.
		target_drone_pos_sub_->set_callback(
			std::bind(&DroneTarget::target_pose_cb, this, std::placeholders::_1)
		);

		LOG_MISSION_DEBUG(this, "%s: Drone target node initialized", this->get_name());
	}

	void set_throttle_frequency(double frequency)
	{
		target_drone_pos_sub_->set_throttle_frequency(frequency);
	}
private:
	rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr position_publisher_;
	// Our throttled subscriber wrapper.
	shared_packages::ThrottledSubscriber<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr target_drone_pos_sub_;
	px4_msgs::msg::VehicleGlobalPosition msg_{};

	// This is the user callback that will be throttled.
	void target_pose_cb(const px4_msgs::msg::VehicleGlobalPosition & msg) {
		LOG_MISSION_DEBUG(this, "%s: Target pose callback, lat: %f, lon: %f, alt: %f",this->get_name(), msg.lat, msg.lon, msg.alt);

		msg_.lat = msg.lat;
		msg_.lon = msg.lon;
		// For now, send zero altitude.
		msg_.alt = 0;
		publish_position();
	}

	void publish_position() {
		position_publisher_->publish(msg_);
	}
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	auto drone_target_node = std::make_shared<DroneTarget>();

	double spin_rate = 50.0;  // default value
	double throttle_rate_hz = 10.0;  // default value
	drone_target_node->get_parameter("spin_rate", spin_rate);
	drone_target_node->get_parameter("throttle_rate", throttle_rate_hz);
	drone_target_node->set_throttle_frequency(throttle_rate_hz);
	rclcpp::Rate loop_rate(spin_rate);


	LOG_SETUP_DEBUG(drone_target_node, "Drone target node started with spin rate: %f and throttle rate: %f",
		spin_rate, throttle_rate_hz);

	while (rclcpp::ok()) {
		rclcpp::spin_some(drone_target_node);
		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}