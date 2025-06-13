// ALL DESCRIPTIONS ARE SUBJECT TO CHANGE AS UNDERSTANDING IS OBTAINED //
/*
 *   Virtual target node to run on target drone. Simply publishes
 *   target location repeatedly.
 *
 *   File: target.cpp
 *   Date: 10-7-2024
 *   Author: Benjamin Ireland
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include <controller_msgs/msg/ref_point_global_position.hpp>
#include "target/libs/global_pose.hpp"
#include "target/libs/state.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include "target/target_node.hpp"


using namespace std::chrono;
using namespace std::chrono_literals;

#define TARGET_QUEUE_SIZE 10
#define SIDE_LENGTH 20
#define FACTOR 1000
#define LOOP_SPEED 150ms

class VirtualTarget : public Target
{
public:
	VirtualTarget()
		: Target()
	{
		/* Quality of Service set-up for PX4 topics. */
    	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
   		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		position_publisher_ = this->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
			"/target/global_position", TARGET_QUEUE_SIZE);

		ctrl_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        	"/ctrl/state", qos, std::bind(&VirtualTarget::ctrl_state_cb, this, std::placeholders::_1));

		reference_point_sub_ = this->create_subscription<controller_msgs::msg::RefPointGlobalPosition>(
        	"/reference_point/global_position", qos,
        	std::bind(&VirtualTarget::rp_pos_cb, this, std::placeholders::_1));


		//! Subscribe to the global position of the first drone ONLY FOR SIM AND SHOULD BE MADE MORE ADAPTABLE
		vehicle_1_global_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
			"/px4_1/fmu/out/vehicle_global_position", qos,
			std::bind(&VirtualTarget::vehicle_pos_cb, this, std::placeholders::_1));
		timer_ = this->create_wall_timer(LOOP_SPEED, std::bind(&VirtualTarget::publish_position, this));
	}

private:
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ctrl_state_sub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr position_publisher_;
	rclcpp::Subscription<controller_msgs::msg::RefPointGlobalPosition>::SharedPtr reference_point_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_1_global_pos_sub_;
	
	state_t state_;
	bool initialized_ = false;
	bool moving_ = false;
	GlobalPose global_position_ = GlobalPose(0, 0 ,0);

	void line(void);
	void square(void);
	void ctrl_state_cb(const std_msgs::msg::UInt8::SharedPtr msg);
	void rp_pos_cb(const controller_msgs::msg::RefPointGlobalPosition::SharedPtr msg);
	void vehicle_pos_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

	void publish_position(void)
	{


		if (!initialized_)
		{
			RCLCPP_INFO(this->get_logger(), "NOT INITIALIZED");
			return;
			// global_position_.set_position(0, 0, 0);
			// initialized_ = true;
		}
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: %d", state_);
		if (state_ == MOVE){
		// if (state_ == MOVE || state_ == RENDEZVOUS)
		// {
			moving_ = true;
			// line();
			square();
			px4_msgs::msg::VehicleGlobalPosition msg{};
			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			msg.lat = global_position_.get_position()[0];
			msg.lon = global_position_.get_position()[1];
			msg.alt = global_position_.get_position()[2];
			position_publisher_->publish(msg);
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Position 3d: %f, %f, %f", msg.lat, msg.lon, msg.alt);
		}
		else
		{
			moving_ = false;
		}
	}
};

void VirtualTarget::ctrl_state_cb(const std_msgs::msg::UInt8::SharedPtr msg)
{
    state_ = (state_t)msg->data;
}

void VirtualTarget::rp_pos_cb(const controller_msgs::msg::RefPointGlobalPosition::SharedPtr msg)
{
	// if (!moving_)
	// {
	// 	RCLCPP_INFO(this->get_logger(), "Setting initial position");
	// 	RCLCPP_INFO(this->get_logger(), "CURRENT Position: %f, %f, %f", global_position_.get_position()[0], global_position_.get_position()[1], global_position_.get_position()[2]);
	// 	RCLCPP_INFO(this->get_logger(), "RECEIVED Position: %f, %f, %f", msg->lat, msg->lon, msg->alt);
		
	// 	global_position_.set_position(msg->lat, msg->lon, msg->alt);
	// 	RCLCPP_INFO(this->get_logger(), "Position saved: %f, %f, %f", global_position_.get_position()[0], global_position_.get_position()[1], global_position_.get_position()[2]);

	// }
}

void VirtualTarget::vehicle_pos_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
	if (!initialized_)
	{
		RCLCPP_INFO(this->get_logger(), "Setting initial position");
		RCLCPP_INFO(this->get_logger(), "Position: %f, %f, %f", msg->lat, msg->lon, msg->alt);
		global_position_.set_position(msg->lat, msg->lon, msg->alt);
		RCLCPP_INFO(this->get_logger(), "Position saved: %f, %f, %f", global_position_.get_position()[0], global_position_.get_position()[1], global_position_.get_position()[2]);
		initialized_ = true;
	}
}
void VirtualTarget::line(void)
{
	double increment = 1;
	static bool done = false;
	if (!done)
	{
		global_position_.x_offset(increment);
		done = true;
	}

}

void VirtualTarget::square(void)
{
	static int direction = 0;
	double increment = 0.21;
	// double increment = SIDE_LENGTH/FACTOR;
	static double distance = 0;

	switch (direction)
	{
		case 0: // N
			global_position_.x_offset(increment);
			break;
		case 1:	// E
			global_position_.y_offset(increment);
			break;
		case 2: // S
			global_position_.x_offset(-increment);
			break;
		case 3:	// W
			global_position_.y_offset(-increment);
			break;
		default:
			break;
	}

	distance += increment;
	if (distance >= SIDE_LENGTH)
	{
		direction++;
		direction = direction % 4;
		distance = 0;
	}

}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting target node");
	rclcpp::spin(std::make_shared<VirtualTarget>());
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down target node");

	rclcpp::shutdown();
	return 0;
}
