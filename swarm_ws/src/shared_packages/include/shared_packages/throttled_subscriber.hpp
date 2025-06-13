#ifndef SHARED_PACKAGES_THROTTLED_SUBSCRIBER_HPP
#define SHARED_PACKAGES_THROTTLED_SUBSCRIBER_HPP

#include <functional>
#include <memory>
#include "logging/logging.hpp"
#include <rclcpp/rclcpp.hpp>



namespace shared_packages {

template<typename T>
class ThrottledSubscriber {
public:
	typedef std::shared_ptr<ThrottledSubscriber<T>> SharedPtr;

	// Construct a throttled subscriber by creating an underlying subscription.
	ThrottledSubscriber(
		rclcpp::Node * node,
		const std::string & topic,
		const rclcpp::QoS & qos)
		: node_(node),
		  throttle_period_(0.0),
		  last_callback_time_(0.0),
		  clock_(rclcpp::Clock(RCL_ROS_TIME))
	{
		// Create a regular subscription whose lambda forwards messages to our throttling logic.
		subscription_ = node_->create_subscription<T>(
			topic, qos,
			[this](typename T::ConstSharedPtr msg) {
				if (msg) {
					this->call_callback(*msg);
				}
			});
	}

	// Factory method.
	static SharedPtr create(
		rclcpp::Node * node,
		const std::string & topic,
		const rclcpp::QoS & qos)
	{
		return std::make_shared<ThrottledSubscriber<T>>(node, topic, qos);
	}

	// Set the throttle period (in seconds). For example, a period of 0.1 equals ~10 Hz.
	void set_throttle_period(double period)
	{
		throttle_period_ = period;
	}

    void set_throttle_frequency(double frequency)
    {
        throttle_period_ = 1.0 / frequency;
    }
	// Set the user callback. The callback must be callable with a parameter of type "const T &".
	template<typename Func, typename... Args>
	void set_callback(Func&& func, Args&&... args)
	{
		user_callback_ = std::bind(std::forward<Func>(func),
								   std::placeholders::_1,
								   std::forward<Args>(args)...);
	}

private:
	// Internal throttling logic.
	void call_callback(const T & message)
	{
		LOG_MISSION_DEBUG(node_, "%s: ThrottledSubscriber callback", node_->get_name());
		if (!user_callback_) {
			return;
		}
		if (throttle_period_ <= 0.0) {
			user_callback_(message);
			return;
		}
		double current_time = clock_.now().seconds();
		if (current_time - last_callback_time_ >= throttle_period_) {
			last_callback_time_ = current_time;
			LOG_MISSION_DEBUG(node_, "%s: ThrottledSubscriber callback is being called", node_->get_name());
			user_callback_(message);

		}
	}

	// The underlying ROS subscription.
	typename rclcpp::Subscription<T>::SharedPtr subscription_;
	// User-supplied callback.
	std::function<void(const T&)> user_callback_;
	// Throttle period in seconds.
	double throttle_period_;
	// Time when the callback was last invoked.
	double last_callback_time_;
	// A stored clock to avoid re-creating it on every callback.
	rclcpp::Clock clock_;
	// The node on which the subscription is created (not owned).
	rclcpp::Node * node_;
};

} // namespace shared_packages

#endif // SHARED_PACKAGES_THROTTLED_SUBSCRIBER_HPP