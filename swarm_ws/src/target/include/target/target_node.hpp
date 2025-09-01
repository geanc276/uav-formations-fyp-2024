/*
 *  target_node.hpp
 *
 *  This is an abstract interface class for all targets.
 *  All targets must implement a position publisher, that sends information 
 *  to the swarm in the form of a PX4 Global Position message.
 * 
 */

#ifndef TARGET_NODE_HPP
#define TARGET_NODE_HPP

class Target : public rclcpp::Node
{
public:
    Target() : Node("target") {};
    virtual void publish_position(void) = 0;
protected:
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr position_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif /* TARGET_NODE_HPP */