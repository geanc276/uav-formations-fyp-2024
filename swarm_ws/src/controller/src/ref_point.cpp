/*
 *   The reference point node. Controls the location and movement of the
 *   virtual reference point at the centre of the swarm formation.
 *
 *   File: ref_point.cpp
 *   Date: 3-7-2024
 *   Author: Benjamin Ireland, Finlay Cross
 */

#include "controller/ref_point.hpp"
#include "logging/logging.hpp"

//  MACROS  //

using namespace std::chrono_literals;
#define REF_POINT_PUB_RATE 10ms


//  CONSTRUCTOR  //


ReferencePoint::ReferencePoint()
    : Node("reference_point")
{
    LOG_SETUP_INFO(this, "Reference Point Node Initialised");
    this->get_parameters();
    this->config_ = RPConfig(this->config_file_path_);
    this->init();
    LOG_SETUP_INFO(this, "Reference Point Node Finished Initialisation");
}


void ReferencePoint::get_parameters()
{

    
    // Declare and retrieve a string parameter
    LOG_SETUP_INFO(this, "Getting parameters for reference point");
    try {
        this->declare_parameter<std::string>("config_file_path", "default_config.yaml");
        this->get_parameter("config_file_path", config_file_path_);

        this->declare_parameter<int>("drone_id", 1);
        this->get_parameter("drone_id", drone_id_);

    }
    catch (const rclcpp::ParameterTypeException &e) {
    }

    // // Declare and retrieve an integer parameter

    // try {
    //     this->declare_parameter<int>("num_drones", 1);
    //     this->get_parameter("num_drones", num_of_drones_);
    // } 
    // catch (const rclcpp::ParameterTypeException &e) {
    // }

}



void ReferencePoint::init()
{
    // global_drone_poses_ = new (std::nothrow) GlobalPose[num_of_drones_];
    rp_established_ = false;

    rp_pos_ = GlobalPose("reference_point");
    target_pos_ = GlobalPose("target_global");
    this->init_ros2();
}

/**
 * @brief   Helper function to initialise ros2 publishers and subscribers.
 */
void ReferencePoint::init_ros2()
{
    /* Quality of Service set-up for PX4 topics. */
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // global_drone_pos_subscribers_ = new (std::nothrow) rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr[num_of_drones_];

    // if (global_drone_pos_subscribers_ == nullptr)
    // {

    //     exit(0);
    // }

    /* Subscribers */
    LOG_SETUP_INFO(this, "REFPOINT DRONE ID: %d", drone_id_);
    std::function<void(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)> fn = std::bind(&ReferencePoint::pos_cb, this, std::placeholders::_1);
    global_drone_pos_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/px4_" + std::to_string(drone_id_) + "/fmu/out/vehicle_global_position", qos, fn);


    // TODO For now target is set to drone num_drones + 1
    target_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/target/global_position", qos, std::bind(&ReferencePoint::target_cb, this, std::placeholders::_1));

    ctrl_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/ctrl/state", qos, std::bind(&ReferencePoint::ctrl_state_cb, this, std::placeholders::_1));

    register_drone_sub_ = this->create_subscription<controller_msgs::msg::RegisterDrone>(
        "/register_drone", RP_QUEUE_SIZE, std::bind(&ReferencePoint::drone_register_cb, this, std::placeholders::_1));
    /* Publishers */
    rp_establish_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/reference_point/rp_set", RP_QUEUE_SIZE);

    global_ref_point_pose_pub_ = this->create_publisher<controller_msgs::msg::RefPointGlobalPosition>(
        "/reference_point/global_position", RP_QUEUE_SIZE);

    register_drone_ack_pub_ = this->create_publisher<controller_msgs::msg::RegisterDroneAck>(
        "/register_drone_ack", RP_QUEUE_SIZE);


    update_drone_list_pub_ = this->create_publisher<controller_msgs::msg::UpdateDroneList>(
        "/update_drone_list", RP_QUEUE_SIZE);


    

    /* Timers */
    rp_established_pub_timer_ = this->create_wall_timer(1000ms, std::bind(&ReferencePoint::publish_rp_established, this));

    global_rp_position_pub_timer_ = this->create_wall_timer(REF_POINT_PUB_RATE, std::bind(&ReferencePoint::publish_rp_pos, this));
}

//  GETTERS/SETTERS  //

bool ReferencePoint::get_rp_established()
{
    return rp_established_;
}

GlobalPose ReferencePoint::get_drone_position(int drone_id)
{
    return global_drone_pose_;
}

//  ROS2 CALLBACK FUNCTIONS  //

void ReferencePoint::pos_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    global_drone_pose_.set_position(msg->lat, msg->lon, msg->alt);

    if (!rp_established_)
    {
        LOG_MISSION_DEBUG(this, "Setting reference point FROM DRONE:\t%f, %f, %f", global_drone_pose_.get_position()[0], global_drone_pose_.get_position()[1], global_drone_pose_.get_position()[2]);
        
        set_rp(global_drone_pose_, config_.get_drone_constraint());
        rp_established_ = true;

    }
}

void ReferencePoint::ctrl_state_cb(const std_msgs::msg::UInt8::SharedPtr msg)
{
    LOG_MISSION_DEBUG(this, "Control state: %d", msg->data);
    state_ = (state_t)msg->data;
}



void ReferencePoint::drone_register_cb(const controller_msgs::msg::RegisterDrone::SharedPtr msg)
{
    LOG_MISSION_DEBUG(this, "Drone register callback");
    drone_list_[msg->px4_id] = msg->name;
    drone_register_ack(msg->px4_id);    
    update_drone_list();
}



void ReferencePoint::drone_register_ack(int id)
{
    controller_msgs::msg::RegisterDroneAck msg{};
    msg.id = id;
    register_drone_ack_pub_->publish(msg);
}

void ReferencePoint::update_drone_list()
{
    controller_msgs::msg::UpdateDroneList msg{};
    msg.size = drone_list_.size();
    int i = 0;
    for (const auto& [px4_id, name] : drone_list_) {;
        msg.px4_ids[i] = px4_id;
        i++;
    }


    update_drone_list_pub_->publish(msg);
}





/**
 * @brief   Superimpose reference point position over target position. Temporary implementation maps
 *          target drones txed position ti the reference point position.
 */
void ReferencePoint::target_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    LOG_MISSION_DEBUG(this, "Target position: %f, %f, %f",msg->lat, msg->lon, msg->alt);
    LOG_MISSION_DEBUG(this, "State: %s", state_to_string(state_).c_str());
    // static GlobalPose previous_target_pose = GlobalPose("target_global");
    if(!rp_established_) 
    {
        LOG_SETUP_DEBUG(this, "SETTING REFERENCE POINT TO TARGET");
        target_pos_.set_position(msg->lat, msg->lon, msg->alt);
        set_rp(target_pos_, config_.get_drone_constraint());
        rp_established_=true;
    }
    if (state_ == MOVE)
    {
        // LOG_MISSION_DEBUG(this, "Target position: %f, %f, %f",msg->lat, msg->lon, msg->alt);

        rp_pos_.set_position(msg->lat, msg->lon, msg->alt);
        // rp_pos_.set_heading(rp_pos_ - previous_target_pose);
        target_pos_ = rp_pos_;
        LOG_MISSION_POSITION(this, "%s", target_pos_.jsonify().dump().c_str());
    }
}

//  PUBLISHERS  //

/**
 * @brief   Publishes the GPS position of the reference point.
 */
void ReferencePoint::publish_rp_pos()
{

    controller_msgs::msg::RefPointGlobalPosition msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    msg.lat = rp_pos_.get_position()[0];
    msg.lon = rp_pos_.get_position()[1];
    msg.alt = rp_pos_.get_position()[2];

    msg.terrain_offset = rp_pos_.terrain_offset_;
    msg.heading = rp_pos_.get_heading();

    global_ref_point_pose_pub_->publish(msg);
    LOG_MISSION_POSITION(this,"%s" ,rp_pos_.jsonify().dump().c_str());
}

/**
 * @brief   Publish state of the reference point every second.
 */
void ReferencePoint::publish_rp_established()
{
    if ( !rp_established_)
    {
        LOG_MISSION_DEBUG(this, "Reference point not established");
        return;
    }
    // Alert the drones that the refpoint has been found.
    std_msgs::msg::Bool msg{};
    msg.data = rp_established_;
    rp_establish_pub_->publish(msg);
}

/**
 * @brief   Initialises the start position of the reference point from the first active drone.
 * 
 * @param drone_pos         The drones GPS pose.
 * @param drone_constraint  The drones constrained pose from the reference point.
 */
void ReferencePoint::set_rp(GlobalPose drone_pos, Constraints drone_constraint)
{
    GlobalPose ideal_ref_point = drone_pos;
    // Make sure rp heading starts at 0
    ideal_ref_point.set_heading(0);

    LOG_MISSION_INFO(this, "Setting reference point");

    float rp_angle;
    float drone_angle;
    float dist_x;
    float dist_y;
    position_config_t pos_constraint = drone_constraint.get_pos_cnstr();

    rp_angle = pos_constraint.pos_angle;

    if (rp_angle >= 0)
    {
        drone_angle = rp_angle - STRAIGHT_ANGLE;
    }
    else
    {
        drone_angle = rp_angle + STRAIGHT_ANGLE;
    }

    drone_angle = DEG_TO_RAD(drone_angle);
    dist_x = pos_constraint.dist * cos(drone_angle);
    dist_y = pos_constraint.dist * sin(drone_angle);

    // Shift the GPS position by calculated offset
    ideal_ref_point.x_offset(dist_x);
    ideal_ref_point.y_offset(dist_y);
    rp_pos_ = ideal_ref_point;

    LOG_MISSION_INFO(this, "Reference point set at: %f, %f, %f", rp_pos_.get_position()[0], rp_pos_.get_position()[1], rp_pos_.get_position()[2]);

}

ReferencePoint::~ReferencePoint()
{

}

/**
 *  @brief  ROS 2 reference point node set up.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Create the ROS node
    auto rp_node = std::make_shared<ReferencePoint>();
    rclcpp::spin(rp_node);
    rclcpp::shutdown();
    return 0;
}


















