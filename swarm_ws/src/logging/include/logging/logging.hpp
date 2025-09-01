#ifndef LOGGING_HPP
#define LOGGING_HPP

// Helper macro to concatenate a prefix with the format string
#define CONCAT_PREFIX(prefix, frmtstring) prefix " : " frmtstring

// Helper macro to handle variadic arguments properly
#define LOG_IMPL_NODE(logger_func, node, frmtstring, ...) \
    logger_func(node->get_logger(), CONCAT_PREFIX(frmtstring, "") __VA_OPT__(, ) __VA_ARGS__)

#define LOG_IMPL(logger_func, logger, frmtstring, ...) \
    logger_func(logger, CONCAT_PREFIX(frmtstring, "") __VA_OPT__(, ) __VA_ARGS__)

// Optimized logging macros using node-based loggers with adaptable prefixes
#define LOG_DEBUGGING_INFO(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_DEBUG, node, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_INFO(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_INFO, node, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_DEBUG(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_DEBUG, node, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_WARN(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_WARN, node, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_ERROR(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_ERROR, node, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_FATAL(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_FATAL, node, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_POSITION(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_INFO, node, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)


#define LOG_MISSION_INFO_THROTTLE(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_INFO_THROTTLE(node->get_logger(), 1), node, "INFO: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_DEBUG_THROTTLE(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_DEBUG_THROTTLE(node->get_logger(), 1), node, "DEBUG: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_WARN_THROTTLE(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_WARN_THROTTLE(node->get_logger(), 1), node, "WARN: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_ERROR_THROTTLE(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_ERROR_THROTTLE(node->get_logger(), 1), node, "ERROR: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_FATAL_THROTTLE(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_FATAL_THROTTLE(node->get_logger(), 1), node, "FATAL: " frmtstring __VA_OPT__(, ) __VA_ARGS__)



// Logging macros for logger-based logging with adaptable prefixes
#define LOG_DEBUGGING_INFO_LOGGER(logger, frmtstring, ...) LOG_IMPL(RCLCPP_DEBUG, logger, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_INFO_LOGGER(logger, frmtstring, ...) LOG_IMPL(RCLCPP_INFO, logger, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_DEBUG_LOGGER(logger, frmtstring, ...) LOG_IMPL(RCLCPP_DEBUG, logger, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_WARN_LOGGER(logger, frmtstring, ...) LOG_IMPL(RCLCPP_WARN, logger, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_ERROR_LOGGER(logger, frmtstring, ...) LOG_IMPL(RCLCPP_ERROR, logger, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_MISSION_FATAL_LOGGER(logger, frmtstring, ...) LOG_IMPL(RCLCPP_FATAL, logger, "MISSION: " frmtstring __VA_OPT__(, ) __VA_ARGS__)





#define LOG_SETUP_INFO(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_INFO, node, "SETUP: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_SETUP_DEBUG(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_DEBUG, node, "SETUP: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_SETUP_WARN(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_WARN, node, "SETUP: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_SETUP_ERROR(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_ERROR, node, "SETUP: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_SETUP_FATAL(node, frmtstring, ...) LOG_IMPL_NODE(RCLCPP_FATAL, node, "SETUP: " frmtstring __VA_OPT__(, ) __VA_ARGS__)

#define LOG_SETUP_GLOBAL_INFO(frmtstring, ...) LOG_IMPL(RCLCPP_INFO, rclcpp::get_logger("rclcpp"), "INFO: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_SETUP_GLOBAL_DEBUG(frmtstring, ...) LOG_IMPL(RCLCPP_DEBUG, rclcpp::get_logger("rclcpp"), "DEBUG: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_SETUP_GLOBAL_WARN(frmtstring, ...) LOG_IMPL(RCLCPP_WARN, rclcpp::get_logger("rclcpp"), "WARN: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_SETUP_GLOBAL_ERROR(frmtstring, ...) LOG_IMPL(RCLCPP_ERROR, rclcpp::get_logger("rclcpp"), "ERROR: " frmtstring __VA_OPT__(, ) __VA_ARGS__)
#define LOG_SETUP_GLOBAL_FATAL(frmtstring, ...) LOG_IMPL(RCLCPP_FATAL, rclcpp::get_logger("rclcpp"), "FATAL: " frmtstring __VA_OPT__(, ) __VA_ARGS__)



// Function entry logging with a default "DEBUG" prefix
#define LOG_FUNCTION_ENTRY(node) RCLCPP_DEBUG(node->get_logger(), "DEBUG: Entered function: %s", __FUNCTION__)

#endif // LOGGING_HPP