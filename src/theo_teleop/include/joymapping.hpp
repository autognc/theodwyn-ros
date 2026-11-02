#ifndef THEODWYNROS_THEOTELEOP_JOYMAPPING_
#define THEODWYNROS_THEOTELEOP_JOYMAPPING_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "theo_msgs/msg/theo_servo_cmd.hpp"

#include <vector>
#include <cstdint>
#include <string>
#include <cmath>

namespace Theodwyn
{
    /**
     * @brief Node that maps joystick input to chassis velocity and pan/tilt commands.
     *
     * Output is only produced while teleop has authority (see authorityCallback()).
     */
    class JoyMapping : public rclcpp::Node
    {
    public:
        /// @brief Whether joystick commands are currently forwarded.
        enum class TeleopMode : int8_t
        {
            On, ///< Teleop has authority; joystick input is processed
            Off ///< Another controller has authority; joystick input is ignored
        };

        /// @brief Declares parameters and creates the subscriptions and publishers.
        JoyMapping();

    private:
        /// Subscription to joystick messages.
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        /// Subscription to the teleop authority flag.
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr authority_subscription_;
        /// Publisher of chassis velocity commands.
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_publisher_;
        /// Publisher of pan/tilt velocity commands.
        rclcpp::Publisher<theo_msgs::msg::TheoServoCmd>::SharedPtr pantilt_publisher_;
        /// Timer driving timerCallback().
        rclcpp::TimerBase::SharedPtr timer_;

        /**
         * @brief Converts a joystick message into chassis and pan/tilt commands and adjusts speed settings.
         *        Does nothing while teleop is Off.
         * @param msg Joystick state.
         */
        void mappingCallback(const sensor_msgs::msg::Joy::SharedPtr);

        /**
         * @brief Sets the teleop mode from the authority flag (true -> On, false -> Off).
         * @param msg Authority flag.
         */
        void authorityCallback(const std_msgs::msg::Bool::SharedPtr);

        /// @brief Periodic step: checks heartbeat of Joystick.
        void timerCallback();

        /// Maximum chassis linear speed [m/s].
        double chassis_max_linspeed_;
        /// Maximum chassis angular speed [rad/s].
        double chassis_max_angspeed_;
        /// Maximum pan/tilt speed [rad/s].
        double pantilt_maxspeed_;
        /// Minimum time between successive speed-setting changes [s].
        double settings_update_waitime_;
        /// If true, speed-setting changes are locked out.
        bool speed_locked_;
        /// Joystick timeout in seconds
        double timeout_;
        /// Time of the last speed-setting change.
        rclcpp::Time last_settings_update_;
        /// Current teleop mode.
        TeleopMode mode_;
        /// Time of last joystick command
        rclcpp::Time last_joy_time_;
        // Stick axis configuration threshold
        static constexpr double stick_conf_thresh_ = 0.99;
        static constexpr double settings_change_thresh_ = 0.1;

        /// Default for chassis_max_linspeed_.
        static constexpr double DEFAULT_chassis_max_linspeed_ = 1.0;
        /// Default for chassis_max_angspeed_.
        static constexpr double DEFAULT_chassis_max_angspeed_ = 1.0;
        /// Default for pantilt_maxspeed_.
        static constexpr double DEFAULT_pantilt_maxspeed_ = 1.57;
        /// Default for settings_update_waitime_.
        static constexpr double DEFAULT_settings_update_waitime_ = 0.5;
        /// Default for speed_locked_.
        static constexpr bool DEFAULT_speed_locked_ = true;
        /// Default for timeout_
        static constexpr double DEFAULT_timeout_ = 0.25;
        /// Default joystick input topic.
        static constexpr std::string_view DEFAULT_joy_topic_ = "joy";
        /// Default authority topic shared with the autonomous controller.
        static constexpr std::string_view DEFAULT_joyauthority_topic_ = "joy_authority";
        /// Default chassis velocity command topic.
        static constexpr std::string_view DEFAULT_velcmd_topic_ = "velcmd_topic";
        /// Default pan/tilt command topic.
        static constexpr std::string_view DEFAULT_servocmd_topic_ = "servocmd_topic";
    };
}

#endif
