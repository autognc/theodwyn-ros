#include "joymapping.hpp"

using namespace Theodwyn;

JoyMapping::JoyMapping()
    : Node("joymapping_node"), mode_(TeleopMode::On)
{
    declare_parameter<double>("chassis_max_linspeed", DEFAULT_chassis_max_linspeed_);
    declare_parameter<double>("chassis_max_angspeed", DEFAULT_chassis_max_angspeed_);
    declare_parameter<double>("pantilt_maxspeed", DEFAULT_pantilt_maxspeed_);
    declare_parameter<double>("settings_update_waitime", DEFAULT_settings_update_waitime_);
    declare_parameter<bool>("speed_locked", DEFAULT_speed_locked_);
    declare_parameter<double>("timeout", DEFAULT_timeout_);
    declare_parameter<std::string>("joy_topic", std::string(DEFAULT_joy_topic_));
    declare_parameter<std::string>("joy_authority_topic", std::string(DEFAULT_joyauthority_topic_));
    declare_parameter<std::string>("vel_cmd_topic", std::string(DEFAULT_velcmd_topic_));
    declare_parameter<std::string>("servo_cmd_topic", std::string(DEFAULT_servocmd_topic_));

    // Pull down configuration parameters
    get_parameter("chassis_max_linspeed", chassis_max_linspeed_);
    get_parameter("chassis_max_angspeed", chassis_max_angspeed_);
    get_parameter("pantilt_maxspeed", pantilt_maxspeed_);
    get_parameter("settings_update_waitime", settings_update_waitime_);
    get_parameter("speed_locked", speed_locked_);
    get_parameter("timeout", timeout_);
    std::string joy_topic, joy_authority_topic, vel_cmd_topic, servo_cmd_topic;
    get_parameter("joy_topic", joy_topic);
    get_parameter("joy_authority_topic", joy_authority_topic);
    get_parameter("vel_cmd_topic", vel_cmd_topic);
    get_parameter("servo_cmd_topic", servo_cmd_topic);

    // Settings Update Time
    last_settings_update_ = get_clock()->now();

    // Joystick Publisher
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
        joy_topic,
        10,
        std::bind(
            &JoyMapping::mappingCallback,
            this,
            std::placeholders::_1));

    // Authority topic shared with Autonomous functionality
    authority_subscription_ = create_subscription<std_msgs::msg::Bool>(
        joy_authority_topic,
        10,
        std::bind(
            &JoyMapping::authorityCallback,
            this,
            std::placeholders::_1));

    // Chassis Subscriber
    chassis_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        vel_cmd_topic,
        10);

    // Pantilt Subscriber
    pantilt_publisher_ = create_publisher<theo_msgs::msg::TheoServoCmd>(
        servo_cmd_topic,
        10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JoyMapping::timerCallback, this));
    last_joy_time_ = get_clock()->now();
};

void JoyMapping::mappingCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (mode_ == TeleopMode::Off)
        return;
    // -> Joystick doesn't have authority (move lower to enable authority override)

    std::vector<float> axes = msg->axes;
    std::vector<int32_t> buttons = msg->buttons;

    // ------------------------------------
    // Assignments
    // ------------------------------------
    // Buttons
    // ------------------------------------
    //  0  -
    //  1  -
    //  2  -
    //  3  -
    //  4  -
    //  5  -
    //  6  - Chassis Bwz-
    //  7  - Chassis Bwz+
    //  8  -
    //  9  - 
    //  10 - 
    //  11 - Chassis Max Speed +
    //  12 - Chassis Max Speed -
    //  13 - Pan-Tilt Max Speed -
    //  14 - Pan-Tilt Max Speed +
    //  15 -
    //  16 -
    //  17 -
    //  18 -
    //  19 -
    //  20 -
    // ------------------------------------
    // Axis
    // ------------------------------------
    //  0  - Chassis Bvx
    //  1  - Chassis Bvy
    //  2  - PT Bvx
    //  3  - PT Bvy
    //  4  - Chassis Bw+
    //  5  - Chassis Bw-
    // ------------------------------------

    geometry_msgs::msg::Twist chassis_msg_out;
    theo_msgs::msg::TheoServoCmd pantilt_msg_out;

    double axes_0 = axes[0];
    double axes_1 = axes[1];
    double axes_2 = axes[2];
    double axes_3 = axes[3];
    /*
        NOTE:   The following variables and their associated conditionals resolve an issue where
                the controller driver reports extreme axes values on connection/disconnection.
                Fortunately, it reports impossible values and can be caught by looking at the
                values on the respective horizontal and vertical axes.
    */
    bool lstick_nconf = (std::abs(axes_0) > stick_conf_thresh_) && (std::abs(axes_1) > stick_conf_thresh_);
    bool rstick_nconf = (std::abs(axes_2) > stick_conf_thresh_) && (std::abs(axes_3) > stick_conf_thresh_);

    // // -> Chassis Movements (Left Stick)
    if (lstick_nconf)
    {
        chassis_msg_out.linear.x = 0.;
        chassis_msg_out.linear.y = 0.;
    }
    else
    {
        chassis_msg_out.linear.x = chassis_max_linspeed_ * axes_1;
        chassis_msg_out.linear.y = chassis_max_linspeed_ * axes_0;
    }

    // // // -> Pantilt Movements (Right Stick)
    if (rstick_nconf)
    {
        pantilt_msg_out.pan_vel = 0.;
        pantilt_msg_out.tilt_vel = 0.;
    }
    else
    {
        pantilt_msg_out.pan_vel = pantilt_maxspeed_ * axes_2;
        pantilt_msg_out.tilt_vel = pantilt_maxspeed_ * axes_3;
    }

    // Triggers for Angular Velocity
    chassis_msg_out.angular.z = chassis_max_angspeed_ * double(axes[4] - axes[5])/2.;

    // // Settings Changes
    if (!speed_locked_)
    {
        double bumper_diff_76 = static_cast<double>(buttons[7] - buttons[6]);
        if (abs(axes[7]) > settings_change_thresh_ || abs(axes[6]) > settings_change_thresh_ || abs(bumper_diff_76) > settings_change_thresh_ )
        {

            rclcpp::Time time = get_clock()->now();

            if ((time - last_settings_update_).seconds() > settings_update_waitime_)
            {

                if (axes[7] > settings_change_thresh_)
                { // -> Max Chassis Speed Increases
                    chassis_max_linspeed_ *= 1.1;
                }
                if (axes[7] < -settings_change_thresh_)
                { // -> Max Chassis Speed Decreases
                    chassis_max_linspeed_ *= 0.9;
                }


                if ( bumper_diff_76 > settings_change_thresh_ ){
                    // -> Max Chassis Angular Speed Decreases
                    chassis_max_angspeed_ *= 1.1;
                }
                if ( bumper_diff_76 < -settings_change_thresh_ ){
                    // -> Max Chassis Angular Speed Decreases
                    chassis_max_angspeed_ *= 0.9;
                }


                if (axes[6] < -settings_change_thresh_)
                { // -> Max Chassis Speed Increases
                    pantilt_maxspeed_ *= 1.1;
                }
                if (axes[6] > settings_change_thresh_)
                { // -> Max Chassis Speed Decreases
                    pantilt_maxspeed_ *= 0.9;
                }
                last_settings_update_ = time;

                RCLCPP_INFO(
                    get_logger(),
                    "CMLS: %.2f, CMAS: %.2f, PTMS: %.2f \r",
                    chassis_max_linspeed_,
                    chassis_max_angspeed_,
                    pantilt_maxspeed_);
            }
        }
    }

    chassis_publisher_->publish(chassis_msg_out);
    pantilt_publisher_->publish(pantilt_msg_out);
    last_joy_time_ = get_clock()->now();
}

void JoyMapping::authorityCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool has_authority = msg->data;
    if (has_authority)
        mode_ = TeleopMode::On;
    else
        mode_ = TeleopMode::Off;
}

void JoyMapping::timerCallback()
{
    if (mode_ == TeleopMode::Off)
        return;

    rclcpp::Time time_now = get_clock()->now();
    double dt = rclcpp::Duration(time_now - last_joy_time_).nanoseconds() * 1e-9;
    if (dt > timeout_)
    {
        geometry_msgs::msg::Twist chassis_msg_out;
        theo_msgs::msg::TheoServoCmd pantilt_msg_out;

        chassis_msg_out.linear.x = 0;
        chassis_msg_out.linear.y = 0;
        chassis_msg_out.angular.z = 0;
        pantilt_msg_out.pan_vel = 0;
        pantilt_msg_out.tilt_vel = 0;

        chassis_publisher_->publish(chassis_msg_out);
        pantilt_publisher_->publish(pantilt_msg_out);
        last_joy_time_ = time_now;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMapping>());
    rclcpp::shutdown();
    return 0;
}
