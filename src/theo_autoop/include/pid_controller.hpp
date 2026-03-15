#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <angles/angles.h>
#include "controller_modes.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "theo_msgs/msg/theo_waypoint.hpp"
#include "theo_msgs/msg/theo_servo_cmd.hpp"
#include "theo_comm/broker_client_node.hpp"

struct MocapData{
    double receive_time;
    Eigen::Vector3d position;
    Eigen::Vector3d rpy;
    bool expired = true;
};

struct PanTiltData{
    double receive_time;
    double pan_angle;
    double tilt_angle;
    bool expired = true;
};


struct WaypointData{
    double receive_time;
    Eigen::Vector3d chassis_position;
    Eigen::Vector3d chassis_rpy;
    Eigen::Vector3d chassis_linear_velocity;
    Eigen::Vector3d chassis_angular_velocity;
    Eigen::Vector2d servo_pan_tilt_angle;
    Eigen::Vector2d servo_pan_tilt_velocity;
    bool servo_enabled = false;
    bool expired = true;
};


struct Error3D{
    double receive_time;
    Eigen::Vector3d p_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d i_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d d_error = Eigen::Vector3d::Zero();
};


struct Error2D{
    double receive_time;
    Eigen::Vector2d p_error = Eigen::Vector2d::Zero();
    Eigen::Vector2d i_error = Eigen::Vector2d::Zero();
    Eigen::Vector2d d_error = Eigen::Vector2d::Zero();
};


struct Error1D{
    double p_error = 0;
    double i_error = 0;
    double d_error = 0;
};


class PIDControllerNode : public BrokerClientNode {

    public:
        PIDControllerNode();
        ~PIDControllerNode();

    private:

        // Controller Handling | // Control Broadcast Parameters
        double p_linear_gain_;
        double i_linear_gain_; 
        double d_linear_gain_; 
        double p_angular_gain_;
        double i_angular_gain_; 
        double d_angular_gain_;
        double p_servo_gain_;
        double i_servo_gain_;
        double d_servo_gain_;
        double proximity_met_range_meters_;
        double proximity_met_direction_radians_;
        double chassis_linear_error_integrator_threshold_; 
        double chassis_angular_error_integrator_threshold_; 
        double servo_error_integrator_threshold_; 
        double valid_error_update_dt_; 
        double last_yaw_;
        Eigen::Vector3d chassis_error_iterm_;
        Eigen::Vector2d servo_error_iterm_;
        Error3D last_chassis_errors_;
        Error2D last_servo_errors_;
        ControllerMode mode_;
        WaypointData last_waypoint_;
        MocapData last_chassis_mocap_;
        MocapData last_servo_mocap_;
        tf2::Matrix3x3 R_C0_C_; // Aligns camera reference configuration with chassis
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_publisher_;
        rclcpp::Publisher<theo_msgs::msg::TheoServoCmd>::SharedPtr servo_publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr authority_subscription_;
        rclcpp::Subscription<theo_msgs::msg::TheoWaypoint>::SharedPtr reference_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_chassis_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_servo_subscription_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teleop_authority_publisher_;
        void change_teleop_authority_to( bool );
        void switch_mode_( ControllerMode );
        void grab_n_expire_waypoint_data( WaypointData& );
        void grab_n_expire_chassis_mocap_data( MocapData& );
        void grab_n_expire_servo_mocap_data( MocapData& );
        Eigen::Matrix2d get_last_rotm();
        void reset_signal_errors_();
        void update_chassis_errors_( Error3D );
        void update_servo_errors_( Error2D );
        void unwind_accumulated_errors();
        Eigen::Vector3d add_n_grab_chassis_error_i_term_( Eigen::Vector3d&, double& );
        Eigen::Vector2d add_n_grab_servo_error_i_term_( Eigen::Vector2d&, double& );
        Error3D grab_n_update_chassis_errors( WaypointData&, MocapData& );
        Error2D grab_n_update_servo_errors( WaypointData&, PanTiltData& );
        void process_on_active();
        bool process_on_responding();
        bool check_configuration( WaypointData&, MocapData&, PanTiltData& );
        PanTiltData get_pantilt_from_mocap( MocapData&, MocapData& );
        void publish_chassis_control_command( WaypointData&, MocapData& );
        void publish_servo_control_command( WaypointData&, PanTiltData& );
        void reset_idle();
        void authorityCallback( const std_msgs::msg::Bool::SharedPtr );
        void waypointCallback( const theo_msgs::msg::TheoWaypoint::SharedPtr );
        void chassisMocapCallback( const geometry_msgs::msg::PoseStamped::SharedPtr );
        void servoMocapCallback( const geometry_msgs::msg::PoseStamped::SharedPtr );

        // // Broker Communications Overrides
        void codeCallback( const theo_msgs::msg::TheoCode::SharedPtr ) override;
        void timerCallback() override;
};


