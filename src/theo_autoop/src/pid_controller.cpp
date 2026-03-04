#include "pid_controller.hpp"


// -------------------------------------------------------------------------------------------------------------
// Constructer / Destructor
// -------------------------------------------------------------------------------------------------------------

PIDControllerNode::PIDControllerNode()
    :   
    BrokerClientNode("pid_controller_node",BroadcastBrokerId::Robotic,"external/broker_exchange","external/broker_robotic_topic"),
    mode_( ControllerMode::Idle )
{

    // setup ros parameters and file handle
    this -> declare_parameter<double>( "p_linear_gain", 0. );
    this -> declare_parameter<double>( "i_linear_gain", 0. );
    this -> declare_parameter<double>( "d_linear_gain", 0. );
    this -> declare_parameter<double>( "p_angular_gain", 0. );
    this -> declare_parameter<double>( "i_angular_gain", 0. );
    this -> declare_parameter<double>( "d_angular_gain", 0. );
    this -> declare_parameter<double>( "p_servo_gain", 0. );
    this -> declare_parameter<double>( "i_servo_gain", 0. );
    this -> declare_parameter<double>( "d_servo_gain", 0. );
    this -> declare_parameter<double>( "proximity_met_range_meters", 0.05 );
    this -> declare_parameter<double>( "proximity_met_direction_degrees", 5 );
    this -> declare_parameter<std::vector<double>>( "rpy_degrees_C0_C", {0.,0.,0.} );

    double proximity_met_direction_degrees;
    std::vector<double> rpy_C0_C;
    this -> get_parameter( "p_linear_gain", this -> p_linear_gain_ );
    this -> get_parameter( "i_linear_gain", this -> i_linear_gain_ );
    this -> get_parameter( "d_linear_gain", this -> d_linear_gain_ );
    this -> get_parameter( "p_angular_gain", this -> p_angular_gain_ );
    this -> get_parameter( "i_angular_gain", this -> i_angular_gain_ );
    this -> get_parameter( "d_angular_gain", this -> d_angular_gain_ );
    this -> get_parameter( "p_servo_gain", this -> p_servo_gain_ );
    this -> get_parameter( "i_servo_gain", this -> i_servo_gain_ );
    this -> get_parameter( "d_servo_gain", this -> d_servo_gain_ );
    this -> get_parameter( "proximity_met_range_meters", this -> proximity_met_range_meters_ );
    this -> get_parameter( "proximity_met_direction_degrees", proximity_met_direction_degrees );
    this -> get_parameter( "rpy_degrees_C0_C", rpy_C0_C );

    double deg2rad_ = (3.14159265358979323846/180.0);
    this -> proximity_met_direction_radians_ = deg2rad_ * proximity_met_direction_degrees;
    this -> R_C0_C_.setRPY( 
        deg2rad_ * rpy_C0_C[0], 
        deg2rad_ * rpy_C0_C[1], 
        deg2rad_ * rpy_C0_C[2] 
    );

    // setup service/publishers/subscribers

    this -> reference_subscription_  = this -> create_subscription<theo_msgs::msg::TheoWaypoint>(
        "external/trajectory_transmission", 
        10, 
        std::bind( 
            &PIDControllerNode::waypointCallback, 
            this, 
            std::placeholders::_1
        )
    );

    this -> vicon_chassis_subscription_  = this -> create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vicon/eowyn/eowyn", 
        10, 
        std::bind( 
            &PIDControllerNode::chassisMocapCallback, 
            this, 
            std::placeholders::_1
        )
    );

    this -> vicon_servo_subscription_  = this -> create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vicon/basler_cam/basler_cam", 
        10, 
        std::bind( 
            &PIDControllerNode::servoMocapCallback, 
            this, 
            std::placeholders::_1
        )
    );

    rclcpp::QoS authority_qos_profile(rclcpp::KeepLast(1));
    authority_qos_profile.transient_local();
    authority_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    this -> teleop_authority_publisher_ = this -> create_publisher<std_msgs::msg::Bool>(
        "joy_authority", 
        authority_qos_profile
    );


    this -> authority_subscription_ = this -> create_subscription<std_msgs::msg::Bool>(
        "auto_authority", 
        10, 
        std::bind( 
            &PIDControllerNode::authorityCallback, 
            this, 
            std::placeholders::_1
        )
    );

    this -> chassis_publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>(
        "velcmd_topic", 
        10
    );

    this -> servo_publisher_ = this -> create_publisher<theo_msgs::msg::TheoServoCmd>(
        "servocmd_topic", 
        10
    );

    this -> last_yaw_ = 0.0;

    return;
}


PIDControllerNode::~PIDControllerNode(){
    this -> reset_idle(); // return authority to proper place :0
};


// -------------------------------------------------------------------------------------------------------------
// Controller Handling/Broadcast
// -------------------------------------------------------------------------------------------------------------

void PIDControllerNode::change_teleop_authority_to( bool mode ){
    std_msgs::msg::Bool msg;
    msg.data = mode;
    this -> teleop_authority_publisher_ -> publish( msg ); // return authority to teleop
}


void PIDControllerNode::switch_mode_( ControllerMode mode_in ){
    switch (mode_in)
    {
    case ControllerMode::Idle:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Controller switching to Idle Mode");
        this -> mode_ = mode_in;
        break;
    
    case ControllerMode::Responding:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Controller switching to Responding Mode");
        this -> mode_ = mode_in;
        break;

    case ControllerMode::Active:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Controller switching to Active Mode");
        this -> mode_ = mode_in;
        break;

    default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "User requested switch to unrecognized mode ... Skipping");
        break;
    }
};


void PIDControllerNode::grab_n_expire_waypoint_data( WaypointData& waypoint_in ){
    waypoint_in = this -> last_waypoint_;
    if( waypoint_in.expired ) return;
    else ( this -> last_waypoint_ ).expired = true;
};


void PIDControllerNode::grab_n_expire_chassis_mocap_data( MocapData& mocap_in ){
    mocap_in = this -> last_chassis_mocap_;
    if( mocap_in.expired ) return;
    else ( this -> last_chassis_mocap_ ).expired = true;
};


void PIDControllerNode::grab_n_expire_servo_mocap_data( MocapData& mocap_in ){
    mocap_in = this -> last_servo_mocap_;
    if( mocap_in.expired ) return;
    else ( this -> last_servo_mocap_ ).expired = true;
};


Eigen::Matrix2d PIDControllerNode::get_last_rotm(){
    double cos_yaw = std::cos( this -> last_yaw_ );
    double sin_yaw = std::sin( this -> last_yaw_ );
    return Eigen::Matrix2d{ 
        {  cos_yaw, sin_yaw },
        { -sin_yaw, cos_yaw } 
    };
};


void PIDControllerNode::process_on_active(){
    
    // copy waypoint and mocap data
    WaypointData current_waypoint; MocapData current_chassis_mocap;
    this->grab_n_expire_waypoint_data( current_waypoint );
    this->grab_n_expire_chassis_mocap_data( current_chassis_mocap );
    if( current_waypoint.expired ) return;

    this -> publish_chassis_control_command( current_waypoint, current_chassis_mocap );
    if( current_waypoint.servo_enabled ){
        MocapData current_servo_mocap; this->grab_n_expire_servo_mocap_data( current_servo_mocap );
        PanTiltData current_pantilt_mocap = get_pantilt_from_mocap( current_chassis_mocap, current_servo_mocap );
        this -> publish_servo_control_command( current_waypoint, current_pantilt_mocap );
    }

    return;

};


bool PIDControllerNode::process_on_responding(){
    
    // copy waypoint and mocap data
    WaypointData current_waypoint; MocapData current_chassis_mocap;
    MocapData current_servo_mocap; PanTiltData current_pantilt_mocap;
    grab_n_expire_waypoint_data( current_waypoint );
    grab_n_expire_chassis_mocap_data( current_chassis_mocap );

    if( !current_waypoint.expired ){
        if( !current_chassis_mocap.expired ){
            this -> publish_chassis_control_command( current_waypoint, current_chassis_mocap );
            if( current_waypoint.servo_enabled ){
                MocapData current_servo_mocap; this->grab_n_expire_servo_mocap_data( current_servo_mocap );
                if( !current_servo_mocap.expired ){
                    current_pantilt_mocap = get_pantilt_from_mocap( current_chassis_mocap, current_servo_mocap );
                    this -> publish_servo_control_command( current_waypoint, current_pantilt_mocap );
                }
                else return false;
            }
        }else return false;
    }else return false;

    return this->check_configuration( current_waypoint, current_chassis_mocap, current_pantilt_mocap );
};


bool PIDControllerNode::check_configuration( 
    WaypointData& current_waypoint, MocapData& current_chassis_mocap, PanTiltData& current_pantilt_mocap
){

    if(
        !(
            ( ( current_waypoint.chassis_position - current_chassis_mocap.position ).segment(0,2) ).norm() 
            < 
            this -> proximity_met_range_meters_ 
        )
    ) return false;

    if(
        !(
            abs( angles::normalize_angle( current_waypoint.chassis_rpy(2) -  current_chassis_mocap.rpy(2) )  )
            <
            this -> proximity_met_direction_radians_
        )
    ) return false;

    
    if( current_waypoint.servo_enabled ){

        if(
            !(
                abs( angles::normalize_angle( current_waypoint.servo_pan_tilt_angle(0) - current_pantilt_mocap.pan_angle )  )
                <
                this -> proximity_met_direction_radians_
            )
        ) return false;

        
        if(
            !(
                abs( angles::normalize_angle( current_waypoint.servo_pan_tilt_angle(1) - current_pantilt_mocap.tilt_angle )  )
                <
                this -> proximity_met_direction_radians_
            )
        ) return false;

    }

    return true;
}


PanTiltData PIDControllerNode::get_pantilt_from_mocap( MocapData& chassis_mocap, MocapData& servo_mocap ){

    PanTiltData ptout;
    if( chassis_mocap.expired || servo_mocap.expired ){
        ptout.expired = true;
        return ptout;
    }

    // Get Rotation Matrices from Mocap System 
    tf2::Matrix3x3 R_V_C; tf2::Matrix3x3 R_V_B;
    R_V_C.setRPY( servo_mocap.rpy(0),   servo_mocap.rpy(1),   servo_mocap.rpy(2)   );
    R_V_B.setRPY( chassis_mocap.rpy(0), chassis_mocap.rpy(1), chassis_mocap.rpy(2) );

    // Get Rotation Matrix for Body ( Chassis) -> Camera Reference Point
    tf2::Matrix3x3 R_C0_B =  (this->R_C0_C_) * R_V_C.transpose() *  R_V_B;
    
    // Extract Pan-Tilt Angles 
    ptout.pan_angle  = std::atan2( R_C0_B[1][0],R_C0_B[0][0] ); 
    ptout.tilt_angle = std::asin( -R_C0_B[2][0] );

    ptout.expired    = false;
    
    return ptout;
}


void  PIDControllerNode::publish_chassis_control_command( WaypointData& waypoint_in, MocapData& mocap_in ){
    // first pull feedforward commands
    Eigen::Vector2d linear_command =  waypoint_in.chassis_linear_velocity.segment(0,2);
    double angular_command         =  waypoint_in.chassis_angular_velocity(2);

    // pull feedback terms if they are available
    if( ! mocap_in.expired ){
        this->last_yaw_ = mocap_in.rpy(2);
        linear_command += 
                ( 
                    this-> p_linear_gain_ * 
                            ( waypoint_in.chassis_position.segment(0,2) - mocap_in.position.segment(0,2) ) 
                );
        
        angular_command += 
                ( 
                    this-> p_angular_gain_ * 
                    angles::normalize_angle( waypoint_in.chassis_rpy(2) -  mocap_in.rpy(2) ) 
                );
         
    } 

    Eigen::Vector2d linear_command_out = this -> get_last_rotm() * linear_command;


    // package and output
    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x  = linear_command_out(0);
    msg_out.linear.y  = linear_command_out(1);
    msg_out.angular.z = angular_command;
    this -> chassis_publisher_ -> publish( msg_out );

    return;
};


void  PIDControllerNode::publish_servo_control_command( WaypointData& waypoint_in, PanTiltData& pantilt_in ){
    // first pull feedforward commands
    Eigen::Vector2d command =  waypoint_in.servo_pan_tilt_velocity;

    // pull feedback terms if they are available
    if( ! pantilt_in.expired ){
        command(0) += 
            ( 
                this-> p_servo_gain_ * 
                        ( waypoint_in.servo_pan_tilt_angle(0) - pantilt_in.pan_angle ) 
            );
        command(1) += 
            ( 
                this-> p_servo_gain_ * 
                        ( waypoint_in.servo_pan_tilt_angle(1) - pantilt_in.tilt_angle ) 
            );
    } 

    // package and output
    theo_msgs::msg::TheoServoCmd msg_out;
    msg_out.pan_vel  = command(0);
    msg_out.tilt_vel = command(1);
    this -> servo_publisher_ -> publish( msg_out );

    return;
};


void PIDControllerNode::reset_idle(){
    this -> switch_mode_( ControllerMode::Idle );
    this -> change_teleop_authority_to( true );
}


void PIDControllerNode::authorityCallback( const std_msgs::msg::Bool::SharedPtr msg ){
    bool has_authority = msg -> data;
    if( !has_authority ){ 
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Autonomous Controller has Lost Authoity. Returning to Idle.");
        this -> reset_idle();
    }
}


void PIDControllerNode::waypointCallback( const theo_msgs::msg::TheoWaypoint::SharedPtr msg_ptr ){
    WaypointData new_data;
    
    new_data.receive_time   = this -> get_clock() -> now().seconds();

    new_data.chassis_position(0)    = msg_ptr -> chassis_state.pose.position.x;
    new_data.chassis_position(1)    = msg_ptr -> chassis_state.pose.position.y;
    new_data.chassis_position(2)    = msg_ptr -> chassis_state.pose.position.z;
    
    tf2::Quaternion tmp_quat(
        msg_ptr -> chassis_state.pose.orientation.x,
        msg_ptr -> chassis_state.pose.orientation.y,
        msg_ptr -> chassis_state.pose.orientation.z,
        msg_ptr -> chassis_state.pose.orientation.w
    );
    tf2::Matrix3x3 tmp_mat(tmp_quat);
    double roll, pitch, yaw;
    tmp_mat.getRPY( roll, pitch, yaw );
    new_data.chassis_rpy(0) = roll;
    new_data.chassis_rpy(1) = pitch;
    new_data.chassis_rpy(2) = yaw;

    new_data.chassis_linear_velocity(0) = msg_ptr -> chassis_state.twist.linear.x;
    new_data.chassis_linear_velocity(1) = msg_ptr -> chassis_state.twist.linear.y;
    new_data.chassis_linear_velocity(2) = msg_ptr -> chassis_state.twist.linear.z;

    new_data.chassis_angular_velocity(0) = msg_ptr -> chassis_state.twist.angular.x;
    new_data.chassis_angular_velocity(1) = msg_ptr -> chassis_state.twist.angular.y;
    new_data.chassis_angular_velocity(2) = msg_ptr -> chassis_state.twist.angular.z;

    // if the servo is enabled, indicate it in the data
    if( msg_ptr -> servo_enabled ){
        new_data.servo_enabled = true;
        new_data.servo_pan_tilt_angle(0)    = msg_ptr -> servo_state.angle.pan_angle;
        new_data.servo_pan_tilt_angle(1)    = msg_ptr -> servo_state.angle.tilt_angle;
        new_data.servo_pan_tilt_velocity(0) = msg_ptr -> servo_state.vel.pan_vel;
        new_data.servo_pan_tilt_velocity(1) = msg_ptr -> servo_state.vel.tilt_vel;
    }
    else new_data.servo_enabled = false;

    new_data.expired = false;
    this -> last_waypoint_ = new_data;
};


void PIDControllerNode::chassisMocapCallback( const geometry_msgs::msg::PoseStamped::SharedPtr  msg_ptr ){
    MocapData new_data;

    new_data.receive_time   = this -> get_clock() -> now().seconds();

    new_data.position(0)    = msg_ptr -> pose.position.x;
    new_data.position(1)    = msg_ptr -> pose.position.y;
    new_data.position(2)    = msg_ptr -> pose.position.z;

    tf2::Quaternion tmp_quat(
        msg_ptr -> pose.orientation.x,
        msg_ptr -> pose.orientation.y,
        msg_ptr -> pose.orientation.z,
        msg_ptr -> pose.orientation.w
    );
    tf2::Matrix3x3 tmp_mat(tmp_quat);
    double roll, pitch, yaw;
    tmp_mat.getRPY( roll, pitch, yaw );
    new_data.rpy(0) = roll;
    new_data.rpy(1) = pitch;
    new_data.rpy(2) = yaw;

    new_data.expired = false;
    
    this -> last_chassis_mocap_ = new_data;
};


void PIDControllerNode::servoMocapCallback( const geometry_msgs::msg::PoseStamped::SharedPtr  msg_ptr ){
    MocapData new_data;

    new_data.receive_time   = this -> get_clock() -> now().seconds();

    new_data.position(0)    = msg_ptr -> pose.position.x;
    new_data.position(1)    = msg_ptr -> pose.position.y;
    new_data.position(2)    = msg_ptr -> pose.position.z;

    tf2::Quaternion tmp_quat(
        msg_ptr -> pose.orientation.x,
        msg_ptr -> pose.orientation.y,
        msg_ptr -> pose.orientation.z,
        msg_ptr -> pose.orientation.w
    );
    tf2::Matrix3x3 tmp_mat(tmp_quat);
    double roll, pitch, yaw;
    tmp_mat.getRPY( roll, pitch, yaw );
    new_data.rpy(0) = roll;
    new_data.rpy(1) = pitch;
    new_data.rpy(2) = yaw;

    new_data.expired = false;
    
    this -> last_servo_mocap_ = new_data;
};

// -------------------------------------------------------------------------------------------------------------
// Broker Client Overrides
// -------------------------------------------------------------------------------------------------------------


void PIDControllerNode::codeCallback(
    const theo_msgs::msg::TheoCode::SharedPtr msg
)
{
    switch (  static_cast<BroadcastBrokerStatus>(msg -> code) )
    {
        case BroadcastBrokerStatus::Standby_After_Received :
        {
            // switch mode and confirm
            // later confirm and send confirmation
            this -> change_teleop_authority_to( false );
            this -> switch_mode_( ControllerMode::Responding );
            this -> send_broker_request_( BroadcastBrokerCodes::Confirm );
            break;
        }

        case BroadcastBrokerStatus::Broadcasting_Trajectory :
        {
            // switch mode / no need to confirm
            this -> change_teleop_authority_to( false );
            this -> switch_mode_( ControllerMode::Active );
            break;
        }
        
        default:
        {
            this -> reset_idle();
            break;
        }
    }
};


void PIDControllerNode::timerCallback(){

    if( this->awaiting_broker_response_ ) return;
    // double current_time_seconds = this -> get_clock() -> now().seconds();

    switch ( this-> mode_ )
    {

        case ControllerMode::Idle :
        {
            break;
        }

        case ControllerMode::Responding :
        {
            if( this -> process_on_responding() ){ 
                if( this -> broker_status_ == BroadcastBrokerStatus::Broadcasting_Configuration  ){
                    this -> send_broker_request_( BroadcastBrokerCodes::Confirm );
                }
                this -> reset_idle();
            };
            break;
        }

        case ControllerMode::Active :
        {   
            process_on_active();
            break;
        }
        
        default:
            break;

    }
};



// -------------------------------------------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------------------------------------------

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PIDControllerNode> node = std::make_shared<PIDControllerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}