#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp> // <-- ADD THIS INCLUDE
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// --- NEW INCLUDES FOR TF2 MATH ---
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#include <chrono>
#include <algorithm> // Required for std::clamp
#include <deque>

using namespace std::chrono_literals;

class VisualServoNode : public rclcpp::Node {
public:
    VisualServoNode() : Node("visual_servo_node"), servo_active_(false) {
        
        // --- NEW: Setup TF2 Listener ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 1. Subscriber: Listens to the Python ArUco Tracker
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/plate_target_pose", 10,
            std::bind(&VisualServoNode::pose_callback, this, std::placeholders::_1));

        // 2. Publisher: Sends velocity commands to MoveIt Servo
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        servo_switch_client_ = this->create_client<moveit_msgs::srv::ServoCommandType>(
            "/servo_node/switch_command_type");

        // 3. Service: A safe terminal trigger to turn servoing ON/OFF
        trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/toggle_visual_servoing",
            std::bind(&VisualServoNode::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 4. Control Loop Timer: Runs at 30Hz to feed MoveIt Servo continuously
        // control_loop_timer_ = this->create_wall_timer(
        //     33ms, std::bind(&VisualServoNode::control_loop, this));
        control_loop_timer_ = this->create_timer(
            200ms, std::bind(&VisualServoNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Visual Servo Node initialized. Waiting for trigger...");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Add new pose to buffer and calculate the average pose for smoothing
        pose_buffer_.push_back(msg->pose);
        if (pose_buffer_.size() > buffer_size_) {
            pose_buffer_.pop_front(); // Remove oldest frame
        }

        target_pose_.header = msg->header;
        target_pose_.pose = calculate_average_pose(pose_buffer_);
    }

    geometry_msgs::msg::Pose calculate_average_pose(const std::deque<geometry_msgs::msg::Pose>& buffer) {
        geometry_msgs::msg::Pose avg;
        double sum_x = 0, sum_y = 0, sum_z = 0;
        double sum_qx = 0, sum_qy = 0, sum_qz = 0, sum_qw = 0;

        auto first_q = buffer.front().orientation;

        for (const auto& p : buffer) {
            sum_x += p.position.x;
            sum_y += p.position.y;
            sum_z += p.position.z;
            
            // Hemisphere Check to safely average quaternions
            double dot_product = p.orientation.x * first_q.x + p.orientation.y * first_q.y + 
                                 p.orientation.z * first_q.z + p.orientation.w * first_q.w;
            
            if (dot_product < 0.0) {
                sum_qx -= p.orientation.x; sum_qy -= p.orientation.y;
                sum_qz -= p.orientation.z; sum_qw -= p.orientation.w;
            } else {
                sum_qx += p.orientation.x; sum_qy += p.orientation.y;
                sum_qz += p.orientation.z; sum_qw += p.orientation.w;
            }
        }
        
        double N = buffer.size();
        avg.position.x = sum_x / N; avg.position.y = sum_y / N; avg.position.z = sum_z / N;
        double norm = std::sqrt(sum_qx*sum_qx + sum_qy*sum_qy + sum_qz*sum_qz + sum_qw*sum_qw);
        avg.orientation.x = sum_qx / norm; avg.orientation.y = sum_qy / norm;
        avg.orientation.z = sum_qz / norm; avg.orientation.w = sum_qw / norm;
        return avg;
    }

    void trigger_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        // Ignore the request data, just flip the boolean switch
        (void)request; 
        if (!servo_active_) {
            // We are turning ON. Tell MoveIt Servo to expect TWIST commands.
            if (servo_switch_client_->wait_for_service(1s)) {
                auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
                req->command_type = moveit_msgs::srv::ServoCommandType::Request::TWIST;
                
                // Fire and forget so we don't deadlock the callback
                servo_switch_client_->async_send_request(req);
                RCLCPP_INFO(this->get_logger(), "Switched MoveIt Servo to TWIST command mode.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Servo switch service not found! Is MoveIt Servo running?");
            }
        }

        servo_active_ = !servo_active_;
        
        response->success = true;
        response->message = servo_active_ ? "Visual Servoing ENABLED" : "Visual Servoing DISABLED";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void control_loop() {
        // If the switch is off, do absolutely nothing.
        if (!servo_active_ || pose_buffer_.empty()) return;

        // --- 1. CONVERT CAMERA TARGET TO TF2 FORMAT ---
        tf2::Transform tf_cam_to_aruco;
        tf2::fromMsg(target_pose_.pose, tf_cam_to_aruco);

        // --- 2. DEFINE THE HOVER OFFSET (Relative to the ArUco marker) ---
        tf2::Transform tf_aruco_to_hover;
        
        // A. Translation: 15cm back along the long axis, 1cm down to grab the sides
        tf_aruco_to_hover.setOrigin(tf2::Vector3(-0.3, 0.0, 0.30));

        // B. Rotation Matrix: Aligning the TCP axes to the ArUco axes
        tf2::Matrix3x3 rot_matrix(
            0,  0,  1,  // ArUco X axis points along TCP Z
            0, -1,  0,  // ArUco Y axis points along TCP -Y
            1,  0,  0   // ArUco Z axis points along TCP X
        );
        // tf2::Matrix3x3 rot_matrix(
        //     0,  1,  0,  // ArUco X axis points along TCP Y
        //     0, 0,  1,  // ArUco Y axis points along TCP Z
        //     1,  0,  0   // ArUco Z axis points along TCP X
        // );
        tf2::Quaternion q_offset;
        rot_matrix.getRotation(q_offset);
        tf_aruco_to_hover.setRotation(q_offset);

        // --- 3. CALCULATE TRUE HOVER TARGET IN CAMERA FRAME ---
        tf2::Transform tf_cam_to_hover = tf_cam_to_aruco * tf_aruco_to_hover;

        geometry_msgs::msg::PoseStamped hover_pose_cam;
        hover_pose_cam.header.frame_id = "camera_link_optical";
        hover_pose_cam.header.stamp = target_pose_.header.stamp;
        tf2::toMsg(tf_cam_to_hover, hover_pose_cam.pose);

        // --- 4. TRANSFORM HOVER TARGET INTO TCP PERSPECTIVE ---
        geometry_msgs::msg::PoseStamped tcp_target_pose;
        try {
            tcp_target_pose = tf_buffer_->transform(hover_pose_cam, "tcp_link", tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Error: %s", ex.what());
            return;
        }

        // --- 5. THE BEAUTIFULLY SIMPLE ERROR MATH ---
        // Because tcp_target_pose is exactly where the TCP *should* be,
        // we just drive all coordinates directly to 0!
        
        // 1. Controller Parameters
        const double Kp_trans = 1.0; 
        const double Kp_ang = 0.8;  
        const double max_trans_vel = 0.2; 
        const double max_ang_vel = 0.2;

        // DEADBAND TOLERANCES
        const double trans_tolerance = 0.005; // 5 mm
        const double ang_tolerance = 0.05;    // ~2.8 degrees

        // 2. Calculate Cartesian Errors
        // Because the camera is the origin (0,0,0), the target coordinates ARE the error.
        double error_x = tcp_target_pose.pose.position.x;
        double error_y = tcp_target_pose.pose.position.y;
        double error_z = tcp_target_pose.pose.position.z;

        // 3. Angular Error
        tf2::Quaternion q_error(
            tcp_target_pose.pose.orientation.x,
            tcp_target_pose.pose.orientation.y,
            tcp_target_pose.pose.orientation.z,
            tcp_target_pose.pose.orientation.w);

        // --- THE FIX: The 180-Degree Flip ---
        // The plate's Z points UP. The TCP's Z points DOWN.
        // We rotate the target 180 degrees (PI radians) around the X-axis to flip it over.
        // tf2::Quaternion q_grasp_offset;
        
        // Note: If your gripper fingers end up perpendicular to the plate instead of parallel,
        // you can easily fix it by adding a Yaw offset here! 
        // Example: q_grasp_offset.setRPY(M_PI, 0, M_PI/2);
        // q_grasp_offset.setRPY(M_PI, 0, M_PI/2); // Roll 180 degrees to flip Z axis

        // Multiply the quaternions to calculate the true angular error
        // tf2::Quaternion q_error = q_plate_in_tcp * q_grasp_offset;
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_error).getRPY(roll, pitch, yaw);

        // --- NEW: Apply Deadband (Stop conditions) ---
        if (std::abs(error_x) < trans_tolerance) error_x = 0.0;
        if (std::abs(error_y) < trans_tolerance) error_y = 0.0;
        if (std::abs(error_z) < trans_tolerance) error_z = 0.0;
        if (std::abs(roll)  < ang_tolerance)   roll = 0.0;
        if (std::abs(pitch) < ang_tolerance)  pitch = 0.0;
        if (std::abs(yaw)   < ang_tolerance)    yaw = 0.0;

        // 4. Calculate Velocities
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->get_clock()->now();
        twist_msg.header.frame_id = "tcp_link"; // Safe to use tcp_link now!

        twist_msg.twist.linear.x = std::clamp(Kp_trans * error_x, -max_trans_vel, max_trans_vel);
        twist_msg.twist.linear.y = std::clamp(Kp_trans * error_y, -max_trans_vel, max_trans_vel);
        twist_msg.twist.linear.z = std::clamp(Kp_trans * error_z, -max_trans_vel, max_trans_vel);

        twist_msg.twist.angular.x = std::clamp(Kp_ang * roll,  -max_ang_vel, max_ang_vel);
        twist_msg.twist.angular.y = std::clamp(Kp_ang * pitch, -max_ang_vel, max_ang_vel);
        twist_msg.twist.angular.z = std::clamp(Kp_ang * yaw,   -max_ang_vel, max_ang_vel);

        // 5. Publish
        twist_pub_->publish(twist_msg);

        // Feedback Logic
        if (error_x == 0 && error_y == 0 && error_z == 0 && roll == 0 && pitch == 0 && yaw == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "HOVER POSITION REACHED. Ready to slide in.");
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Servoing | Dist to Hover: Z=%.3fm, X=%.3fm", error_z, error_x);
        }
    }

    bool servo_active_;
    geometry_msgs::msg::PoseStamped target_pose_;
    std::deque<geometry_msgs::msg::Pose> pose_buffer_;
    const size_t buffer_size_ = 10; // Averages the last 10 frames (~0.33 seconds of data)
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr servo_switch_client_; // <-- ADDED CLIENT
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualServoNode>());
    rclcpp::shutdown();
    return 0;
}