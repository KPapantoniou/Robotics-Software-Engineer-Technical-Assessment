#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <thread>
#include <vector>
#include <chrono>



using namespace std::chrono;

class UR20Display : public rclcpp::Node
{
    public:
        UR20Display() : Node("ur20_display_node"){
            // Constructor: initializes ROS node, TF buffer/listener, publishers, and RViz tools for robot visualization

            RCLCPP_INFO(this->get_logger(), "UR20 Display Node has been started.");
            this->declare_parameter("joint_config", std::vector<double>{} );
            joint_config_ = this->get_parameter("joint_config").as_double_array();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            initial_timer_ = this->create_wall_timer(100ms, std::bind(&UR20Display::initial, this));

            visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers",this));
            visual_tools_->loadMarkerPub();
            visual_tools_->deleteAllMarkers();

            traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory", 10);

        }

        ~UR20Display(){
            if(trajectory_thread_.joinable()){
                trajectory_thread_.join();
            }
        }
    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        rclcpp::TimerBase::SharedPtr timer_, initial_timer_;
        std::vector<double> joint_config_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        Eigen::Isometry3d T_elbow_gripper;
        Eigen::Isometry3d T_world_elbow;
        Eigen::Isometry3d T_world_gripper;
        bool transforms_received_ = false;

        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

        double t_period = 4.0;
        double total_time_ = 1.5 * t_period;
        std::vector<double> q0_;
        std::vector<double> qf_;
        double omega_ =2 * M_PI / t_period;
        std::vector<double> q_; 
        double t_ = 0.0; 
        double dt_ = 0.033; 
        sensor_msgs::msg::JointState joint_state;

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
        std::thread trajectory_thread_; 

        void initial(){
            initial_timer_->cancel();
            joint_state.name={"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            joint_state.position = joint_config_;
            for (int i = 0; i < 20; i++) {
                joint_state.header.stamp = this->now();
                joint_pub_->publish(joint_state);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            q0_ = joint_config_;
            q_.resize(q0_.size());
            qf_.resize(q0_.size());
            random_goal(qf_);
            timer_ = this->create_wall_timer(33ms, std::bind(&UR20Display::tf_timer, this));
            trajectory_thread_ = std::thread(&UR20Display::run_trajectory, this);
        }
        
        void tf_timer(){
            // Periodically queries TF tree and publishes robot frame visualization in RViz
            
            std::string gripper_link = "gripper_link";
            std::string elbow_link = "forearm_link";
            std::string world_link = "world";

            geometry_msgs::msg::TransformStamped Tf_elbow_gripper;
            geometry_msgs::msg::TransformStamped Tf_world_elbow;
            geometry_msgs::msg::TransformStamped Tf_world_gripper;

            try{
                Tf_elbow_gripper = tf_buffer_ -> lookupTransform(
                    elbow_link,
                    gripper_link,
                    tf2::TimePointZero
                );

                Tf_world_elbow = tf_buffer_ -> lookupTransform(
                    world_link,
                    elbow_link,
                    tf2::TimePointZero
                );

                Tf_world_gripper = tf_buffer_ -> lookupTransform(
                    world_link,
                    gripper_link,                    
                    tf2::TimePointZero
                );

            } catch (const tf2::TransformException &ex){
                RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", gripper_link.c_str(), world_link.c_str(), ex.what());
                return;
            }
            T_elbow_gripper = transformToEigen(Tf_elbow_gripper);
            T_world_elbow = transformToEigen(Tf_world_elbow);
            T_world_gripper = transformToEigen(Tf_world_gripper);

            // Verification: chaining transforms proves kinematic consistency of the TF tree
            if (!transforms_received_){
                Eigen::Isometry3d verification = T_world_elbow * T_elbow_gripper;
                RCLCPP_INFO(this->get_logger(), "Successfully received transforms. Displaying them below.");
                RCLCPP_INFO(this->get_logger(), "T_world_gripper:\n%s", matrix_to_string(T_world_gripper).c_str());
                RCLCPP_INFO(this->get_logger(), "T_world_elbow * T_elbow_gripper:\n%s", matrix_to_string(verification).c_str());    
                transforms_received_ = true;                
            }
            visual_tools_->deleteAllMarkers();
            visual_tools_->setAlpha(1.0);
            visual_tools_ ->publishAxisLabeled(T_world_gripper, "Tf_world_gripper",  rviz_visual_tools::XXLARGE);
            visual_tools_ ->publishText(T_elbow_gripper, "Tf_elbow_gripper", rviz_visual_tools::WHITE, rviz_visual_tools::XXLARGE);
            visual_tools_ ->trigger();

        }

        void run_trajectory(){
            // visual_tools_->deleteAllMarkers();
            while(rclcpp::ok()){

                int num_points = static_cast<int>(total_time_ / dt_);
                trajectory_msgs::msg::JointTrajectory traj_msg;
                traj_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", 
                                        "elbow_joint", "wrist_1_joint", 
                                        "wrist_2_joint", "wrist_3_joint"};


                
                for(int i = 0; i < num_points; i++){
                    double t = i * dt_;
                    std::vector<double> q(6);

                    // Sinusoidal interpolation: smooth trajectory using cosine-based blending between q0 and qf
                    for(size_t j = 0; j < 6; j++){
                        q[j] = q0_[j] + (qf_[j] - q0_[j]) * 0.5 * (1 - cos(omega_ * t));
                    }

                    sensor_msgs::msg::JointState js;
                    js.name = {"shoulder_pan_joint", "shoulder_lift_joint",
                            "elbow_joint", "wrist_1_joint",
                            "wrist_2_joint", "wrist_3_joint"};
                    js.position = q;
                    js.header.stamp = this->now();
                    joint_pub_->publish(js);

                    trajectory_msgs::msg::JointTrajectoryPoint point;
                    point.positions = q;
                    point.time_from_start = rclcpp::Duration::from_seconds(t);
                    traj_msg.points.push_back(point);

                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
            
                }

                traj_pub_->publish(traj_msg);
                RCLCPP_INFO(this->get_logger(), "Trajectory published!");

                q0_ = joint_config_;
                random_goal(qf_);
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        }

        Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::TransformStamped &t){
            // Converts ROS TransformStamped into Eigen Isometry for matrix-based kinematic computation
            geometry_msgs::msg::Transform transform = t.transform;
            Eigen::Quaterniond quad(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
            return Eigen::Translation3d(transform.translation.x, transform.translation.y, transform.translation.z) * quad;
        }

        std::string matrix_to_string(const Eigen::Isometry3d& T) {
            std::stringstream ss;
            Eigen::IOFormat MatFmt(4, 0, ", ", "\n", "[", "]");
            ss << T.matrix().format(MatFmt);
            return ss.str();
        }

        void random_goal(std::vector<double>& qf){
            qf.resize(q0_.size());
            for (size_t i = 0; i < 6; ++i){
                qf[i] = ((double) rand() / RAND_MAX) * 2 * M_PI - M_PI;
            }
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR20Display>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}