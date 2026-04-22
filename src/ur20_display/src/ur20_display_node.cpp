#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <rviz_visual_tools/rviz_visual_tools.hpp>



using namespace std::chrono;

class UR20Display : public rclcpp::Node
{
    public:
        UR20Display() : Node("ur20_display_node"){

            RCLCPP_INFO(this->get_logger(), "UR20 Display Node has been started.");
            this->declare_parameter("joint_config", std::vector<double>{} );
            joint_config_ = this->get_parameter("joint_config").as_double_array();
            // RCLCPP_INFO(this->get_logger(), "Joint config size: %zu", joint_config_.size());

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            timer_ = this->create_wall_timer(33ms, std::bind(&UR20Display::on_timer, this));

            visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers",this));
            visual_tools_->loadMarkerPub();
            visual_tools_->deleteAllMarkers();
          

        }
    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<double> joint_config_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        Eigen::Isometry3d T_elbow_gripper;
        Eigen::Isometry3d T_world_elbow;
        Eigen::Isometry3d T_world_gripper;
        bool transforms_received_ = false;

        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;


        void on_timer(){
            sensor_msgs::msg::JointState joint_state;
            joint_state.name={"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            joint_state.position = joint_config_;
            joint_state.header.stamp = this->now();

            joint_pub_ ->publish(joint_state);
            
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

            

            if (!transforms_received_){
                Eigen::Isometry3d verification = T_world_elbow * T_elbow_gripper;
                RCLCPP_INFO(this->get_logger(), "Successfully received transforms. Displaying them below.");
                RCLCPP_INFO(this->get_logger(), "T_world_gripper:\n%s", matrix_to_string(T_world_gripper).c_str());
                RCLCPP_INFO(this->get_logger(), "T_world_elbow * T_elbow_gripper:\n%s", matrix_to_string(verification).c_str());    
                transforms_received_ = true;                
            }
            visual_tools_->setAlpha(1.0);
            visual_tools_ ->publishAxisLabeled(T_world_gripper, "Tf_world_gripper",  rviz_visual_tools::XXLARGE);
            visual_tools_ ->publishText(T_elbow_gripper, "Tf_elbow_gripper", rviz_visual_tools::WHITE, rviz_visual_tools::XXLARGE);
            visual_tools_ ->trigger();
            
        }

        Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::TransformStamped &t){
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
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR20Display>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}