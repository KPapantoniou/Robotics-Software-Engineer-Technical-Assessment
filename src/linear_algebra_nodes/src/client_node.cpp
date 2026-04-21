#include <yaml-cpp/yaml.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_service/srv/least_squares.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <eigen3/Eigen/Geometry>

using namespace std;
// ClientNode class definition this node will send a initial request 
class ClientNode : public rclcpp::Node{
    public:
    ClientNode(): Node("client_node"){
        using namespace std::placeholders;
        this->client_ptr_ = this->create_client<linear_algebra_service::srv::LeastSquares>("LeastSquares");
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("transformed_vector", 10);
    }

    // Function to send a request to the server node
    void send_request(){
        vector<vector<double>> A;
        vector<double> b;
        // Create a request object as a shared pointer to be send
        auto request = std::make_shared<linear_algebra_service::srv::LeastSquares::Request>();
        
        load_yaml("/ros2_ws/src/data.yaml", A, b);
        
        for(const auto &row: A){
            geometry_msgs::msg::Vector3 vec;
            vec.x = row[0];
            vec.y = row[1];
            vec.z = row[2];
            request->matrix_a.push_back(vec);
        }
        request->vector_b.x = b[0];
        request->vector_b.y = b[1];
        request->vector_b.z = b[2];
        
        while (!this->client_ptr_->wait_for_service(std::chrono::seconds(1))){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(rclcpp::get_logger("client_node"), "Interrupted while waiting for the service. Exiting.");
                return ;
            }
            ros_log("[client] Service not available, waiting again...");
        }
        // Send the request and wait for the response
        auto result = this->client_ptr_ -> async_send_request(request);
        //shared_from_this() is used to get a shared pointer to the current instance of the node and remain alive
        if(rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS){
                auto response = result.get();
                ros_log("[client] Received response from server:");
                ros_log("[client] Transformed vector x: [" + to_string(response->x.x) + ", " + to_string(response->x.y) + ", " + to_string(response->x.z) + "]");
                ros_log("[client] Translation vector: [" + to_string(response->transformation.translation.x) + ", " + to_string(response->transformation.translation.y) + ", " + to_string(response->transformation.translation.z) + "]");
                ros_log("[client] Rotation quaternion: [" + to_string(response->transformation.rotation.x) + ", " + to_string(response->transformation.rotation.y) + ", " + to_string(response->transformation.rotation.z) + ", " + to_string(response->transformation.rotation.w) + "]");
                
                Eigen::Vector3d x_recovered = invert_model(response);
                
                ros_log("[client] x vector: [" + to_string(x_recovered.x()) + ", " + to_string(x_recovered.y()) + ", " + to_string(x_recovered.z()) + "]");
                auto message = geometry_msgs::msg::Vector3();
                message.x = x_recovered.x();
                message.y = x_recovered.y();
                message.z = x_recovered.z();
                publisher_->publish(message);


            }else{
                RCLCPP_ERROR(rclcpp::get_logger("client_node"), "Failed to call service LeastSquares");
            }

    }
    private:
        rclcpp::Client<linear_algebra_service::srv::LeastSquares>::SharedPtr client_ptr_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
        // Function to load the YAML file and create A, b from it
        void load_yaml(const string &file_path, vector<vector<double>> &A, vector<double> &b){
            YAML::Node config = YAML::LoadFile(file_path);
            A = config["matrix_a"].as<vector<vector<double>>>();
            b = config["vector_b"].as<vector<double>>();
        }

        // Function to log messages to the console
        void ros_log(const string &message){
            RCLCPP_INFO(rclcpp::get_logger("client_node"), message.c_str());
        }
        // Function to print a vector to the console
        void print_vector(const vector<double> &vec){
            for(const auto& value: vec){
                cout << value << " ";
            }
            cout << endl;
        }
        // Function to print a matrix to the console
        void print_matrix(const vector<vector<double>> &mat){
            for(const auto& row: mat){
                for(const auto& value: row){
                    cout << value << " ";
                }
                cout << endl;
            }
        }
        
        Eigen::Vector3d invert_model(const std::shared_ptr<linear_algebra_service::srv::LeastSquares::Response> response){
            Eigen::Vector3d x_new(response->x.x, response->x.y, response->x.z);
            Eigen::Vector3d t(response->transformation.translation.x, response->transformation.translation.y, response->transformation.translation.z);
            Eigen::Quaterniond q(response->transformation.rotation.w, response->transformation.rotation.x, response->transformation.rotation.y, response->transformation.rotation.z);
            Eigen::Vector3d x_recovered = q.inverse() * (x_new - t);
            return x_recovered;
        }
    
    };





int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNode>();
    node->send_request();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}