#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_service/srv/least_squares.hpp"
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "geometry_msgs/msg/vector3.hpp"

using namespace linear_algebra_service::srv;

// ServerNode class definition
class ServerNode : public rclcpp::Node{
    public:
        ServerNode(): Node("server_node"){
            
            using namespace std::placeholders;
            // Create a service with a callback function that solves the equation uppon receiving a request
            this->service_ = create_service<LeastSquares>(
                "LeastSquares",
                std::bind(&ServerNode::find_least_squares_solution, this, _1, _2)
            );
            worker_thread_ = std::thread(&ServerNode::worker_thread_function, this);
            //creates subscription, name must match publisher's topic name "transformed_vector"
            subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
                "transformed_vector",
                10,
                std::bind(&ServerNode::transformed_vector_callback, this, _1)
            );
            // Start the worker thread to process received transformed vectors
            
        }

        // Destructor to join the worker thread before shutting down
        ~ServerNode(){
            worker_thread_.join();       
        }

        // Function to log messages to the console
        void logging(const std::string &message){
            RCLCPP_INFO(rclcpp::get_logger("least_squares_node"), message.c_str());
        }
        
    private:
        // instance of the service as a share pointer to be used in the constructor
        rclcpp::Service<LeastSquares>::SharedPtr service_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
        bool received_message_ = false;
        std::thread worker_thread_;
        std::mutex mutex_;
        std::condition_variable cv_;
        geometry_msgs::msg::Vector3 latest_message_;
        // Callback function to solve the least squares problem with eingen library
        void find_least_squares_solution(const std::shared_ptr<LeastSquares::Request> request, const std::shared_ptr<LeastSquares::Response> response){
            auto A = request->matrix_a;
            Eigen::MatrixXd A_eigen(A.size(), 3);
            for(size_t i = 0; i < A.size(); ++i){
                A_eigen(i, 0) = A[i].x;
                A_eigen(i, 1) = A[i].y; 
                A_eigen(i, 2) = A[i].z;
            }

            auto b = request->vector_b;
            Eigen::Vector3d b_eigen;
            b_eigen << b.x, b.y, b.z;
            logging("===== Received Request =====");
            logging("[server] Solving least squares problem for " + std::to_string(A_eigen.size()) + " x3 matrix.");
            logging("[server] Matrix A:");    
            for(int i = 0; i < A_eigen.rows(); ++i){
                
                std::string row_str;
                for(int j = 0; j < A_eigen.cols(); ++j){
                    row_str += std::to_string(A_eigen(i, j)) + ", ";
                }
                logging(row_str+ "]");
            }
            logging("[server] Vector b: [" + std::to_string(b_eigen.x()) + ", " + std::to_string(b_eigen.y()) + ", " + std::to_string(b_eigen.z()) + "]");
            Eigen::Vector3d x = A_eigen.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_eigen);
            logging("[server] Computed least squares solution x: [" + std::to_string(x.x()) + ", " + std::to_string(x.y()) + ", " + std::to_string(x.z()) + "]");

            Eigen::Vector3d t = Eigen::Vector3d::Random();
            Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();

            Eigen::Vector3d x_new = q * x + t;
            logging("[server] Transformed vector: [" + std::to_string(x_new.x()) + ", " + std::to_string(x_new.y()) + ", " + std::to_string(x_new.z()) + "]");
            logging("[server] Transformation translation: [" + std::to_string(t.x()) + ", " + std::to_string(t.y()) + ", " + std::to_string(t.z()) + "]");
            logging("[server] Transformation rotation (quaternion): [" + std::to_string(q.x()) + ", " + std::to_string(q.y()) + ", " + std::to_string(q.z()) + ", " + std::to_string(q.w()) + "]"); 
            
            response->x.x = x_new(0);
            response->x.y = x_new(1);
            response->x.z = x_new(2);
            
            response->transformation.translation.x = t.x();
            response->transformation.translation.y = t.y();
            response->transformation.translation.z = t.z();
            response->transformation.rotation.x = q.x();
            response->transformation.rotation.y = q.y();
            response->transformation.rotation.z = q.z();
            response->transformation.rotation.w = q.w();
            
            logging("[server] Response sent with transformed solution and transformation details.");
            
        }
        // Callback function to receive transformed vector from client and log it
        void transformed_vector_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
            std::unique_lock<std::mutex> lock(mutex_);
            latest_message_ = *msg;
            received_message_ = true;
            cv_.notify_one();
        }
        // Worker thread function to process received transformed vectors
        void worker_thread_function(){
             while(true){
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]{return received_message_;});
                logging("[server] Processing received transformed vector: [" + std::to_string(latest_message_.x) + ", " + std::to_string(latest_message_.y) + ", " + std::to_string(latest_message_.z) + "]");
                received_message_ = false;
            }
        }
};

int main(int argc , char* argv[]){

    rclcpp::init(argc,argv);
    auto node = std::make_shared<ServerNode>();
   
    node -> logging("Least Squares Service is ready to receive requests.");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}