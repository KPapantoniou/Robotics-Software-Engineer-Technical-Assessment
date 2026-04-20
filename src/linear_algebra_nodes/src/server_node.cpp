#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_service/srv/least_squares.hpp"
#include <thread>
#include <eigen3/Eigen/Dense>

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
        }

        // Function to log messages to the console
        void logging(const std::string &message){
            RCLCPP_INFO(rclcpp::get_logger("least_squares_node"), message.c_str());
        }
        
    private:
        // instance of the service as a share pointer to be used in the constructor
        rclcpp::Service<LeastSquares>::SharedPtr service_;
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

                    
        

            Eigen::Vector3d x = A_eigen.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_eigen);
            response->x.x = x(0);
            response->x.y = x(1);
            response->x.z = x(2);
            
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