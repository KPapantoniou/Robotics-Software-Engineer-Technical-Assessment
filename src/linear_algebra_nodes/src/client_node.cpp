#include <yaml-cpp/yaml.h>
#include <iostream>
using namespace std;

int main(){
    // Load the YAML file and create A, b from it
    YAML::Node config = YAML::LoadFile("../../data.yaml");
    vector<vector<double>> A = config["matrix_a"].as<vector<vector<double>>>();
    vector<double> b = config["vector_b"].as<vector<double>>();
    // Print A and b to verify they were loaded correctly
    cout << "Matrix A:" << endl;
    for(const auto& row: A){
        for(const auto& value: row){
            cout << value << " ";
        }
    } 
    cout << endl << "Vector b:" << endl;
    for(const auto& value: b){
        cout << value << " ";
    }

    return 0;
}