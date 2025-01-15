
#include <Eigen/Dense>
#include <vector>
#include<iostream>


namespace EigenUtil {

    void print_vector3d(const Eigen::Vector3d& vec) {
        std::cout << "["
            << vec(0) << ", "
            << vec(1) << ", "
            << vec(2) << "]"
            << std::endl;
    }

    void print_matrix3d(const Eigen::Matrix3d& mat) {
        
        for (int i = 0; i < 3; ++i) {
            if (i == 0) {
                std::cout << "[";
            }
            else {
                std::cout << " ";
            }

            std::cout << "["
                << std::setw(10) << mat(i, 0) << " "
                << std::setw(10) << mat(i, 1) << " "
                << std::setw(10) << mat(i, 2);

            if (i == 2) {
                std::cout << "]]" << std::endl;
            }
            else {
                std::cout << "]," << std::endl;
            }
        }
    }

    Eigen::Vector3d from_std_vector(const std::vector<double>& vec){
        Eigen::Vector3d out_vec(3);
        for (size_t i=0; i<3; ++i){
            out_vec[i] = vec[i];
        }
        return out_vec;
    }
}
