

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>
// #include <Eigen/Geometry>



#include <iostream>


Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

/*
You can use this function to implement formulation of the matrix
*/
Eigen::Matrix4f form_matrix(float x, float y, float angle) {
    //Eigen::Matrix4f pose; <-- for 3D pose
    //Eigen::Matrix3f pose; <-- for 2D pose

    Eigen::Matrix4f pose;

    // you code here


    // you code here
    return pose;
}




int main() {
    
    // create matrices needed for calculation


    // Calculate the end effector position and save the type
    // to eeFrame
    Eigen::Matrix4f eeFrame; // final transform

    std::cout << eeFrame.format(CleanFmt) << std::endl; // print formated matrix to console


    std::cout << "Hello, world!" << std::endl;
    return 0;
}