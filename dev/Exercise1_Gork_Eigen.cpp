/*
Simple example showing how to use solvers in the KDL and creating kinematic model.

Task 1: Exercise task is to create chain forward kinematic solver and read end effector position given 
the kinematic chain.
*/
#include <iostream>
#include <Eigen/Geometry>

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

/*
You can use this function to implement formulation of the matrix
*/
Eigen::Matrix4f form_matrix(float x, float y, float angle) {

    //you code here
    //Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(); // for 3D pose
    //Eigen::Matrix3f pose = Eigen::Matrix3f::Identity(); // for 2D pose
   
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();

    Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
    transform.rotate(rotation);
    transform.pretranslate(Eigen::Vector3f(x, y, 0.0f));

    return transform.matrix();
}


Eigen::Matrix4f old_form_matrix(float x, float y, float angle) {

    //you code here
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(); // for 3D pose
    //Eigen::Matrix3f pose = Eigen::Matrix3f::Identity(); // for 2D pose

    pose(0, 3) = x;
    pose(1, 3) = y;

    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

    float cos_theta = std::cos(angle);
    float sin_theta = std::sin(angle);

    transform_matrix(0, 0) = cos_theta;
    transform_matrix(0, 1) = -sin_theta;
    transform_matrix(1, 0) = sin_theta;
    transform_matrix(1, 1) = cos_theta;

    // you code here
    return transform_matrix * pose;
} 

int main(int argc, char **argv)
{
    // create matrices needed for calculation
    float x = 2.0f;
    float y = 3.0f;
    float angle = M_PI / 4;  //45 degrees
    // Calculate the end effector position and save the type
    // to eeFrame
    Eigen::Matrix4f eeFrame = form_matrix(x, y, angle); // final transform

    Eigen::Matrix4f oldFrame =  old_form_matrix(x, y, angle);


    std::cout << "new" << eeFrame.format(CleanFmt) << std::endl; // print formated matrix to console
    std::cout << "old" << oldFrame.format(CleanFmt) << std::endl; // print formated matrix to console
    return 0;
}

