/*
Simple example showing how to use solvers in the KDL and creating kinematic model.

Task 1: Your task is to create kinematic chain in KDL using KDL::Chain and calculate the end effector rotation
        and translation matrices using KDL::ChainFkSolverPos_recursive to calculate the forward kinematics

    HELP: For the Chain creation http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Chain.html
    and for the solver http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1TreeFkSolverPos__recursive.html

Task 2: Access the position and rotation data from the KDL::Frame variable
    HELP: http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Frame.html

*/
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>

#include <iomanip> // for std::setprecision
#define _USE_MATH_DEFINES
#include <cmath>

// #include <Eigen>

// Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

using namespace KDL;


void printArray(double array[], int len = 3){

    for(int i=0; i < len; ++i)
    {
        std::cout << std::fixed << std::setprecision(3) << array[i] << ", ";

    }
    std::cout << std::endl;

}

void prettyPrint(KDL::JntArray jointAngles,float dx, float dz, float l1, float l2, Frame F_result){

    using namespace std;
    {
        cout<< endl;
        cout<< "For base position of: x=" << std::fixed << std::setprecision(3) << dx << ", z=" << dz << endl;
        cout<< "Arm lenghts: L1=" << std::fixed << std::setprecision(3) << l1 << ", L2=" << l2 << endl;
        cout<< "Joint angles: q1=" << std::fixed << std::setprecision(3) << jointAngles(0) << ", q2=" << jointAngles(1) << " [rad]" << endl;
        cout<< "The end effector position is at position: " ;
        printArray(F_result.p.data);
    } // namespace std
    

}

int main(int argc, char **argv)
{
    float dx = 0;
    float dz = 0;
    
    float l1 = 1;
    float l2 = 1;

    

    KDL::Chain kdlChain = KDL::Chain();
    
    kdlChain.addSegment( Segment(Joint(Joint::None),Frame(Vector(dx,0.0,dz))) ) ;
    kdlChain.addSegment( Segment(Joint(Joint::RotY),Frame(Vector(l1,0.0,0.0))) );
    kdlChain.addSegment( Segment(Joint(Joint::RotY),Frame(Vector(l2,0.0,0.0))) );
    //kdlChain.addChain( Segment(Joint(Joint::TransX),Frame(Vector(l2,0.0,0.0))) )
    // your code here to form chain

    // your code here to form chain

    unsigned int nj = kdlChain.getNrOfJoints();
    unsigned int js = kdlChain.getNrOfSegments();

    std::cout << nj <<std::endl;
    std::cout << js <<std::endl;


    // KDL::JntArray jointAngles = KDL::JntArray(3);
    KDL::JntArray jointAngles = KDL::JntArray(kdlChain.getNrOfJoints());
    
    jointAngles(0) = M_PI/2.0;
    jointAngles(1) = 0; //-M_PI/2;
   

    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChain);
    
    Frame F_result;
    FKSolver.JntToCart(jointAngles,F_result);

    printArray(F_result.p.data);

    prettyPrint(jointAngles,dx,dz,l1,l2,F_result);

    return 0;
}