#include <iostream>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "Eigen/Dense"
#include "Eigen/QR"
#include <math.h>       /* pow */

using namespace Eigen;
using namespace std;

typedef Matrix<double, 6, 6> Matrix_d_6x6;
typedef Matrix<double, 3, 3> Matrix_d_3x3;
typedef Matrix<double, 6, 1> Vector_d_6x1;
typedef Matrix<double, 3, 1> Vector_d_3x1;

struct Twist
{
    Vector_d_3x1 lin_vel;
    Vector_d_3x1 ang_vel;
};


struct Orientation
{
    double angle;
    Vector_d_3x1 axis;
};

Matrix_d_6x6 compute_damped_pseudo_inv(Matrix_d_6x6 M);
Twist compute_twist(Matrix_d_6x6 J, Vector_d_6x1 dq);
Orientation rot2axisAngle(Matrix_d_3x3 R);
Vector_d_3x1 compute_axisangle_error(Matrix_d_3x3 R_des, Matrix_d_3x3 R_med);


int main(){
    
    const string urdf_filename=string("/home/jhon/catkin_ws/journal_ws/src/ur5_description/urdf/ur5_joint_limited_robot.urdf");

    // Load the URDF model
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    cout << "model name: " << model.name << endl;

    // useful vectors
    Vector_d_6x1 q, dq, ddq;
    q.Zero();
    dq.Zero();
    ddq.Zero();


    // Home position
			

    q[0] = -0.22007212; 
    q[1] = -0.72670633; 
    q[2] = -4.72730825;  
    q[3] = 0.7416256;   
    q[4] = 1.57079633; 
    q[5] = -3.36166477;
    // Build a data related to model
    pinocchio::Data data(model);

    pinocchio::crba(model,data,q);
    const Eigen::MatrixXd M = data.M;
    cout << "\n\nM = " << M << endl;

    
    pinocchio::rnea(model,data,q, dq, ddq);
    const Eigen::VectorXd b=data.tau;
    cout << "\n\nb = " << b << endl;    

    //const Eigen::MatrixXd J; 
    Matrix_d_6x6 J;
    const std::string frame_ID = "ee_link";
    cout<<"\n\nframe ID: "<<model.getFrameId(frame_ID)<<endl;
     
    pinocchio::computeJointJacobians(model,data,q);
    pinocchio::forwardKinematics(model,data,q);
    pinocchio::getFrameJacobian(model, data, model.getFrameId(frame_ID), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
    cout << "\n\nJ = " << J << endl;

    Matrix_d_6x6 dJ;
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
    pinocchio::getFrameJacobianTimeVariation(model, data, model.getFrameId(frame_ID),pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
    cout << "\n\ndJ = " << dJ << endl;

    Matrix_d_6x6 invJ;
    invJ = compute_damped_pseudo_inv(J);
    cout << "\n\ninvJ = " << invJ << endl;

    Twist twist;
    twist = compute_twist(J, dq);
    cout << "\n\n v = " << twist.lin_vel << endl;
    cout << "\n\n w = " << twist.ang_vel << endl;

    Vector_d_3x1 pos = data.oMf[model.getFrameId(frame_ID)].translation();
    Matrix_d_3x3 R = data.oMf[model.getFrameId(frame_ID)].rotation();
    cout << "\n\n pos = " << pos << endl;
    cout << "\n\n R = " << R << endl;

    cout<<"I can read the Pinocchio library!"<<endl;

    return 0;
}

Matrix_d_6x6 compute_damped_pseudo_inv(Matrix_d_6x6 M)
{
    double _lambda=0.0000001;
    Matrix_d_6x6 I = Matrix_d_6x6::Identity();
    return M.transpose()*((M*M.transpose() + _lambda*I).inverse());
}

Twist compute_twist(Matrix_d_6x6 J, Vector_d_6x1 dq)
{
    Twist twist;
    twist.ang_vel = (J.block<3,6>(0,0))*dq;
    twist.ang_vel = (J.block<3,6>(3,0))*dq;
    return twist;
}

Orientation rot2axisAngle(Matrix_d_3x3 R)
{
    double R32=R(2,1), R23=R(1,2), R13=R(0,2), R31=R(2,0), R21=R(1,0), R12=R(0,1), tr=R.trace();
    Orientation ori;

    ori.angle = atan2(0.5*sqrt( pow(R21-R12,2)+pow(R31-R13,2)+pow(R32-R23,2) ), 0.5*(tr-1) );
    if (ori.angle!=0)
    {
        ori.axis(0,0) = (R32-R23)/(2*sin(ori.angle));
        ori.axis(1,0) = (R13-R31)/(2*sin(ori.angle));
        ori.axis(2,0) = (R21-R12)/(2*sin(ori.angle));
    }
    else
    {
        ori.axis(0,0) = 0.0;
        ori.axis(1,0) = 0.0;
        ori.axis(2,0) = 0.0;        
    }

    return ori;
}

Vector_d_3x1 compute_axisangle_error(Matrix_d_3x3 R_des, Matrix_d_3x3 R_med)
{
    Matrix_d_3x3 R_e = R_des*R_med;
    Orientation ori_e;
    ori_e = rot2axisAngle(R_e);

    return R_med*(ori_e.angle*ori_e.axis);
}

