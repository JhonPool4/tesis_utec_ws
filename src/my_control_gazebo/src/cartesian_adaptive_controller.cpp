// =======================================
// author 	: 	jhon charaja
// email 	: 	jhon.charaja@utec.edu.pe
// info 	: 	cartesian adaptive controller
// =======================================

// Pinocchio
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
// Control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <trajectory_msgs/JointTrajectory.h>
// Ros msg
#include <my_control_gazebo/Pose.h>
#include <std_msgs/Bool.h>
// Console
#include <ros/console.h>
// Real time
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Eigen
#include <iostream>
#include "Eigen/Dense"
#include "Eigen/QR"
// C++ math
#include <math.h>
// name spaces
using namespace Eigen;
using namespace std;

// useful matrix templates
typedef Matrix<double, 6, 6> Matrix_d_6x6;
typedef Matrix<double, 3, 3> Matrix_d_3x3;
typedef Matrix<double, 6, 1> Vector_d_6x1;
typedef Matrix<double, 3, 1> Vector_d_3x1;

// useful structures
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

namespace effort_controllers_ns{

	class CartesianAdaptiveController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
        // type: effort signal
        // space: task-space 
        // control method: proportional-derivative
		private:
			// to save joint names
			std::vector< std::string > _joint_names;
			// to handle joint vector
			std::vector< hardware_interface::JointHandle > _joints;
			// number of joints
			unsigned int _n_joints;
			// real time buffer: desired pose 
			realtime_tools::RealtimeBuffer<std::vector<double> > _pose_command;
			realtime_tools::RealtimeBuffer<std::vector<double> > _dpose_command;
			realtime_tools::RealtimeBuffer<std::vector<double> > _ddpose_command;
			// Subscriber
			ros::Subscriber _sub_pose_command;
			
			// Publisher
			ros::Publisher _pub_start_command;
			ros::Publisher _pub_current_pose;
			// Control gains
			Matrix_d_6x6 _kp;
			Matrix_d_6x6 _kd;
			
			// Pinocchio model
            const string _urdf_filename=string("/home/jhon/catkin_ws/tesis_utec_ws/src/ur5_description/urdf/ur5_joint_limited_robot.urdf");
			pinocchio::Model _model;
			// Build a data related to model
			pinocchio::Data _data;
			const std::string _frame_ID = "ee_link";
			
			// Just to print
			int _counter = 0.0;
			int _p_rate = 100;
			// start flag
			bool _send_start_command = true;
			// current joint states
			my_control_gazebo::Pose _pose;

			// useful vectors	
			Vector_d_6x1 q; 	// current angular position
			Vector_d_6x1 dq;	// current angular velocity
			Vector_d_6x1 z; 	// zeros
			Vector_d_6x1 x_des; // desired end-effector pose
			Vector_d_6x1 dx_des; // desired end-effector dpose
			Vector_d_6x1 ddx_des; // desired end-effector ddpose
			Matrix_d_3x3 R_des;  // desired rotation matrix
			Vector_d_3x1 w_des;  // desired angular velocity
			Vector_d_3x1 dw_des; // desired angular acceleration
			Vector_d_6x1 ddpw_des; // desired linear and angular acceleration

			Vector_d_3x1 p_med; // measured end-effector pose
			Vector_d_3x1 dp_med; // measured end-effector dpose	
			Vector_d_3x1 w_med; // measured angular velocity
			Matrix_d_3x3 R_med;  // measured rotation matrix		

			Vector_d_6x1 x_e; // error end-effector pose
			Vector_d_6x1 dx_e; // error end-effector dpose
			Vector_d_6x1 ddx_e; // error end-effector ddpose

			// control terms
			Vector_d_6x1 F;
			Vector_d_6x1 u;			

			Matrix_d_6x6 J; // Jacobian
			Matrix_d_6x6 Jinv; // inverse Jacobian
			Matrix_d_6x6 dJ; // time-derivative of Jacobian

			Matrix_d_6x6 M; // inertia matrix at joint space
			Matrix_d_6x6 Mx; // inertia matrix at Cartesian space
			Vector_d_6x1 b; // nonlinear effects vector	at joint space
			Vector_d_6x1 bx; // nonlinear effects vector at Cartesian space		

			// adaptation terms
			Vector_d_6x1 s; // sliding surface
			Vector_d_6x1 ds; // time-derivative of s	
			Matrix_d_6x6 M_hat; // new inertia matrix
			Matrix_d_6x6 _alpha; // learning rate

        public:
			bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
			{
				ROS_INFO_STREAM("============================");
                // Read joints of robot
				std::string param_name = "joints";
				if(!n.getParam(param_name, _joint_names))
				{
					ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
					return false;
				}
				
				ROS_INFO_STREAM("Success to getParam '" << param_name <<"'(namespace: "<< n.getNamespace() << "). ");
				
				// Number of DoF
				_n_joints = _joint_names.size();

				// Add joints to hardware interface
				for(unsigned int i=0; i<_n_joints; i++)
				{
					try
					{
						_joints.push_back(hw->getHandle(_joint_names[i]));
						ROS_INFO_STREAM("Joint ("<<_joint_names[i]<<") added to hardware interface.");
					}
					catch (const hardware_interface::HardwareInterfaceException& e)
					{
						ROS_ERROR_STREAM("Exception thrown: " << e.what());
						return false;
					}
				}
				
				// resize command buffer with non real time communication
				_pose_command.writeFromNonRT(std::vector<double>(6, 0.0));
				_dpose_command.writeFromNonRT(std::vector<double>(6, 0.0));
				_ddpose_command.writeFromNonRT(std::vector<double>(6, 0.0));
				// Subscribers
				_sub_pose_command = n.subscribe<my_control_gazebo::Pose>("pose_command", 1, &CartesianAdaptiveController::poseCommandCB, this);
				// Publishers
				_pub_start_command = n.advertise<std_msgs::Bool>("gazebo_start_command", 1);
				//_pub_current_pose = n.advertise<my_control_gazebo::Pose>("measured_pose", 1);
				
                // load urdf model
                pinocchio::urdf::buildModel(_urdf_filename, _model);				
				_data = pinocchio::Data(_model);
				
				return true;
            }

            void poseCommandCB(const my_control_gazebo::Pose::ConstPtr& msg){
                std::vector<double> _desired_pose(6,0.0); // to recieve message  
                // Reciving desired position message
				_desired_pose[0] = msg->x;
				_desired_pose[1] = msg->y;
				_desired_pose[2] = msg->z;
				_desired_pose[3] = msg->roll;
				_desired_pose[4] = msg->pitch;
				_desired_pose[5] = msg->yaw;

				std::vector<double> _desired_dpose(6,0.0); // to recieve message  
                // Reciving desired position message
				_desired_dpose[0] = msg->dx;
				_desired_dpose[1] = msg->dy;
				_desired_dpose[2] = msg->dz;
				_desired_dpose[3] = msg->wx;
				_desired_dpose[4] = msg->wy;
				_desired_dpose[5] = msg->wz;			

				std::vector<double> _desired_ddpose(6,0.0); // to recieve message  
                // Reciving desired position message
				_desired_ddpose[0] = msg->ddx;
				_desired_ddpose[1] = msg->ddy;
				_desired_ddpose[2] = msg->ddz;
				_desired_ddpose[3] = msg->dwx;
				_desired_ddpose[4] = msg->dwy;
				_desired_ddpose[5] = msg->dwz;							
                
				// Send desired angular position
				_pose_command.initRT(_desired_pose); // require std::vector<double>
				_dpose_command.initRT(_desired_dpose);	 
				_ddpose_command.initRT(_desired_ddpose);	                
            }

			void starting(const ros::Time& time)
			{
				// control values according to theraban brand
				_kp = Matrix_d_6x6::Identity()*200;
				_kd = Matrix_d_6x6::Identity()*30;

				// new inertia matrix
				M_hat = Matrix_d_6x6::Zero();
				// learning rate
				_alpha.diagonal() << 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5;
				
				// Vector_d_3x1::Identity()*1e-4; // position
				//_alpha.block<3,1>(3,0) = Vector_d_3x1::Identity()*1e-5; // orientation 

				// To recieve angular position and velocity message
				std::vector<double> _desired_pose(6,0.0);
				std::vector<double> _desired_dpose(6,0.0);
				std::vector<double> _desired_ddpose(6,0.0);

				// Set desired pose
				_desired_pose[0] = 0.5;
				_desired_pose[1] = 0.0;
				_desired_pose[2] = 0.0;
				_desired_pose[3] = M_PI/4;
				_desired_pose[4] = 0.0;
				_desired_pose[5] = 0.0;
				// Send desired pose
				_pose_command.initRT(_desired_pose);			  						
				_dpose_command.initRT(_desired_dpose);
				_ddpose_command.initRT(_desired_ddpose);				
					
			}

            void update(const ros::Time& time, const ros::Duration& period)
			{

				// ===============================================================
				//   Fist stage: Read joint states and compute forward kinematics    
				// ===============================================================
				for(unsigned int i=0; i<_n_joints; i++)
				{										
					// get current angular position and velocity
					q[i] = _joints[i].getPosition();
					dq[i] = _joints[i].getVelocity();
				}
				
				// these three methods compute fk and jabocobians; and store in _data object
				pinocchio::forwardKinematics(_model,_data,q);
    			pinocchio::computeJointJacobians(_model,_data,q);
				pinocchio::computeJointJacobiansTimeVariation(_model, _data, q, dq);				

				// measured Cartesian position
				p_med = _data.oMf[_model.getFrameId(_frame_ID)].translation();
				// measured rotation matrix
				R_med = _data.oMf[_model.getFrameId(_frame_ID)].rotation();
				
				// jacobian
				pinocchio::getFrameJacobian(_model, _data, _model.getFrameId(_frame_ID), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
				// time-derivative of jacobian
				pinocchio::getFrameJacobianTimeVariation(_model, _data, _model.getFrameId(_frame_ID),pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ);
				// inverse jacobian
				Jinv = compute_damped_pseudo_inv(J);
				
				// compute twist
				dp_med = (J.block<3,6>(0,0))*dq; // measured linear velocity
				w_med  = (J.block<3,6>(3,0))*dq; // measured angular velocity

				// ======================================================
				//   Second stage: Orientation and derivatives mapping         
				// ======================================================	
				std::vector<double> & des_pose = *_pose_command.readFromRT();
				std::vector<double> & des_dpose = *_dpose_command.readFromRT();
				std::vector<double> & des_ddpose = *_ddpose_command.readFromRT();	

				for(unsigned int i=0; i<_n_joints; i++)
				{										
					// pose and dpose with eigen vectors
					x_des[i] =des_pose[i];
					dx_des[i] =des_dpose[i];
					ddx_des[i] =des_ddpose[i];
				}	
				//R_des << 0, 0, 1,
				//		 0, 1, 0,
				//		 -1, 0, 0; 
				R_des = rpy2rot(x_des.block<3,1>(3,0));
				w_des = rpy2angularVel(x_des.block<3,1>(3,0), dx_des.block<3,1>(3,0));
				dw_des = rpy2angularAcc(x_des.block<3,1>(3,0), dx_des.block<3,1>(3,0), ddx_des.block<3,1>(3,0));

				ddpw_des.block<3,1>(0,0) = ddx_des.block<3,1>(0,0);
				ddpw_des.block<3,1>(3,0) = dw_des;
				//ddpw_des.Zero();
				
				// ======================================================
				//   Third stage: End-Effector Pose error         
				// ======================================================	
				// Cartesian error: position
				x_e.block<3,1>(0,0) = x_des.block<3,1>(0,0) - p_med;
				dx_e.block<3,1>(0,0) = dx_des.block<3,1>(0,0) - dp_med;
				// Cartesian error: orientation
				x_e.block<3,1>(3,0) = compute_axisangle_error(R_des, R_med);
				dx_e.block<3,1>(3,0) = w_des - w_med;
				
				// ======================================================
				//   Fourth stage: Compute dynamics         
				// ======================================================				
				// compute robot dynamics
				pinocchio::crba(_model, _data, q); z.Zero();
				pinocchio::rnea(_model, _data, q, dq, z);		
				M = _data.M; // inertia matrix
				Mx = compute_damped_pseudo_inv(J*(M.inverse()*J.transpose()));
				b = _data.tau; // nonlinear effects vector
				bx = J.inverse().transpose()*b - J.inverse().transpose()*M*dJ*dq;

				// ======================================================
				//   Fifth stage: Compute control signal      
				// ======================================================
				F = (Mx+ M_hat)*(ddpw_des + _kp*x_e + _kd*dx_e) + bx;
				u = eval_control_limits(J.transpose()*F);
				//u = J.transpose()*F;

				// send control signal
				for (int i = 0; i < 6; ++i)
				{	
					double effort_command = u[i];
					_joints[i].setCommand(effort_command);
				}

				// ======================================================
				//   Sixth stage: Adaptation process      
				// ======================================================				
				s = dx_e + 0.5*_kd*x_e;
				ds = -0.5*_kd*dx_e - _kp*x_e; 
				/*
				for (int i=0; i<6; ++i)
				{
					M_hat(i,i) = M_hat(i,i) - _alpha(i,i)*(s(i)*ds(i));
				}
				*/
				
				
				if (std::abs(x_e(0)) >= 1e-2){M_hat(0,0) = M_hat(0,0) - _alpha(0,0)*( s(0)*ds(0) );}
				if (std::abs(x_e(1)) >= 1e-2){M_hat(1,1) = M_hat(1,1) - _alpha(1,1)*( s(1)*ds(1) );}
				if (std::abs(x_e(2)) >= 1e-2){M_hat(2,2) = M_hat(2,2) - _alpha(2,2)*( s(2)*ds(2) );}
				if (std::abs(x_e(3)) >= (3*M_PI/180)){M_hat(3,3) = M_hat(3,3) - _alpha(3,3)*( s(3)*ds(3) );}
				if (std::abs(x_e(4)) >= (3*M_PI/180)){M_hat(4,4) = M_hat(4,4) - _alpha(4,4)*( s(4)*ds(4) );}
				if (std::abs(x_e(5)) >= (3*M_PI/180)){M_hat(5,5) = M_hat(5,5) - _alpha(5,5)*( s(5)*ds(5) );}
								
				/*
				for(int i=0; i<3; ++i)
				{
					if (std::abs(x_e(i)) >= 5e-3){M_hat(i,i) = M_hat(i,i) - _alpha(i)*( s(i)*ds(i) );}	
				}
	
				for(int i=3; i<6; ++i)
				{
					if (std::abs(x_e(i)) >= (1*M_PI/180)){M_hat(i,i) = M_hat(i,i) - _alpha(i)*( s(i)*ds(i) );}	
				}*/
				//print
				_counter += 1;
				if (true)
				{
					if (_counter>= 50000){
						_counter = 0;
					}
					if (_counter%_p_rate == 0){
						std::cout<<"\n==============="<<std::endl;
						//std::cout<<"\nM: "<<M <<std::endl;
						//std::cout<<"\nb: "<<b<<std::endl;
						//std::cout<<"\np_med: "<<p_med.transpose()<<std::endl;
						//std::cout<<"\nR_med: "<<R_med<<std::endl;
						//std::cout<<"\nx_des: "<<x_des.transpose()<<std::endl;
						//std::cout<<"\nR_des: "<<R_des<<std::endl;	
						std::cout<<"\nrecieving_data: "<<!_send_start_command<<std::endl;	
						std::cout<<"\npos_e: "<<100*x_e.block<3,1>(0,0).transpose()<<" cm"<<std::endl;
						std::cout<<"\nori_e: "<<(180/M_PI)*x_e.block<3,1>(3,0).transpose()<<" deg"<<std::endl;
						//std::cout<<"\ndx_e: "<<dx_e.transpose()<<std::endl;
						std::cout<<"\nnorm_pos: "<<100*x_e.block<3,1>(0,0).norm()<<" cm"<<std::endl;
						std::cout<<"\nnorm_ori: "<<(180/M_PI)*x_e.block<3,1>(3,0).norm()<<" deg"<<std::endl;
						//std::cout<<"\ns: "<<s.transpose()<<std::endl;
						//std::cout<<"\nds: "<<ds.transpose()<<std::endl;
						std::cout<<"\nM_hat: "<<M_hat.diagonal()<<std::endl;
						std::cout<<"\nalpha: "<<_alpha.diagonal()<<std::endl;
						//std::cout<<"\nddpw_des: "<<ddpw_des<<std::endl;				
						std::cout<<"\nq: "<<q.transpose()<<std::endl;
						//std::cout<<"\nJinv: "<<J.inverse()<<std::endl;
						//std::cout<<"\nMx: "<<Mx<<std::endl;	
						//std::cout<<"\nbx: "<<bx<<std::endl;	
						std::cout<<"\ntau: "<<u.transpose()<<std::endl;	
					}
				}

				if ((x_e.block<3,1>(0,0).norm()<0.008) && _send_start_command)
				{
					std::cout<<"Pose error achieved ..."<<std::endl;
					std::cout<<"Sending start signal to recieve trajectory ..." <<std::endl;
					_send_start_command=false;
					std_msgs::Bool start_signal;
					start_signal.data = true;
					_pub_start_command.publish(start_signal);
				}


			}

            void stopping(const ros::Time& time){
				std::cout<<"Sending stop signal to recieve trajectory ..." <<std::endl;
				std_msgs::Bool start_signal;
				start_signal.data = false;
				_pub_start_command.publish(start_signal);				
			}

			// @info limits control signal to safe values
			// @param u control singal
			Vector_d_6x1 eval_control_limits(Vector_d_6x1 u)
			{
				float scale = 1;
				float size0 = 12*scale;
				float size1 = 28*scale;
				float size2 = 56*scale;
				float size3 = 150*scale;
				float size4 = 330*scale;
				Vector_d_6x1 tau;

				if (std::abs(u[0]) >= size3)
				{	tau[0] = sgn(u[0])*size3;	}
				else{ tau[0] = u[0]; }

				if (std::abs(u[1]) >= size3)
				{	tau[1] = sgn(u[1])*size3;	}
				else{ tau[1] = u[1]; }

				if (std::abs(u[2]) >= size3)
				{	tau[2] = sgn(u[2])*size3;	}
				else{ tau[2] = u[2]; }

				if (std::abs(u[3]) >= size1)
				{	tau[3] = sgn(u[3])*size1;	}
				else{ tau[3] = u[3]; }

				if (std::abs(u[4]) >= size1)
				{	tau[4] = sgn(u[4])*size1;	}
				else{ tau[4] = u[4]; }

				if (std::abs(u[5]) >= size1)
				{	tau[5] = sgn(u[5])*size1;	}
				else{ tau[5] = u[5]; }	
				
				return tau;			
			}

			// @info signed function
			int sgn(double val)  
			{
			  if (val >= 0){ return 1;}

			  else{return -1;}
			} 

			// Useful functions
			Matrix_d_3x3 rpy2rot(Vector_d_3x1 rpy)
			{
				/*
				@info: computes rotation matrix from roll, pitch, yaw (ZYX euler angles) representation

				@inputs:
				-------
				- rpy[0]: rotation in z-axis (roll)
				- rpy[1]: rotation in y-axis (pitch)
				- rpy[2]: rotation in x-axis (yaw)
				@outputs:
				--------
				- R: rotation matrix        
				*/

				Matrix_d_3x3 Rz, Ry, Rx;

				Rz<< cos(rpy[0]), -sin(rpy[0]), 0,
					 sin(rpy[0]), cos(rpy[0]) , 0,
					 	0, 			 0, 		1;

				Ry<< cos(rpy[1])   ,   0   ,    sin(rpy[1]),
					  0            ,   1   ,           0,
					-sin(rpy[1])   ,   0   ,   cos(rpy[1]);

				Rx<< 1   ,    0           ,        0,
					 0   ,    cos(rpy[2]) ,  -sin(rpy[2]),
					 0   ,    sin(rpy[2]) ,   cos(rpy[2]);

				return  (Rz*Ry)*Rx;
			}

			Vector_d_3x1 rpy2angularVel(Vector_d_3x1 rpy, Vector_d_3x1 drpy)
			{
				/*
				@info: compute angular velocity (w) from euler angles (roll, pitch and yaw) and its derivaties
				@inputs:
				-------
				- rpy[0]: rotation in z-axis (roll)
				- rpy[1]: rotation in y-axis (pitch)
				- rpy[2]: rotation in x-axis (yaw)
				- drpy[0]: rotation ratio in z-axis
				- drpy[1]: rotation ratio in y-axis
				- drpy[2]: rotation ratio in x-axis
				@outputs:
				--------
				- w: angular velocity

				*/
				
				Matrix_d_3x3 E0;
				E0 << 0, -sin(rpy[0]),  cos(rpy[0])*cos(rpy[1]),
					  0,  cos(rpy[0]),  sin(rpy[0])*cos(rpy[1]),
					  1,         0,          -sin(rpy[1]);

				return E0*drpy;
						
			}

			Vector_d_3x1 rpy2angularAcc(Vector_d_3x1 rpy, Vector_d_3x1 drpy, Vector_d_3x1 ddrpy)
			{
				/*
				@info: compute angular velocity (w) from euler angles (roll, pitch and yaw) and its derivaties
				@inputs:
				-------
				- rpy[0]: rotation in z-axis (roll)
				- rpy[1]: rotation in y-axis (pitch)
				- rpy[2]: rotation in x-axis (yaw)
				- drpy[0]: rotation speed in z-axis
				- drpy[1]: rotation speed in y-axis
				- drpy[2]: rotation speed in z-axis
				- ddrpy[0]: rotation acceleration in z-axis
				- ddrpy[1]: rotation acceleration in y-axis
				- ddrpy[2]: rotation acceleration in x-axis        
				@outputs:
				--------
				- dw: angular acceleration				
				*/

				Matrix_d_3x3 E0, E1;
				E0 << 0, -sin(rpy[0]), cos(rpy[0])*cos(rpy[1]),
					  0,  cos(rpy[0]), sin(rpy[0])*cos(rpy[1]),
					  1,         0,          -sin(rpy[1]);

				E1 << 0, -cos(rpy[0])*drpy[0], -sin(rpy[0])*drpy[0]*cos(rpy[1])-cos(rpy[0])*sin(rpy[1])*drpy[1],
					  0, -sin(rpy[0])*drpy[0],  cos(rpy[0])*drpy[0]*cos(rpy[1])-sin(rpy[0])*sin(rpy[1])*drpy[1],
					  0,         0,            -cos(rpy[1])*drpy[1];

				return E1*drpy + E0*ddrpy;  
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
				Matrix_d_3x3 R_e = R_med.transpose()*R_des;
				Orientation ori_e;
				ori_e = rot2axisAngle(R_e);

				return R_med*(ori_e.angle*ori_e.axis);
			}

			Matrix_d_6x6 compute_damped_pseudo_inv(Matrix_d_6x6 M)
			{
				double _lambda=0.0000001;
				Matrix_d_6x6 I = Matrix_d_6x6::Identity();
				return M.transpose()*((M*M.transpose() + _lambda*I).inverse());
			}			

    };

	// export control library (very important!)
	PLUGINLIB_EXPORT_CLASS(effort_controllers_ns::CartesianAdaptiveController, controller_interface::ControllerBase);
}    