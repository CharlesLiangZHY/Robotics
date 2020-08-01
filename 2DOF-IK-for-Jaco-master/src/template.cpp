#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_srvs/Empty.h"


#include <stdio.h>
#include <math.h>
#include <string.h>

double included_angle(double d1_x, double d1_y, double d2_x, double d2_y){
    double theta = acos((d1_x * d2_x + d1_y * d2_y) / (35.0*50.0) ); // radius is known
    return theta;
}


bool constrain(double x, double y){
    if(y > -20.0 && pow(x,2)+pow(y,2) >= 20.0*20.0 && pow(x,2)+pow(y,2) <= 85.0*85.0){
        return true;
    }else{
        std::cout << "Can not move in this direction." << std::endl;
        return false;
    }

}




class arm{
public:
	double arm_joint[6] = {3.14, 3.92, 5.48, 1.0, 2.0, 2.5};
	double gripper[3] = {0.0, 0.0, 0.0};
	double x_cur = 0.0;
	double y_cur = 36.0;
	double x_M_cur = 35.0;
	double y_M_cur = 36.0;
	double arm1_len = 50.0;
	double arm2_len = 35.0;



	void move_arm(double time, ros::Publisher pub){
		trajectory_msgs::JointTrajectory jointCmd;
		trajectory_msgs::JointTrajectoryPoint point;
		jointCmd.header.stamp = ros::Time::now() + ros::Duration(0.5);
		jointCmd.joint_names.resize(6);
		jointCmd.points.resize(1);
		point.time_from_start = ros::Duration(time);
		point.positions.resize(6);
		point.velocities.resize(6);
		point.accelerations.resize(6);
		point.effort.resize(6);
		for(int i=0;i<6;i++){
			jointCmd.joint_names[i] = "j2n6s300_joint_" + std::to_string(i+1);
			point.positions[i] = arm_joint[i];
			point.velocities[i] = 0.0;
			point.accelerations[i] = 0.0;
			point.effort[i] = 0.0;
		}
		jointCmd.points[0] = point;
		ros::Rate loop_rate(100);
		int count = 0;
		while(count < 40){
			pub.publish(jointCmd);
			// ros::spinOnce();
			count++;
			loop_rate.sleep();
		}
	}

	void move_finger(double time, ros::Publisher pub){
		trajectory_msgs::JointTrajectory jointCmd;
		trajectory_msgs::JointTrajectoryPoint point;
		jointCmd.header.stamp = ros::Time::now() + ros::Duration(0.5);
		jointCmd.joint_names.resize(3);
		jointCmd.points.resize(1);
		point.time_from_start = ros::Duration(time);
		point.positions.resize(3);
		point.velocities.resize(3);
		point.accelerations.resize(3);
		point.effort.resize(3);
		for(int i=0;i<3;i++){
			jointCmd.joint_names[i] = "j2n6s300_joint_finger_" + std::to_string(i+1);
			point.positions[i] = gripper[i];
			point.velocities[i] = 0.0;
			point.accelerations[i] = 0.0;
			point.effort[i] = 0.0;
		}
		jointCmd.points[0] = point;
		ros::Rate loop_rate(100);
		int count = 0;
		while(count < 40){
			pub.publish(jointCmd);
			// ros::spinOnce();
			count++;
			loop_rate.sleep();
		}
	}

	bool two_DOF_simple_IK(double x_G, double y_G){
		double a = pow(x_G/y_G , 2) + 1;
        double b = -(pow(arm1_len,2)-pow(arm2_len,2) + pow(x_G,2) + pow(y_G,2))*x_G/pow(y_G,2);
        double c = pow( (pow(arm1_len,2)-pow(arm2_len,2) + pow(x_G,2) + pow(y_G,2))/(2*y_G) , 2) - pow(arm1_len,2);

        double x_M1 = (-b + sqrt(pow(b,2) - 4.0*a*c))/(2*a);
        double y_M1 = (pow(arm1_len,2)-pow(arm2_len,2) + pow(x_G,2) + pow(y_G,2))/(2*y_G) - x_G/y_G * x_M1;

        double x_M2 = (-b - sqrt(pow(b,2) - 4.0*a*c))/(2*a);
        double y_M2 = (pow(arm1_len,2)-pow(arm2_len,2) + pow(x_G,2) + pow(y_G,2))/(2*y_G) - x_G/y_G * x_M2;

        if(pow(x_M1 - x_M_cur , 2) + pow(y_M1 - y_M_cur, 2) < pow(x_M2 - x_M_cur , 2) + pow(y_M2 - y_M_cur, 2)){
            x_M_cur = x_M1;
            y_M_cur = y_M1;
        }else{
            x_M_cur = x_M2;
            y_M_cur = y_M2;
        }
        if(y_M_cur <= -20){
        	std::cout << "Can not move in this direction." << std::endl;
        	return false;
        }else{
        	if(y_M_cur >= 0){
                arm_joint[1] = 4.71 - acos(x_M_cur/50.0);
            }else{
                if(x_M_cur >= 0){              
                    arm_joint[1] = 4.71 + acos(x_M_cur/50.0);
                }else{
                    arm_joint[1] = acos(x_M_cur/50.0) - 1.57;
                }
            }

            double d1_x = x_M_cur;
            double d1_y = y_M_cur;
            double d2_x = x_G - x_M_cur;
            double d2_y = y_G - y_M_cur;
            double theta = included_angle(d1_x, d1_y, d2_x, d2_y);

            double sina = d1_y / 50.0;
            double cosa = d1_x / 50.0;
            double sinb = d2_y / 35.0;
            double cosb = d2_x / 35.0;
            if(sinb * cosa - sina * cosb >= 0){
                arm_joint[2] = 3.14 + theta;
            }else{
                arm_joint[2] = 3.14 - theta;
            }
            x_cur = x_G;
            y_cur = y_G;
            return true;
        }


	}

	bool constrain(double x_G, double y_G){
		if(y_G > -20 && pow(x_G,2)+pow(y_G,2) >= pow(20.0,2) & pow(x_G,2)+pow(y_G,2) <= pow(arm1_len + arm2_len,2)){
			return true;
		}else{
			std::cout << "Can not move in this direction." << std::endl;
        	return false;
		}
	}

};


class Subscribe_And_Publish{
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	
public:
	ros::Publisher pub;
	ros::Publisher pub_finger;
	int key;
	Subscribe_And_Publish(){
		sub = n.subscribe<std_msgs::Int32> ("/Key", 1, &Subscribe_And_Publish::callback, this);
		// pub = n.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300_driver/effort_joint_trajectory_controller/command",1); // 1 is queue_size

        pub = n.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300_driver/trajectory_controller/command",1); // 1 is queue_size
        
		pub_finger = n.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300/effort_finger_trajectory_controller/command",1); // 1 is queue_size
		key = -1;	
	}

	void callback(const std_msgs::Int32::ConstPtr& msg){
		// std::cout << msg->data << std::endl;
		key = msg->data;
	}

};


int main(int argc, char **argv){
    ros::init(argc, argv, "template");

    Subscribe_And_Publish T;

    arm J;
    double x_G = 0.0;
    double y_G = 0.0;

    double step1 = 2.0; // move
    double step2 = 0.2; // rotatation
    double step3 = 0.25; // finger

    J.move_arm(5.0,T.pub); // move to home position
    J.move_finger(5.0, T.pub_finger);
    std::cout << "Moving to Home Position." << std::endl;
    ros::Rate r(2);
    while(ros::ok()){ 	
    	ros::spinOnce(); // execute callback once
    	if(T.key == 27){ // esc
    		std::cout << "Ending." << std::endl;
    		ros::shutdown();
    	}else if(T.key == 119){ // w
    		x_G = J.x_cur;
    		y_G = J.y_cur + step1;
    		if(J.constrain(x_G,y_G)){
    			if(J.two_DOF_simple_IK(x_G,y_G)){
    				J.move_arm(0.1, T.pub);
    				std::cout << "The end effecter is moving upward.Please wait." << std::endl;
    			}
    		}else{
    			std::cout << "Can not move in this direction." << std::endl;
    		}
    	}else if(T.key == 97){ // a
    		x_G = J.x_cur - step1;
    		y_G = J.y_cur;
    		if(J.constrain(x_G,y_G)){
    			if(J.two_DOF_simple_IK(x_G,y_G)){
    				J.move_arm(0.1, T.pub);
    				std::cout << "The end effecter is moving upward.Please wait." << std::endl;
    			}
    		}else{
    			std::cout << "Can not move in this direction." << std::endl;
    		}
    	}else if(T.key == 115){ // s
    		x_G = J.x_cur;
    		y_G = J.y_cur - step1;
    		if(J.constrain(x_G,y_G)){
    			if(J.two_DOF_simple_IK(x_G,y_G)){
    				J.move_arm(0.1, T.pub);
    				std::cout << "The end effecter is moving upward.Please wait." << std::endl;
    			}
    		}else{
    			std::cout << "Can not move in this direction." << std::endl;
    		}
    	}else if(T.key == 100){ // d
    		x_G = J.x_cur + step1;
    		y_G = J.y_cur;
    		if(J.constrain(x_G,y_G)){
    			if(J.two_DOF_simple_IK(x_G,y_G)){
    				J.move_arm(0.1, T.pub);
    				std::cout << "The end effecter is moving upward.Please wait." << std::endl;
    			}
    		}else{
    			std::cout << "Can not move in this direction." << std::endl;
    		}
    	}else if(T.key == 113){ // q
            if(J.arm_joint[0] - step2 >= -3.14){
                J.arm_joint[0] = J.arm_joint[0] - step2;
                J.move_arm(0.1, T.pub);
                std::cout << "The base is rotating counter-clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(T.key == 101){ // e
            if(J.arm_joint[0] + step2 <= 3.14){
                J.arm_joint[0] = J.arm_joint[0] + step2;
                J.move_arm(0.1, T.pub);
                std::cout << "The base is rotating clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(T.key == 122){ // z
            if(J.arm_joint[5] - step2 >= -3.14){
                J.arm_joint[5] = J.arm_joint[5] - step2;
                J.move_arm(0.1, T.pub);
                std::cout << "The end effector is rotating counter-clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(T.key == 120){ // x
            if(J.arm_joint[5] + step2 <= 3.14){
                J.arm_joint[5] = J.arm_joint[5] + step2;
                J.move_arm(0.1, T.pub);
                std::cout << "The end effector is rotating clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(T.key == 99){ // c
            if(J.arm_joint[3] - step2 >= -3.14){
                J.arm_joint[3] = J.arm_joint[3] - step2;
                J.move_arm(0.1, T.pub);
                std::cout << "The end effector is rotating counter-clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(T.key == 118){ // v
            if(J.arm_joint[3] + step2 <= 3.14){
                J.arm_joint[3] = J.arm_joint[3] + step2;
                J.move_arm(0.1, T.pub);
                std::cout << "The end effector is rotating clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(T.key == 98){ // b
            if(J.arm_joint[4] - step2 >= -3.14){
                J.arm_joint[4] = J.arm_joint[4] - step2;
                J.move_arm(0.1, T.pub);
                std::cout << "The end effector is rotating counter-clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(T.key == 110){ // n
            if(J.arm_joint[4] + step2 <= 3.14){
                J.arm_joint[4] = J.arm_joint[4] + step2;
                J.move_arm(0.1, T.pub);
                std::cout << "The end effector is rotating clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(T.key == 104){ // h
        	J.arm_joint[3] = 1.0;
        	J.arm_joint[4] = 2.0;
        	J.move_arm(0.1, T.pub);
        	std::cout << "Reseting joint4 and joint5." << std::endl;
        }else if(T.key == 111){ // o
        	if(J.gripper[0] - step3 >= 0){
                J.gripper[0] = J.gripper[0] - step3;
                J.gripper[1] = J.gripper[1] - step3;
                J.gripper[2] = J.gripper[2] - step3;
                J.move_finger(1.0, T.pub_finger);
                std::cout << "Gripper is opening. Please wait." << std::endl;
            }else{
                std::cout << "Can not open more." << std::endl;
            }
        }else if(T.key == 112){ // p
        	if(J.gripper[0] + step3 <= 1.5){
                J.gripper[0] = J.gripper[0] + step3;
                J.gripper[1] = J.gripper[1] + step3;
                J.gripper[2] = J.gripper[2] + step3;
                J.move_finger(1.0, T.pub_finger);
                std::cout << "Gripper is closing. Please wait." << std::endl;
            }else{
                std::cout << "Can not close more." << std::endl;
            }
        }




    	T.key = 0;

    r.sleep();
    }




return 0;
}