#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_srvs/Empty.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

int getch(){
    static struct termios oldt, newt;
    // newt.c_cc[VMIN] = 0;
    // newt.c_cc[VTIME] = 0;
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    // system("stty -echo");
    int c = getchar();
    // system("stty echo");
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}


double included_angle(double d1_x, double d1_y, double d2_x, double d2_y){
    double theta = acos((d1_x * d2_x + d1_y * d2_y) / (35.0*50.0) ); // radius is known
    return theta;
}

double angle(double x, double y){
    double d = sqrt(pow(x,2) + pow(y,2));
    double theta;
    if(y > 0){
        theta = acos(x/d);
    }else if(y < 0){
        theta = 2.0 * 3.14 - acos(x/d);
    }else{ // y == 0
        if(x >= 0){
            theta = 0.0;
        }else{
            theta = 3.14;
        }
    }

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







void move_joint(double joint_state[6], ros::Publisher pub, double time){
	trajectory_msgs::JointTrajectory jointCmd;
	trajectory_msgs::JointTrajectoryPoint point;
	jointCmd.header.stamp = ros::Time::now() + ros::Duration(0.0);
	jointCmd.joint_names.resize(6);
	jointCmd.points.resize(1);
	point.time_from_start = ros::Duration(time);
	point.positions.resize(6);
	point.velocities.resize(6);
	point.accelerations.resize(6);
	point.effort.resize(6);
	for(int i=0;i<6;i++){
		jointCmd.joint_names[i] = "j2n6s300_joint_" + std::to_string(i+1);
		point.positions[i] = joint_state[i];
		point.velocities[i] = 0.0;
		point.accelerations[i] = 0.0;
		point.effort[i] = 0.0;
	}
	jointCmd.points[0] = point;
	ros::Rate loop_rate(100);
	int count = 0;
	while(count < 40){
		pub.publish(jointCmd);
		ros::spinOnce();
		count++;
		loop_rate.sleep();
	}

}

void move_finger(double joint_state[3], ros::Publisher pub, double time){
	trajectory_msgs::JointTrajectory jointCmd;
	trajectory_msgs::JointTrajectoryPoint point;
	jointCmd.header.stamp = ros::Time::now() + ros::Duration(0.0);
	jointCmd.joint_names.resize(3);
	jointCmd.points.resize(1);
	point.time_from_start = ros::Duration(time);
	point.positions.resize(3);
	point.velocities.resize(3);
	point.accelerations.resize(3);
	point.effort.resize(3);
	for(int i=0;i<3;i++){
		jointCmd.joint_names[i] = "j2n6s300_joint_finger_" + std::to_string(i+1);
		point.positions[i] = joint_state[i];
		point.velocities[i] = 0.0;
		point.accelerations[i] = 0.0;
		point.effort[i] = 0.0;
	}
	jointCmd.points[0] = point;
	ros::Rate loop_rate(100);
	int count = 0;
	while(count < 40){
		pub.publish(jointCmd);
		ros::spinOnce();
		count++;
		loop_rate.sleep();
	}
}






int main(int argc, char **argv)
{
ros::init(argc, argv, "two_DOF_IK_teleop_key");
ros::NodeHandle n;
ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300/effort_joint_trajectory_controller/command",1); // 1 is queue_size
ros::Publisher pub_finger = n.advertise<trajectory_msgs::JointTrajectory>("/j2n6s300/effort_finger_trajectory_controller/command",1); // 1 is queue_size

int order_type = 0; // 0 no order; 1 move; 2 rotate root; 3 rotate hand

double x_cur = 0.0;
double y_cur = 36.0;
double x_M_cur = 35.0;
double y_M_cur = 36.0;
double x_G = 0.0;
double y_G = 0.0;


double j1 = 3.14;
double j2 = 3.92;
double j3 = 5.48;
double j4 = 1.0;
double j5 = 2.0;
double j6 = 2.5;
double f1 = 0.0;

double f2 = 0.0;

double f3 = 0.0;


system("stty -echo");

// home state
double js[6] = {j1,j2,j3,j4,j5,j6};
move_joint(js,pub,5.0);
double fg_js[3] = {f1,f2,f3};
move_finger(fg_js,pub_finger,5.0);

ros::Rate loop_rate(2);
    while(ros::ok()){
        int c = getch();
        // std::cout << c << std::endl;

        if(c == 27){ // esc
            ros::shutdown();
        }
        else if(c == 119){ // w
            x_G = x_cur;
            y_G = y_cur + 1;
            if(constrain(x_G, y_G)){
                std::cout << "The end effecter is moving upward.Please wait." << std::endl;
                order_type = 1;

            }else{
            	std::cout << "Can not move in this direction." << std::endl;
            }

        }else if(c == 97){ // a
            x_G = x_cur - 1;
            y_G = y_cur;
            if(constrain(x_G, y_G)){
                std::cout << "The end effecter is moving left. Please wait." << std::endl;
                order_type = 1;

            }else{
            	std::cout << "Can not move in this direction." << std::endl;
            }
        }else if(c == 115){ // s
            x_G = x_cur;
            y_G = y_cur - 1;
            if(constrain(x_G, y_G)){
                std::cout << "The end effecter is moving downward. Please wait." << std::endl;
                order_type = 1;

            }else{
            	std::cout << "Can not move in this direction." << std::endl;
            }
        }else if(c == 100){ // d
            x_G = x_cur + 1;
            y_G = y_cur;
            if(constrain(x_G, y_G)){
                std::cout << "The end effecter is moving right. Please wait." << std::endl;
                order_type = 1;

            }else{
            	std::cout << "Can not move in this direction." << std::endl;
            }
        }else if(c == 113){ // q
            if(j1 - 0.1 >= -3.14){
                j1 = j1 - 0.1;
                order_type = 2;
                std::cout << "The base is rotating counter-clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(c == 101){ // e
            if(j1 + 0.1 <= 3.14){
                j1 = j1 + 0.1;
                order_type = 2;
                std::cout << "The base is rotating clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(c == 122){ // z
            if(j6 - 0.1 >= -3.14){
                j6 = j6 - 0.1;
                order_type = 3;
                std::cout << "The end effector is rotating counter-clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }


        }else if(c == 120){ // x
            if(j4 + 0.1 <= 3.14){
                j6 = j6 + 0.1;
                order_type = 3;
                std::cout << "The end effector is rotating clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(c == 99){ // c
            if(j4 - 0.1 >= -3.14){
                j4 = j4 - 0.1;
                order_type = 3;
                std::cout << "The end effector is rotating counter-clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(c == 118){ // v
            if(j4 + 0.1 <= 3.14){
                j4 = j4 + 0.1;
                order_type = 3;
                std::cout << "The end effector is rotating clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(c == 98){ // b
            if(j5 - 0.1 >= -3.14){
                j5 = j5 - 0.1;
                order_type = 3;
                std::cout << "The end effector is rotating counter-clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(c == 110){ // n
            if(j5 + 0.1 <= 3.14){
                j5 = j5 + 0.1;
                order_type = 3;
                std::cout << "The end effector is rotating clockwise. Please wait." << std::endl;
            }else{
                std::cout << "Can not rotate in this direction." << std::endl;
            }
        }else if(c == 104){ // h
            j4 = 1.0;
            j5 = 2.0;
            std::cout << "Reset joint 4 and joint 5" << std::endl;
            order_type = 3;
        }else if(c == 111){ // o
            if(f1 - 0.25 >= 0){
                f1 = f1 - 0.25;
                f2 = f2 - 0.25;
                f3 = f3 - 0.25;
                order_type = 4;
                std::cout << "Gripper is opening. Please wait." << std::endl;
            }else{
                std::cout << "Can not open more." << std::endl;
            }
        }else if(c == 112){ // p
            if(f1 + 0.25 <= 1.5){
                f1 = f1 + 0.25;
                f2 = f2 + 0.25;
                f3 = f3 + 0.25;
                order_type = 4;
                std::cout << "Gripper is closing. Please wait." << std::endl;
            }else{
                std::cout << "Can not close more." << std::endl;
            }
        }

        if(order_type == 1){
            double a = pow(x_G/y_G , 2) + 1;
            double b = -(1275 + pow(x_G,2) + pow(y_G,2))*x_G/pow(y_G,2);
            double c = pow( (1275 + pow(x_G,2) + pow(y_G,2))/(2*y_G) , 2) - 50.0*50.0;

            double x_M1 = (-b + sqrt(pow(b,2) - 4.0*a*c))/(2*a);
            double y_M1 = (1275 + pow(x_G,2) + pow(y_G,2))/(2*y_G) - x_G/y_G * x_M1;

            double x_M2 = (-b - sqrt(pow(b,2) - 4.0*a*c))/(2*a);
            double y_M2 = (1275 + pow(x_G,2) + pow(y_G,2))/(2*y_G) - x_G/y_G * x_M2;

            double x_M;
            double y_M;

            if(pow(x_M1 - x_M_cur , 2) + pow(y_M1 - y_M_cur, 2) < pow(x_M2 - x_M_cur , 2) + pow(y_M2 - y_M_cur, 2)){
                x_M = x_M1;
                y_M = y_M1;
            }else{
                x_M = x_M2;
                y_M = y_M2;
            }

            x_M_cur = x_M;
            y_M_cur = y_M;


            if(y_M <= -20){
            	std::cout << "Can not move in this direction." << std::endl;
            }else{
            	if(y_M >= 0){
	                j2 = 4.71 - acos(x_M/50.0);
	                // std::cout << j2 << std::endl;
	            }else{
	                if(x_M >= 0){
	                    
	                    j2 = 4.71 + acos(x_M/50.0);
	                }else{
	                    j2 = acos(x_M/50.0) - 1.57;
	                }
	            }
	            double d1_x = x_M;
	            double d1_y = y_M;
	            double d2_x = x_G - x_M;
	            double d2_y = y_G - y_M;
	            double theta = included_angle(d1_x, d1_y, d2_x, d2_y);

	            // double alpha = angle(d1_x, d1_y);
	            // double beta  = angle(d2_x, d2_y);

	            double sina = d1_y / 50.0;
	            double cosa = d1_x / 50.0;
	            double sinb = d2_y / 35.0;
	            double cosb = d2_x / 35.0;
	            if(sinb * cosa - sina * cosb >= 0){
	                j3 = 3.14 + theta;
	            }else{
	                j3 = 3.14 - theta;
	            }
	            

	            x_cur = x_G;
	            y_cur = y_G;



            }

            // std::cout << "xM: " << x_M << "yM: " << y_M << std::endl; 
            // std::cout << "xG: " << x_G << "yG: " << y_G << std::endl; 

	            



        }

        if(order_type == 4){
        	double new_js[3] = {f1,f2,f3};
        	move_finger(new_js,pub_finger,1.0);

        }else{
        	double new_js[6] = {j1,j2,j3,j4,j5,j6};
			move_joint(new_js,pub,0.1);
        }



        order_type = 0;

    loop_rate.sleep();
    }


system("stty echo");

return 0;
}