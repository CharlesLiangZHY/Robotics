#include "ros/ros.h"
#include "sensor_msgs/JointState.h"




int main(int argc, char **argv)
{
    ros::init(argc, argv, "Yeah");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("/joint_states",1);
    int t = 0;
    double j1 = 0;
    double j2 = 3.14;
    double j3 = 3.14;
    double j4 = 0;
    double j5 = 0;
    double j6 = 0;
    double f1 = 0;
    double ft1 = 0;
    double f2 = 0;
    double ft2 = 0;
    double f3 = 0;
    double ft3 = 0;


    ros::Rate loop_rate(10);
    while(ros::ok()){
        if(t <= 15){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};
            j2 += (3.84 - 3.14)/15;
            j3 += (5.14 - 3.14)/15;


            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);

            ros::spinOnce();
        }else if(t <= 20){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};
            j4 -= (1.56 - 0)/5;
            


            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);

            ros::spinOnce();
        }else if(t <= 25){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};
            j5 -= (0.75 - 0)/5;
            


            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);

            ros::spinOnce();
        }else if(t <= 35){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};
            j5 += (0.75 - (- 0.75) )/10;
            


            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);
            
            ros::spinOnce();
        }else if(t <= 40){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};
            j6 -= (0.5 - 0) /5;
            


            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);
            
            ros::spinOnce();
        }else if(t <= 50){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};
            j6 += (0.5 - (- 0.5) )/10;
            


            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);
            
            ros::spinOnce();
        }else if(t <= 55){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};
            j6 -= (0.5 - 0) /5;
            


            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);
            
            ros::spinOnce();
        }else if(t <= 75){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};
            f1 += (1.51 - 0)/20;
            ft1 += (2 - 0)/20;
            f2 += (1.51 - 0)/20;
            ft2 += (2 - 0)/20;
            f3 += (1.51 - 0)/20;
            ft3 += (2 - 0)/20;

            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);

            ros::spinOnce();
        }else if(t <= 85){
            sensor_msgs::JointState js;
            js.name = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5",
        "j2n6s300_joint_6", "j2n6s300_joint_finger_1", "j2n6s300_joint_finger_tip_1", "j2n6s300_joint_finger_2",
        "j2n6s300_joint_finger_tip_2", "j2n6s300_joint_finger_3", "j2n6s300_joint_finger_tip_3"};

            f2 -= (1.51 - 0) /10;
            ft2 -= (2 - 0)/10;
            f3 -= (1.51 - 0)/10;
            ft3 -= (2 - 0)/10;


            js.position = {j1, j2, j3, j4, j5, j6, f1, ft1, f2, ft2, f3, ft3};
            t = t+1;
            pub.publish(js);

            ros::spinOnce();
        }


    loop_rate.sleep();


    }

    
    return 0;

}