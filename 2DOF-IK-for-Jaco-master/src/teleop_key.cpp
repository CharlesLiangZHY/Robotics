#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>

int getch(){
    static struct termios oldt, newt;
    // newt.c_cc[VMIN] = 0;
    // newt.c_cc[VTIME] = 0;
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    system("stty -echo");
    int c = getchar();
    system("stty echo");
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "teleop_key");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Int32>("Key", 1000);
    ros::Rate loop_rate(2);
    while(ros::ok()){
        int c = getch();
        // std::cout << c << std::endl;
        std_msgs::Int32 msg;
        msg.data = c;
        pub.publish(msg);
        // ros::spinOnce();
        
    loop_rate.sleep();
    }

    return 0;

}

