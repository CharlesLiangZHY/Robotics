#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <kinova_driver/kinova_ros_types.h>

#include <math.h>










class pickPlaceDemo{
public:
    pickPlaceDemo(ros::NodeHandle &nh);
    void get_current_state(const sensor_msgs::JointStateConstPtr &msg);
    void get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg);
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_joint_;
    
private:
    
    boost::mutex mutex_state_;
    boost::mutex mutex_pose_;
    sensor_msgs::JointState current_state_;
    geometry_msgs::PoseStamped current_pose_;

    moveit::planning_interface::MoveGroupInterface* arm;
    moveit::planning_interface::MoveGroupInterface* gripper;

    ros::NodeHandle nh_;




    bool pre_grasp();
    bool grasp_pose();
    bool grasp();
    bool retreat();
    bool release();
    bool home();

};


class twistDemo{
public:
    twistDemo(ros::NodeHandle &nh);
    
    void get_current_state(const sensor_msgs::JointStateConstPtr &msg);
    void get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg);
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_joint_;

private:
    boost::mutex mutex_state_;
    boost::mutex mutex_pose_;
    sensor_msgs::JointState current_state_;
    geometry_msgs::PoseStamped current_pose_;

    moveit::planning_interface::MoveGroupInterface* arm;
    moveit::planning_interface::MoveGroupInterface* gripper;

    ros::NodeHandle nh_;

    bool pre_twist();
    bool twist_pose();
    bool twist();
    bool pinch();
    bool retreat();
    bool home();
};

tf::Quaternion shaftAngle2Quaternion(double ax, double ay, double az, double theta){
    return tf::Quaternion(ax*sin(theta/2), ay*sin(theta/2), az*sin(theta/2), cos(theta/2));
}