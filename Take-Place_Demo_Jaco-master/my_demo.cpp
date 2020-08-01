# include <my_demo.h>

static tf::Vector3 homeLoc = tf::Vector3(0.2, -0.2, 0.5);
static tf::Vector3 retreatLoc = tf::Vector3(0.4, 0.2, 0.5);
static tf::Quaternion Orientation = shaftAngle2Quaternion(1,0,0,1.57);
static tf::Quaternion retreatOri = shaftAngle2Quaternion(1,0,0,3.14);
static std::vector<double> gripperOpen = {0.0, 0.0, 0.0};
static std::vector<double> gripperGrasp = {0.65, 0.65, 0.6};
static std::vector<double> gripperPinch = {1.2, 1.2, 1.2};



pickPlaceDemo::pickPlaceDemo(ros::NodeHandle &nh){
  
  nh_ = nh;
  ros::NodeHandle pn("~");
  sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/j2n6s300_driver/out/joint_state", 1, &pickPlaceDemo::get_current_state, this);
  sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/j2n6s300_driver/out/tool_pose", 1,  &pickPlaceDemo::get_current_pose, this);

  arm = new moveit::planning_interface::MoveGroupInterface("arm");
  gripper = new moveit::planning_interface::MoveGroupInterface("gripper");
  arm->setEndEffectorLink("j2n6s300_end_effector");


    bool p0 = home();
    if(p0){bool p1 = pre_grasp();
    if(p1){bool p2 = grasp_pose();
    if(p2){bool p3 = grasp();
    if(p3){bool p4 = retreat();
    if(p4){bool p5 = release();
    if(p5){bool p6 = home();
    }}}}}}


}

bool pickPlaceDemo::pre_grasp(){
    

    geometry_msgs::Pose current_pose;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_pose_);
        current_pose = current_pose_.pose;
    }
    
    std::vector<double> curLoc = {current_pose.position.x, current_pose.position.y, current_pose.position.z};
    tf::Quaternion curOri = tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    
    
    
    tf::Vector3 goalLoc = tf::Vector3(curLoc[0], curLoc[1]-0.2, curLoc[2]-0.4);
    tf::Pose P;
    P.setOrigin(goalLoc);
    P.setRotation(curOri);
    geometry_msgs::Pose p;
    tf::poseTFToMsg(P, p);
    arm->setPoseTarget(p);
    bool success = (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    ROS_INFO("Pre_grasp %s",success?"SUCCEED":"FAILED"); 
    return success;
}

bool pickPlaceDemo::grasp_pose(){
    geometry_msgs::Pose current_pose;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_pose_);
        current_pose = current_pose_.pose;
    }
    
    std::vector<double> curLoc = {current_pose.position.x, current_pose.position.y, current_pose.position.z};
    tf::Quaternion curOri = tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    
    
    
    tf::Vector3 goalLoc = tf::Vector3(curLoc[0], curLoc[1]-0.1, curLoc[2]);
    tf::Pose P;
    P.setOrigin(goalLoc);
    P.setRotation(curOri);
    geometry_msgs::Pose p;
    tf::poseTFToMsg(P, p);
    arm->setPoseTarget(p);
    bool success = (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    ROS_INFO("Grasp_pose %s",success?"SUCCEED":"FAILED"); 
    return success;
}

bool pickPlaceDemo::grasp(){
  
  gripper->setJointValueTarget(gripperGrasp);
  bool success = (gripper->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
  ROS_INFO("Grasp %s",success?"SUCCEED":"FAILED"); 
  return success;
}

bool pickPlaceDemo::retreat(){

    geometry_msgs::Pose current_pose;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_pose_);
        current_pose = current_pose_.pose;
    }

    std::vector<double> curLoc = {current_pose.position.x, current_pose.position.y, current_pose.position.z};
    tf::Quaternion curOri = tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

    tf::Vector3 goalLoc = tf::Vector3(curLoc[0], curLoc[1], curLoc[2]+0.2);
      

      
      tf::Pose P;
      P.setOrigin(goalLoc);
      P.setRotation(curOri);
      geometry_msgs::Pose p;
      tf::poseTFToMsg(P, p);
      arm->setPoseTarget(p);
      bool success = (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);


    
    sensor_msgs::JointState current_joint;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_state_);
        current_joint = current_state_;
    }

    std::vector<double> jointValue = {current_joint.position[0]+1.57, current_joint.position[1], current_joint.position[2], current_joint.position[3], current_joint.position[4], current_joint.position[5]};
    




    arm->setJointValueTarget(jointValue);
    success = success && (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if(success){

      
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_pose_);
        current_pose = current_pose_.pose;
    }
    
    curLoc = {current_pose.position.x, current_pose.position.y, current_pose.position.z};
    curOri = tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

    goalLoc = tf::Vector3(curLoc[0]-0.2, curLoc[1], curLoc[2]+0.1);
      

      
      tf::Pose P;
      P.setOrigin(goalLoc);
      P.setRotation(curOri);
      geometry_msgs::Pose p;
      tf::poseTFToMsg(P, p);
      arm->setPoseTarget(p);
      success = success && (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    }
    ROS_INFO("Retreat %s",success?"SUCCEED":"FAILED"); 
    return success;
}

bool pickPlaceDemo::release(){
  
  gripper->setJointValueTarget(gripperOpen);
  bool success = (gripper->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
  ROS_INFO("Release %s",success?"SUCCEED":"FAILED");
  
  return success;
}

bool pickPlaceDemo::home(){
    tf::Pose Home;
    Home.setOrigin(homeLoc);
    Home.setRotation(Orientation);
    geometry_msgs::Pose initialPose;
    tf::poseTFToMsg(Home, initialPose);
    arm->setPoseTarget(initialPose);
    bool success = true;
    success = (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if(success){
      gripper->setJointValueTarget(gripperOpen);
      success = success && (gripper->move() == moveit_msgs::MoveItErrorCodes::SUCCESS); 
    }
    ROS_INFO("Initialization %s",success?"SUCCEED":"FAILED");
    return success;
}

void pickPlaceDemo::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

void pickPlaceDemo::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}







twistDemo::twistDemo(ros::NodeHandle &nh){
  nh_ = nh;
  ros::NodeHandle pn("~");
  sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/j2n6s300_driver/out/joint_state", 1, &twistDemo::get_current_state, this);
  sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/j2n6s300_driver/out/tool_pose", 1,  &twistDemo::get_current_pose, this);

  arm = new moveit::planning_interface::MoveGroupInterface("arm");
  gripper = new moveit::planning_interface::MoveGroupInterface("gripper");
  arm->setEndEffectorLink("j2n6s300_end_effector");

  geometry_msgs::Pose current_pose;
    { // scope for mutex update
      boost::mutex::scoped_lock lock_pose(mutex_pose_);
      current_pose = current_pose_.pose;
    }

    std::cout << current_pose << std::endl;

    sensor_msgs::JointState current_joint;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_state_);
        current_joint = current_state_;
    }
    std::cout << current_joint << std::endl;

    tf::Quaternion Ori = tf::Quaternion(0.677066, 0.113773, -0.132932, 0.714819);
    tf::Vector3 Loc = tf::Vector3(0.022902, -0.870165, 0.226356);
    tf::Pose P;
    P.setOrigin(Loc);
    P.setRotation(Ori);
    geometry_msgs::Pose p;
    tf::poseTFToMsg(P, p);
    arm->setPoseTarget(p);
    arm->move();
    std::vector<double> g = {1.02625,1.02872,1.04473};
    gripper->setJointValueTarget(g);
    gripper->move();
    // bool p0 = home();
    // if(!p0){bool p1 = pre_twist();
    // if(p1){bool p2 = twist_pose();
    // if(p2){bool p3 = pinch();
    // if(p3){bool p4 = twist();
    // if(p4){bool p5 = retreat();
    // if(p5){bool p6 = home();
    // }}}}}}
    


}

void twistDemo::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

void twistDemo::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}

bool twistDemo::pre_twist(){
    geometry_msgs::Pose current_pose;
    { // scope for mutex update
      boost::mutex::scoped_lock lock_pose(mutex_pose_);
      current_pose = current_pose_.pose;
    }
    
    std::vector<double> curLoc = {current_pose.position.x, current_pose.position.y, current_pose.position.z};
    tf::Quaternion curOri = tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);



    tf::Vector3 goalLoc = tf::Vector3(curLoc[0], curLoc[1]-0.2, curLoc[2]-0.1);
    tf::Pose P;
    P.setOrigin(goalLoc);
    P.setRotation(curOri);
    geometry_msgs::Pose p;
    tf::poseTFToMsg(P, p);
    arm->setPoseTarget(p);
    bool success = (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    ROS_INFO("Pre_twist %s",success?"SUCCEED":"FAILED"); 
    return success;
}

bool twistDemo::twist_pose(){
    geometry_msgs::Pose current_pose;
    { // scope for mutex update
      boost::mutex::scoped_lock lock_pose(mutex_pose_);
      current_pose = current_pose_.pose;
    }
    
    std::vector<double> curLoc = {current_pose.position.x, current_pose.position.y, current_pose.position.z};
    tf::Quaternion curOri = tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

  
    tf::Vector3 goalLoc = tf::Vector3(curLoc[0], curLoc[1]-0.1, curLoc[2]);
    tf::Pose P;
    P.setOrigin(goalLoc);
    P.setRotation(curOri);
    geometry_msgs::Pose p;
    tf::poseTFToMsg(P, p);
    arm->setPoseTarget(p);
    bool success = (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    ROS_INFO("Twist_pose %s",success?"SUCCEED":"FAILED"); 
    return success;
}

bool twistDemo::pinch(){
  gripper->setJointValueTarget(gripperPinch);
  bool success = (gripper->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
  ROS_INFO("Pinch %s",success?"SUCCEED":"FAILED"); 
  return success;
}

bool twistDemo::twist(){
  geometry_msgs::Pose current_pose;
  { // scope for mutex update
    boost::mutex::scoped_lock lock_pose(mutex_pose_);
    current_pose = current_pose_.pose;
  }
  
  tf::Vector3 curLoc = tf::Vector3(current_pose.position.x, current_pose.position.y, current_pose.position.z);
  tf::Quaternion originOri = tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);


  bool success = true;
  int time = 2; // twist time
  for(int i=0; i<time; i+=1){

    sensor_msgs::JointState current_joint;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_state_);
        current_joint = current_state_;
    }

    std::vector<double> jointValue = {current_joint.position[0], current_joint.position[1], current_joint.position[2], current_joint.position[3], current_joint.position[4], current_joint.position[5]-1.0};
    

    arm->setJointValueTarget(jointValue);
    success = success && (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if(!success){
      break;
    }

    gripper->setJointValueTarget(gripperGrasp);
    success = success && (gripper->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if(!success){
      break;
    }
    
    tf::Pose P;
    P.setOrigin(curLoc);
    P.setRotation(originOri);
    geometry_msgs::Pose p;
    tf::poseTFToMsg(P, p);
    arm->setPoseTarget(p);
    success = success && (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if(!success){
      break;
    }

    if(i + 1 < time){
      gripper->setJointValueTarget(gripperPinch);
      success = success && (gripper->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
      if(!success){
        break;
      }
    }  
  }
  ROS_INFO("Twist %s",success?"SUCCEED":"FAILED"); 
  return success;
}

bool twistDemo::retreat(){
    
    gripper->setJointValueTarget(gripperOpen);
    bool success = (gripper->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if(success){
      geometry_msgs::Pose current_pose;
      { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_pose_);
        current_pose = current_pose_.pose;
      }
      
      std::vector<double> curLoc = {current_pose.position.x, current_pose.position.y, current_pose.position.z};
      tf::Quaternion curOri = tf::Quaternion(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

      tf::Vector3 goalLoc = tf::Vector3(curLoc[0], curLoc[1]+0.1, curLoc[2]);


      tf::Pose P;
      P.setOrigin(goalLoc);
      P.setRotation(curOri);
      geometry_msgs::Pose p;
      tf::poseTFToMsg(P, p);
      arm->setPoseTarget(p);
      success = success && (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    }
    ROS_INFO("Retreat %s",success?"SUCCEED":"FAILED");
    return success; 
}

bool twistDemo::home(){
    
    tf::Pose Home;
    Home.setOrigin(homeLoc);
    Home.setRotation(Orientation);
    geometry_msgs::Pose initialPose;
    tf::poseTFToMsg(Home, initialPose);
    arm->setPoseTarget(initialPose);
    bool success = true;
    success = (arm->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if(success){
      gripper->setJointValueTarget(gripperOpen);
      success = success && (gripper->move() == moveit_msgs::MoveItErrorCodes::SUCCESS); 
    }
    ROS_INFO("Initialization %s",success?"SUCCEED":"FAILED");
    return success;
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_demo");
  ros::NodeHandle node_handle; 
  


  ros::AsyncSpinner spinner(1);
  spinner.start();

  
  // pickPlaceDemo pickPlaceDemo(node_handle);
  twistDemo twistDemo(node_handle);

  
  return 0;

}


