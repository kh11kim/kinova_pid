#include "gen3.h"
#include "key.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

class Gen3Node: public Gen3{
private:
  ros::NodeHandle n;
  ros::Publisher joint_state_pub;
  ros::Publisher robot_state_pub;
  ros::Subscriber robot_config_sub;
  ros::Subscriber trajectory_sub;

  std::string joint_names[7];
  sensor_msgs::JointState joint_state_msg;
  std_msgs::Float64MultiArray robot_state_msg;

public:
  Gen3Node(ros::NodeHandle& n, const std::string& ip, const Model& model, Data& data);
  void PublishJointState();
  void PublishRobotState();
  void CallbackRobotConfig(const std_msgs::Float64MultiArray& gain);
  void CallbackTrajectory(const trajectory_msgs::JointTrajectory& traj);
  void GetWaypointFromJointTrajPointMsg(const trajectory_msgs::JointTrajectoryPoint& msg, Waypoint& w);
};

Gen3Node::Gen3Node(ros::NodeHandle& n, const std::string& ip, const Model& model, Data& data)
:n(n), Gen3::Gen3(ip, model, data)
{
  // Initialize pub, sub
  joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_state", 1);
  robot_state_pub = n.advertise<std_msgs::Float64MultiArray>("/robot_state", 1);
  robot_config_sub = n.subscribe("/robot_config", 10, &Gen3Node::CallbackRobotConfig, this);
  trajectory_sub = n.subscribe("/joint_traj", 1, &Gen3Node::CallbackTrajectory, this);
  
  // Initialize message
  joint_state_msg.name.resize(7);
  joint_state_msg.position.resize(7);
  joint_state_msg.velocity.resize(7);
  joint_state_msg.effort.resize(7);
  robot_state_msg.data.resize(23);

  std::string joint_ = "joint_";
  std::string joint_name;
  for (int i=0; i<dof; i++){
    joint_name = joint_ + std::to_string(i);
    joint_state_msg.name[i] = joint_name;
  }
}

void Gen3Node::PublishJointState(){
  joint_state_msg.header.stamp = ros::Time::now();
  //joint_position
  pthread_mutex_lock(&state.mutex);
  for (int i=0; i<dof; i++){
    joint_state_msg.position[i] = state.q[i];
    joint_state_msg.velocity[i] = state.dq[i];
    joint_state_msg.effort[i] = state.tau_J[i];
  }
  pthread_mutex_unlock(&state.mutex);
  joint_state_pub.publish(joint_state_msg);
}

void Gen3Node::PublishRobotState(){
  //total 23
  //posctrlmode 1 istrajfollowing 1 Kp(7) Ki(7) gamma(7)
  pthread_mutex_lock(&ctrlvar.mutex);
  robot_state_msg.data[0] = (double)ctrlvar.pos_ctrl_mode;
  robot_state_msg.data[1] = ctrlvar.is_traj_following;
  int offset = 2;
  for(int i=0;i<7;i++){
    robot_state_msg.data[offset+i] = ctrlvar.kp[i];
    robot_state_msg.data[offset+i+7] = ctrlvar.ki[i];
    robot_state_msg.data[offset+i+14] = ctrlvar.gamma[i];
  }
  pthread_mutex_unlock(&ctrlvar.mutex);
  robot_state_pub.publish(robot_state_msg);
}

void Gen3Node::CallbackRobotConfig(const std_msgs::Float64MultiArray& config){
  int config_type = (int)config.data[0];
  int offset = 1;
  Vector7d gain;
  switch(config_type){
    case 0: //Set kp
    {
      for(int i=0;i<7;i++){
        gain[i] = config.data[i+offset];
      }
      pthread_mutex_lock(&ctrlvar.mutex);
      ctrlvar.kp = gain;
      pthread_mutex_unlock(&ctrlvar.mutex);
      std::cout << "Set kp: " << gain.transpose() << std::endl;
      break;
    }
    case 1: //Set ki
    {
      for(int i=0;i<7;i++){
        gain[i] = config.data[i+offset];
      }
      pthread_mutex_lock(&ctrlvar.mutex);
      ctrlvar.ki = gain;
      pthread_mutex_unlock(&ctrlvar.mutex);
      std::cout << "Set ki: " << gain.transpose() << std::endl;
      break;
    }
    case 2: //Set gamma
    {
      for(int i=0;i<7;i++){
        gain[i] = config.data[i+offset];
      }
      pthread_mutex_lock(&ctrlvar.mutex);
      ctrlvar.gamma = gain;
      pthread_mutex_unlock(&ctrlvar.mutex);
      std::cout << "Set gamma: " << gain.transpose() << std::endl;
      break;
    }
    case 3: //Set pos_ctrl_mode
    {
      int mode = config.data[offset];
      if ((mode == NORMAL_PID)||(mode == GRAVITY_COMPENSATION_ONLY))
      {
        SetPositionCtrlMode(mode);  
      }
      std::cout << "Set pos_ctrl_mode: " << mode << std::endl;
      break;
    }
  }
}

void Gen3Node::GetWaypointFromJointTrajPointMsg(const trajectory_msgs::JointTrajectoryPoint& msg, Waypoint& w){
  for(int i=0;i<dof;i++){
    w.q[i] = msg.positions[i];
    w.dq[i] = msg.velocities[i];
  }
  w.time_from_start = msg.time_from_start.toSec();
}
void Gen3Node::CallbackTrajectory(const trajectory_msgs::JointTrajectory& traj){
  ROS_INFO("waypoint in");

  pthread_mutex_lock(&ctrlvar.mutex);
  ctrlvar.waypoints.clear();
  pthread_mutex_unlock(&ctrlvar.mutex);

  for(const auto& traj_point : traj.points){
    Waypoint w;
    GetWaypointFromJointTrajPointMsg(traj_point, w);
    pthread_mutex_lock(&ctrlvar.mutex);
    ctrlvar.waypoints.push_back(w);
    pthread_mutex_unlock(&ctrlvar.mutex);
    std::cout << w.q.transpose() << std::endl;
  }
  
  SetPositionCtrlMode(NORMAL_PID); //for test
  //turn on traj mode
  ExecuteTrajectory(ctrlvar.waypoints);
}

bool keyboard_interface(Gen3Node& robot){
  if(kbhit()){
    int keyPressed = getchar();
    switch(keyPressed){
      case '1':
        std::cout << "---Normal PID Mode---" << std::endl;
        robot.SetPositionCtrlMode(NORMAL_PID);
      break;
      case '2':
        std::cout << "---Gravity Compensation Mode---" << std::endl;
        robot.SetPositionCtrlMode(GRAVITY_COMPENSATION_ONLY);
      break;
      case '3':
        // std::cout << "---Set current joints as q1---" << std::endl;
        // q1 = robot.state.q;
        // std::cout << q1.transpose() << std::endl;
      break;
      case '4':
        // std::cout << "---Set current joints as q2---" << std::endl;
        // q2 = robot.state.q;
        // std::cout << q2.transpose() << std::endl;
      break;
      case '5':
        // robot.SetPositionCtrlMode(Gen3::NORMAL_PID);
        // traj_following = true;
        // std::cout << "---Trajectory Following q2->q1 ---" << std::endl;
        // w1.q = q1;
        // w2.q = q2;
        // w1.time_from_start = 7;
        // waypoints.push_back(w2);
        // waypoints.push_back(w1);
        // robot.ExecuteTrajectory(waypoints);
      break;
      case 'q':
        return false;
      break;
      default:
      break;
    } 
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gen3_node");
  ROS_INFO("Start gen3_node");
  ros::NodeHandle n;

  std::string ip = "172.16.0.8";
  Model model;
  pinocchio::urdf::buildModel(KINOVA_URDF_PATH, model);
  Data data(model);
  Gen3Node robot(n, ip, model, data);
  
  init_keyboard();
  
  robot.RunPositionCtrl();

  ros::Rate rate(100);
  int cnt10ms = 0;
  bool isRunning = true;
  while(ros::ok() && isRunning){
    robot.PublishJointState(); 
    robot.PublishRobotState();
    
    if(cnt10ms++>= 100){
      std::cout << robot.state.traj_time << robot.state.dq_d.transpose() << std::endl;  
      cnt10ms = 0;
    }
    

    isRunning = keyboard_interface(robot);
    
    ros::spinOnce();
    rate.sleep();
  }

  robot.StopPositionCtrl();

  std::cout << "Quit" << std::endl;
  close_keyboard();
  return 0;
}