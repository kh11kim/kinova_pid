#pragma once

#include "core.h"
#include "filter.h"
#include "spline.h"

#define KINOVA_URDF_PATH "/home/pandacom/ws/kinova_joint_pid/robot_model/gen3_7dof/urdf/GEN3_URDF_V12_with_Hand_e_rev.urdf"
#define MSEC_ns 1000000
#define CTRL_DT 0.001
#define FILTER_CUTOFF 15
#define DOF 7
#define DESIRED_INERTIA_RATIO 0.7
#define PORT 10000
#define PORT_REAL_TIME 10001
#define Q_DIFF_THRESHOLD 0.1

//Kortex(kinova) API
#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/util/json_util.h>

//pinocchio
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

//Threading
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <errno.h>


inline double JointMap(double theta)
{
  if(theta >= M_PI){
    return theta - M_PI*2;
  }
  return theta;
}

using namespace Eigen;
using namespace pinocchio;


//Data Structure
enum CtrlMode{
  POSITION_CONTROL = 0,
  TORQUE_CONTROL,
  CURRENT_CONTROL,
};

enum PosCtrlMode{
  NORMAL_PID = 0,
  GRAVITY_COMPENSATION_ONLY,
};

struct RobotState
{
  pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
  Vector7d q;
  Vector7d dq;
  Vector7d q_d;
  Vector7d dq_d;
  Vector7d tau_J;
  double traj_time;
};

struct CtrlVar
{
  pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
  bool running = true;
  //state
  Vector7d error_sum; 
  Vector7d error_prev;
  //param
  int pos_ctrl_mode = NORMAL_PID;
  bool is_traj_following = false;
  std::vector<Waypoint> waypoints;
  Vector7d kp;
  Vector7d ki;
  Vector7d gamma;
  bool IsRunning(){
    bool result;
    //check loop terminate condition
    pthread_mutex_lock(&mutex);
    result = running;
    pthread_mutex_unlock(&mutex);
    return result;
  }
};

class Gen3{
private:
  // robot
  Model model;
  Data data;

  // kinova
  std::string ip;
  Kinova::Api::Base::BaseClient *base;
  Kinova::Api::BaseCyclic::BaseCyclicClient *base_cyclic;
  Kinova::Api::ActuatorConfig::ActuatorConfigClient *actuator_config;
  Kinova::Api::SessionManager* session_manager;
  Kinova::Api::SessionManager* session_manager_real_time;
  Kinova::Api::RouterClient* router;
  Kinova::Api::TransportClientTcp* transport;
  Kinova::Api::RouterClient* router_real_time;
  Kinova::Api::TransportClientUdp* transport_real_time;
  Kinova::Api::BaseCyclic::Feedback base_feedback;
  Kinova::Api::BaseCyclic::Command  base_command;
  int kinova_ctrl_mode;

  // filters
  LPF q_d_filter;
  LPF dq_d_filter;
  LPF tau_filter;

  // spline
  Spline sp;

  // Kinova API
  bool SetKinovaControlMode(int mode);
  bool SetCurrentInput(Vector7d current);
  bool SetKinovaServoingMode(Kinova::Api::Base::ServoingMode mode);

  // control API
  void UpdateState();
  
  // current control
  pthread_t rtThread;
  pthread_attr_t rtThreadAttr;
  void* CurrentCtrlLoop();
  static void* CurrentCtrlLoopHelper(void* context){
    return ((Gen3*)context)->CurrentCtrlLoop();
  }
  Vector7d CalculateControlInput();
  Vector7d ConvertTorqueToCurrent(Vector7d tau);
  Vector7d GetGravity(Vector7d q);
  Matrix7d rotor_inertia_matrix;
  Matrix7d rotor_inertia_map_matrix;

protected:
  int dof = 7;

public:
  RobotState state;
  CtrlVar ctrlvar;

  Gen3(std::string ip, const Model& model, Data& data);
  void InitPositionCtrlVar();
  void RunPositionCtrl();
  void StopPositionCtrl();
  void SetPositionCtrlMode(int mode);
  void ExecuteTrajectory(const std::vector<Waypoint>& waypoints);
};
