
#include "gen3.h"
//#include "key.h"


Gen3::Gen3(std::string ip, const Model& model, Data& data)
  :ip(ip), q_d_filter(DOF, CTRL_DT, FILTER_CUTOFF), dq_d_filter(DOF, CTRL_DT, FILTER_CUTOFF), tau_filter(DOF, CTRL_DT, FILTER_CUTOFF)
{
  this->model = model;
  this->data = data;
  Vector7d rotor_inertia_vector;
  rotor_inertia_vector << 0.4, 0.4, 0.4, 0.4, 0.2, 0.2, 0.2;
  rotor_inertia_matrix.setZero();
  rotor_inertia_matrix.diagonal() = rotor_inertia_vector;
  Matrix7d  rotor_inertia_matrix_d;
  rotor_inertia_matrix_d = rotor_inertia_matrix * DESIRED_INERTIA_RATIO;
  rotor_inertia_map_matrix = rotor_inertia_matrix*rotor_inertia_matrix_d.inverse();

  //initialize Gen3
  auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };

  std::cout << "[Constructor] Creating transport objects" << std::endl;
  transport = new Kinova::Api::TransportClientTcp();
  router = new Kinova::Api::RouterClient(transport, error_callback);
  transport->connect(ip, PORT);

  std::cout << "[Constructor] Creating transport real time objects" << std::endl;
  transport_real_time = new Kinova::Api::TransportClientUdp();
  router_real_time = new Kinova::Api::RouterClient(transport_real_time, error_callback);
  transport_real_time->connect(ip, PORT_REAL_TIME);

  // Set session data connection information
  auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
  create_session_info.set_username("admin");
  create_session_info.set_password("admin");
  create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

  // Session manager service wrapper
  std::cout << "[Constructor] Creating sessions for communication" << std::endl;
  session_manager = new Kinova::Api::SessionManager(router);
  session_manager->CreateSession(create_session_info);
  session_manager_real_time = new Kinova::Api::SessionManager(router_real_time);
  session_manager_real_time->CreateSession(create_session_info);
  std::cout << "[Constructor] Sessions created" << std::endl;

  //Create Services
  base = new Kinova::Api::Base::BaseClient(router);
  base_cyclic = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_real_time);
  actuator_config = new Kinova::Api::ActuatorConfig::ActuatorConfigClient(router);
  
  bool success = true;
  // std::cout<<"[Constructor] homing"<<std::endl;
  // success &= Homing("Zero");
  success &= SetKinovaServoingMode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  // success &= InitializeControl(CURRENT_CONTROL);
  
  if(success)
  {  
    std::cout<<"[Constructor] Kinova Gen3 Initialization Success"<<std::endl;
  }
  else{
    std::cout<<"[Constructor] Initialization Failed"<<std::endl;
  }
}

bool Gen3::SetKinovaServoingMode(Kinova::Api::Base::ServoingMode mode)
{ 
  bool return_status = true;

  try{
    auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
    // Set the base in low-level servoing mode
    servoing_mode.set_servoing_mode(mode);
    base->SetServoingMode(servoing_mode);
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
    return_status = false;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
    return_status = false;
  }
  return return_status;
}

bool Gen3::SetKinovaControlMode(int mode){
  bool return_status = true;
  std::cout << "Set control mode" << mode << std::endl;
  // validate input
  if(kinova_ctrl_mode == mode){
    std::cout << "Already in the mode" << mode << std::endl;
    return true;
  }

  // Clearing faults
  try
  {
      base->ClearFaults();
  }
  catch(...)
  {
      std::cout << "Unable to clear robot faults" << std::endl;
      return false;
  }

  std::vector<float> commands;
  Kinova::Api::ActuatorConfig::ControlMode kinova_mode;
  try
  {
    base_feedback = base_cyclic->RefreshFeedback();
    std::cout << "Initialize current states" << std::endl;
    if(mode == CURRENT_CONTROL || mode == TORQUE_CONTROL){
      // Initialize each actuator to their current position
      for (int i = 0; i < dof; i++)
      {
        commands.push_back(base_feedback.actuators(i).position());
        base_command.add_actuators()->set_position(base_feedback.actuators(i).position());      
      }
      // Send a first frame
      base_feedback = base_cyclic->Refresh(base_command);
    }

    switch(mode){
      case POSITION_CONTROL:
        kinova_mode = Kinova::Api::ActuatorConfig::ControlMode::POSITION;
      break;
      case TORQUE_CONTROL:
        kinova_mode = Kinova::Api::ActuatorConfig::ControlMode::TORQUE;
      break;
      case CURRENT_CONTROL:
        kinova_mode = Kinova::Api::ActuatorConfig::ControlMode::CURRENT;
      break;
    }
    
    // Set all actuators in torque control mode.
    std::cout << "Change control mode" << std::endl;
    auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(kinova_mode);
    for (int i = 0; i < dof; i++)
    {
      actuator_config->SetControlMode(control_mode_message, i+1);
    }
    kinova_ctrl_mode = mode;
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
    std::cout << "API error: " << ex.what() << std::endl;
    return_status = false;
  }
  catch (std::runtime_error& ex2)
  {
    std::cout << "Error: " << ex2.what() << std::endl;
    return_status = false;
  }
  return return_status;
}

void Gen3::UpdateState(){
  Vector7d q, dq, q_d, dq_d, tau_J_raw, tau_J, theta, dtheta;
  base_feedback = base_cyclic->RefreshFeedback();
  for(int i=0; i<dof; i++)
  {
    theta(i) = (double)base_feedback.actuators(i).position()* M_PI/180; // motor position
    dtheta(i) = (double)base_feedback.actuators(i).velocity() * M_PI/180; // motor velocity
    tau_J_raw(i) = -(double)base_feedback.actuators(i).torque(); // joint torque sensor
  }

  q = theta;
  dq = dtheta;
  
  pthread_mutex_lock(&state.mutex);
  //update
  for(int i=0; i<dof; i++){
    state.q[i] = JointMap(q(i));
  }
  state.dq = dq;
  state.tau_J = tau_filter.Apply(tau_J_raw);
  
  //Trajectory following
  if(ctrlvar.is_traj_following == true){
    bool isSuccess = sp.GetSplineResult(state.traj_time, q_d, dq_d);
    if(!isSuccess){
      state.traj_time = 0.;
      ctrlvar.is_traj_following == false;
    }
    state.q_d = q_d;
    state.dq_d = dq_d;
    state.traj_time += CTRL_DT;
    if(state.traj_time >= sp.last_timestep){
      ctrlvar.is_traj_following == false;
    }
  }
  pthread_mutex_unlock(&state.mutex);
}

void* Gen3::CurrentCtrlLoop(){
  std::cout << "Position Control Start" << std::endl;

  Vector7d u, current, tau;
  while (ctrlvar.IsRunning()) {
    tau = CalculateControlInput();
    current = ConvertTorqueToCurrent(tau);
    SetCurrentInput(current);
    usleep(1000); //1ms
  }

  std::cout << "Position Control End" << std::endl;
  return NULL;
}

Vector7d Gen3::CalculateControlInput(){
  Vector7d u, tau, error, derror;
  pthread_mutex_lock(&ctrlvar.mutex);

  UpdateState();
  error = state.q - q_d_filter.Apply(state.q_d);
  derror = state.dq - dq_d_filter.Apply(state.dq_d);
  ctrlvar.error_sum = ctrlvar.error_sum + (error + ctrlvar.error_prev)/2 * CTRL_DT;
  
  u.setZero();
  if(ctrlvar.pos_ctrl_mode == NORMAL_PID)
  {
    for (int i=0;i<dof; i++){
      double kp = ctrlvar.kp[i];
      double ki = ctrlvar.ki[i];
      double gamma = ctrlvar.gamma[i];
      u[i] = - (0.001 + 1/(gamma*gamma)) * \
        (derror[i] + kp*error[i] + ki*ctrlvar.error_sum[i]);
    }
  }
  
  //Gravity compensation
  u += GetGravity(state.q);

  //Inertia shaping
  tau = rotor_inertia_map_matrix*u + state.tau_J
        -rotor_inertia_map_matrix*(state.tau_J);
  
  //Error update
  ctrlvar.error_prev = error;
  
  pthread_mutex_unlock(&ctrlvar.mutex);
  return tau;
}

Vector7d Gen3::ConvertTorqueToCurrent(Vector7d tau){
  Vector7d current(7);
  Vector7d current_limit(7);
  current_limit << 10, 10, 10, 10, 6, 6, 6;
      
  //To avoid Current warning: clipping
  for (int i=0; i<4; i++)
  {
    current[i] = tau[i] / 11.;
  }
  for (int i=4; i<7; i++)
  {
    current[i] = tau[i] / 7.6;
  }
  current = current.cwiseMin(current_limit).cwiseMax(-current_limit); //clip
  return current;
}

bool Gen3::SetCurrentInput(Vector7d current){
  bool return_status = true;

  //Input current.
  for(int i=0; i<dof; i++)
  {   
    //To prevent following error.. set position as current position..!
    base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
    //Set actuator current.
    base_command.mutable_actuators(i)->set_current_motor(current[i]);
  }


  // Incrementing identifier ensures actuators can reject out of time frames
  base_command.set_frame_id(base_command.frame_id() + 1);
  if (base_command.frame_id() > 65535)
      base_command.set_frame_id(0);

  for (int i = 0; i < dof; i++)
  {
      base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
  }

  try
  {
      base_feedback = base_cyclic->Refresh(base_command, 0);
  }
  catch (Kinova::Api::KDetailedException& ex)
  {
      std::cout << "Kortex exception: " << ex.what() << std::endl;

      std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
      return_status = false;
  }
  catch (std::runtime_error& ex2)
  {
      std::cout << "runtime error: " << ex2.what() << std::endl;
      return_status = false;
  }
  catch(...)
  {
      std::cout << "Unknown error." << std::endl;
      return_status = false;
  }

  return return_status;
}

Vector7d Gen3::GetGravity(Vector7d q){
  return pinocchio::computeGeneralizedGravity(this->model, this->data, q);
}

void Gen3::InitPositionCtrlVar(){
  //initialize variables
  pthread_mutex_lock(&ctrlvar.mutex);
  ctrlvar.error_sum.setZero();
  ctrlvar.error_prev.setZero();
  ctrlvar.kp = Vector7d::Constant(50.);
  ctrlvar.ki = (Vector7d() << 625,700,625,625,625,625,625).finished();
  ctrlvar.gamma = (Vector7d() << 0.25,0.1,0.25,0.25,0.25,0.25,0.25).finished();
  pthread_mutex_unlock(&ctrlvar.mutex);
  
  //Initialize desired state
  UpdateState();
  pthread_mutex_lock(&state.mutex);
  state.q_d = state.q;
  state.dq_d.setZero();
  pthread_mutex_unlock(&state.mutex);
  q_d_filter.Init(state.q);
  dq_d_filter.Init(state.dq);
}

void Gen3::RunPositionCtrl(){ 
  InitPositionCtrlVar();
  
  //initialize thread
  pthread_attr_init(&rtThreadAttr);
  pthread_attr_setschedpolicy(&rtThreadAttr, SCHED_FIFO); // Set thread scheduling policy to SCHED_FIFO (real-time)
  struct sched_param rtThreadParam; // Set the thread's priority
  rtThreadParam.sched_priority = 99;
  pthread_attr_setschedparam(&rtThreadAttr, &rtThreadParam);

  bool success = SetKinovaControlMode(CURRENT_CONTROL);
  if(!success){
    SetKinovaControlMode(POSITION_CONTROL);
    std::cout << "Failed to set current control mode!" << std::endl;
  }
  else{
    std::cout << "Set current control mode" << std::endl;
  }

  // Create the real-time thread
  if (pthread_create(&rtThread, &rtThreadAttr, &Gen3::CurrentCtrlLoopHelper, this) != 0) {
    std::cerr << "Failed to create real-time thread." << std::endl;
    return;
  }
}

void Gen3::StopPositionCtrl(){
  std::cerr << "Stopping Control..." << std::endl;
  pthread_mutex_lock(&ctrlvar.mutex);
  ctrlvar.running = false;
  pthread_mutex_unlock(&ctrlvar.mutex);
  pthread_join(rtThread, NULL);
  std::cerr << "Control Stopped!" << std::endl;
  SetKinovaControlMode(POSITION_CONTROL);
}

void Gen3::SetPositionCtrlMode(int mode){
  InitPositionCtrlVar();
  pthread_mutex_lock(&ctrlvar.mutex);
  ctrlvar.pos_ctrl_mode = mode;
  pthread_mutex_unlock(&ctrlvar.mutex);
}

void Gen3::ExecuteTrajectory(const std::vector<Waypoint>& waypoints){
  Vector7d q_curr;
  q_curr = state.q;
  double q_diff_norm = (q_curr - waypoints[0].q).norm();
  if(q_diff_norm > Q_DIFF_THRESHOLD){  
    std::cout << "Trajectory init point is far from current state!" << std::endl;
    return;
  }

  sp.SetWaypoints(waypoints);
  bool isSuccess = sp.CalculateParameters();
  if(isSuccess){
    // Initialize
    pthread_mutex_lock(&state.mutex);
    state.traj_time = 0;
    pthread_mutex_unlock(&state.mutex);
    // Set
    pthread_mutex_lock(&ctrlvar.mutex);
    ctrlvar.is_traj_following = true; // The flag turned off in UpdateState
    pthread_mutex_unlock(&ctrlvar.mutex);
  }
  else{
    std::cout << "Generating Spline failed!" << std::endl;
  }
}



// int main(){
//   std::string ip = "172.16.0.8";
//   Model model;
//   pinocchio::urdf::buildModel(KINOVA_URDF_PATH, model);
//   Data data(model);
//   Gen3::Gen3 robot(ip, model, data);
  
//   robot.RunPositionCtrl();
  
//   init_keyboard();
//   bool traj_following = false;
//   Vector7d q1, q2;
//   Waypoint w1, w2;
//   std::vector<Waypoint> waypoints;
//   // Main thread logic here (non-real-time tasks)
//   bool is_running = true;
//   while (is_running) {
//     usleep(10000);
    
//     if(kbhit()){
//       int keyPressed = getchar();
//       switch(keyPressed){
//         case '1':
//           std::cout << "---Normal PID Mode---" << std::endl;
//           robot.SetPositionCtrlMode(Gen3::NORMAL_PID);
//         break;
//         case '2':
//           std::cout << "---Gravity Compensation Mode---" << std::endl;
//           robot.SetPositionCtrlMode(Gen3::GRAVITY_COMPENSATION_ONLY);
//         break;
//         case '3':
//           std::cout << "---Set current joints as q1---" << std::endl;
//           q1 = robot.state.q;
//           std::cout << q1.transpose() << std::endl;
//         break;
//         case '4':
//           std::cout << "---Set current joints as q2---" << std::endl;
//           q2 = robot.state.q;
//           std::cout << q2.transpose() << std::endl;
//         break;
//         case '5':
//           robot.SetPositionCtrlMode(Gen3::NORMAL_PID);
//           traj_following = true;
//           std::cout << "---Trajectory Following q2->q1 ---" << std::endl;
//           w1.q = q1;
//           w2.q = q2;
//           w1.time_from_start = 7;
//           waypoints.push_back(w2);
//           waypoints.push_back(w1);
//           robot.ExecuteTrajectory(waypoints);
//         break;
//         case 'q':
//           is_running = false;
//         break;
//         default:
//         break;
//       } 
//     }
//     if(traj_following){
//       std::cout << robot.state.traj_time <<": "<< robot.state.q_d.transpose() << std::endl;
//     }
//   }
  
//   // Wait for the real-time thread to finish (if needed)
//   robot.StopPositionCtrl();
  
//   close_keyboard();
//   return 0;
// }