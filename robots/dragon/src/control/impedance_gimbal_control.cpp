#include <dragon/control/impedance_gimbal_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

DragonImpedanceGimbalController::DragonImpedanceGimbalController():
  HydrusImpedanceController()
{
}

void DragonImpedanceGimbalController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  /* initialize the flight control */
  HydrusImpedanceController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  dragon_robot_model_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(robot_model);


  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller1/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller2/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller3/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller4/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller5/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller6/simulation/command", 1));
  target_joint_pos_[0] = 0.0;
  target_joint_pos_[1] = -1.57;
  target_joint_pos_[2] = 0.0;
  target_joint_pos_[3] = -1.57;
  target_joint_pos_[4] = 0.0;
  target_joint_pos_[5] = -1.57;
  pos_cmd_.x = -0.35;
  pos_cmd_.y = 0.35;
  pos_cmd_.z = 0.0;

  std_msgs::Float64 yaw_term, pitch_term;
  yaw_term.data = 2;
  pitch_term.data = 0.0;

  joint_cmd_pubs_[0].publish(pitch_term);
  joint_cmd_pubs_[1].publish(yaw_term);
  joint_cmd_pubs_[2].publish(pitch_term);
  joint_cmd_pubs_[3].publish(yaw_term);
  joint_cmd_pubs_[4].publish(pitch_term);
  joint_cmd_pubs_[5].publish(yaw_term);


  /* initialize the matrix */
  P_xy_ = Eigen::MatrixXd::Zero(2, motor_num_ * 2);
  for(int i = 0; i < motor_num_; i++)
    {
      P_xy_(0, i * 2) = 1;
      P_xy_(1, 1 + i * 2) = 1;
    }

  lqi_att_terms_.resize(motor_num_);

  /* initialize the gimbal target angles */
  target_gimbal_angles_.resize(motor_num_ * 2, 0);
  /* additional vectoring force for grasping */
  extra_vectoring_force_ = Eigen::VectorXd::Zero(3 * motor_num_);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbal_target_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/gimbals_target_force", 1);

  att_control_feedback_state_sub_ = nh_.subscribe("rpy/feedback_state", 1, &DragonImpedanceGimbalController::attControlFeedbackStateCallback, this);
  extra_vectoring_force_sub_ = nh_.subscribe("extra_vectoring_force", 1, &DragonImpedanceGimbalController::extraVectoringForceCallback, this);

  std::string service_name;
  nh_.param("apply_external_wrench", service_name, std::string("apply_external_wrench"));
  add_external_wrench_service_ = nh_.advertiseService(service_name, &DragonImpedanceGimbalController::addExternalWrenchCallback, this);
  nh_.param("clear_external_wrench", service_name, std::string("clear_external_wrench"));
  clear_external_wrench_service_ = nh_.advertiseService(service_name, &DragonImpedanceGimbalController::clearExternalWrenchCallback, this);

  //message
  pid_msg_.yaw.total.resize(1);
  pid_msg_.yaw.p_term.resize(1);
  pid_msg_.yaw.i_term.resize(1);
  pid_msg_.yaw.d_term.resize(1);
}

void DragonImpedanceGimbalController::attControlFeedbackStateCallback(const spinal::RollPitchYawTermConstPtr& msg)
{
  if(motor_num_ == 0) return;

  if(!add_lqi_result_) return;

  /* reproduce the control term about attitude in spinal based on LQI, instead of the roll/pitch control from pose linear controller  */
  /* -- only consider the P term and I term, since D term (angular velocity from gyro) in current Dragon platform is too noisy -- */

  for(int i = 0; i < motor_num_; i++)
    {
      lqi_att_terms_.at(i) = -roll_gains_.at(i)[0] * (msg->roll_p / 1000.0) + roll_gains_.at(i)[1] * (msg->roll_i / 1000.0) + (-pitch_gains_.at(i)[0]) * (msg->pitch_p / 1000.0) + pitch_gains_.at(i)[1] * (msg->pitch_i / 1000.0);
    }
}

bool DragonImpedanceGimbalController::update()
{
  if(gimbal_vectoring_check_flag_) return false;

  HydrusImpedanceController::update();

  return true;
}


void DragonImpedanceGimbalController::controlCore()
{
  Eigen::MatrixXd BE = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd Bpr = Eigen::MatrixXd::Zero(3, 9);
  Eigen::MatrixXd CE = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(9, 9);
  Eigen::VectorXd x =  Eigen::VectorXd::Zero(9); 
  Eigen::VectorXd x_dot =  Eigen::VectorXd::Zero(9); 
  Eigen::VectorXd x_d_dot =  Eigen::VectorXd::Zero(9); 
  Eigen::VectorXd x_d_ddot =  Eigen::VectorXd::Zero(9); 
  if (mode_.data == 1)
  {
    x =  Eigen::VectorXd::Zero(6); 
    x_dot =  Eigen::VectorXd::Zero(6); 
    x_d_dot =  Eigen::VectorXd::Zero(6); 
    x_d_ddot =  Eigen::VectorXd::Zero(6); 
  }

  
  Eigen::VectorXd u = Eigen::VectorXd::Zero(9); 
 

  Eigen::MatrixXd J1_p = getPositionJacobian("link1");
  Eigen::MatrixXd J2_p = getPositionJacobian("link2");
  Eigen::MatrixXd J3_p = getPositionJacobian("link3");
  Eigen::MatrixXd J4_p = getPositionJacobian("link4");
  Eigen::MatrixXd Je_p = getPositionJacobian("end_effector");
  Eigen::MatrixXd J1_o = getOrientationJacobian("link1");
  Eigen::MatrixXd J2_o = getOrientationJacobian("link2");
  Eigen::MatrixXd J3_o = getOrientationJacobian("link3");
  Eigen::MatrixXd J4_o = getOrientationJacobian("link4");

  Eigen::Vector3d P1 = robot_model_->getPosition("link1");
  Eigen::Vector3d P2 = robot_model_->getPosition("link2");
  Eigen::Vector3d P3 = robot_model_->getPosition("link3");
  Eigen::Vector3d P4 = robot_model_->getPosition("link4");
  Eigen::Vector3d Pe = robot_model_->getPosition("end_effector");
  Eigen::Matrix3d R1 = robot_model_->getRotation("link1");
  Eigen::Matrix3d R2 = robot_model_->getRotation("link2");
  Eigen::Matrix3d R3 = robot_model_->getRotation("link3");
  Eigen::Matrix3d R4 = robot_model_->getRotation("link4");
  double M1 = robot_model_->getInertiaMap().at("link1").getMass();
  double M2 = robot_model_->getInertiaMap().at("link2").getMass();
  double M3 = robot_model_->getInertiaMap().at("link3").getMass();
  double M4 = robot_model_->getInertiaMap().at("link4").getMass();
  Eigen::Matrix3d I1 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link1").getRotationalInertia());
  Eigen::Matrix3d I2 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link2").getRotationalInertia());
  Eigen::Matrix3d I3 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link3").getRotationalInertia());
  Eigen::Matrix3d I4 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link4").getRotationalInertia());
  Eigen::Matrix3d Rc = aerial_robot_model::kdlToEigen(robot_model_->getCog<KDL::Frame>().M);
  Eigen::Vector3d Pc = aerial_robot_model::kdlToEigen(robot_model_->getCog<KDL::Frame>().p);
  tf::Matrix3x3 cog = estimator_->getOrientation(Frame::COG, estimate_mode_);

  tf::Vector3 rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
 
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = cog.getRow(0).x();
  R(0, 1) = cog.getRow(0).y();
  R(0, 2) = cog.getRow(0).z();

  R(1, 0) = cog.getRow(1).x();
  R(1, 1) = cog.getRow(1).y();
  R(1, 2) = cog.getRow(1).z();

  R(2, 0) = cog.getRow(2).x();
  R(2, 1) = cog.getRow(2).y();
  R(2, 2) = cog.getRow(2).z();

  Eigen::Matrix3d T;
  T(0, 0) = 1.0;
  T(0, 1) = 0.0;
  T(0, 2) = -sin(rpy.y());

  T(1, 0) = 0.0;
  T(1, 1) = cos(rpy.x());
  T(1, 2) = sin(rpy.x()) * cos(rpy.y());

  T(2, 0) = 0.0;
  T(2, 1) = -sin(rpy.x());
  T(2, 2) = cos(rpy.x()) * cos(rpy.y());

  Eigen::Matrix3d Q = R.transpose() * T;

 
  // if (mode_.data == 1) // position_control
  //   J.block(3, 3, 3, 3) = Rc.inverse() * (Je_p - (J1_p + J2_p + J3_p + J4_p) / 4);

  double uav_mass = robot_model_->getMass();
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  // Cartesian Impedance Control of a UAV with a Robotic Arm, Equation (14)

  Eigen::Matrix3d B12 = - M1*aerial_robot_model::skew(R*(P1+P2)/2)*T - M2*aerial_robot_model::skew(R*(P2+P3)/2)*T - M3*aerial_robot_model::skew(R*(P3+P4)/2)*T - M4*aerial_robot_model::skew(R*(P4+Pe)/2)*T;;
  Eigen::MatrixXd B13 = M1*R*J1_p + M2*R*J2_p + M3*R*J3_p + M4*R*J4_p;
  Eigen::MatrixXd B23 = Q.transpose()*R1.transpose()*I1*R1*J1_o + Q.transpose()*R2.transpose()*I2*R2*J2_o + Q.transpose()*R3.transpose()*I3*R3*J3_o + Q.transpose()*R4.transpose()*I4*R4*J4_o - T.transpose()*M1*aerial_robot_model::skew(R*(P1+P2)/2).transpose()*R*J1_p - T.transpose()*M2*aerial_robot_model::skew(R*(P2+P3)/2).transpose()*R*J2_p - T.transpose()*M3*aerial_robot_model::skew(R*(P3+P4)/2).transpose()*R*J3_p - T.transpose()*M4*aerial_robot_model::skew(R*(P4+Pe)/2).transpose()*R*J4_p;
  // Bpr.block(0, 0, 3, 3) = B12;
  Bpr.block(0, 3, 3, 6) = B13;
  BE.block(0, 0, 3, 3) = Q.transpose() * inertia * Q;
  BE.block(3, 3, 3, 3) = M1*J1_p.transpose()*J1_p + M2*J2_p.transpose()*J2_p + M3*J3_p.transpose()*J3_p + M4*J4_p.transpose()*J4_p + J1_o.transpose()*R1.transpose()*I1*R1*J1_o + J2_o.transpose()*R2.transpose()*I2*R2*J2_o + J3_o.transpose()*R3.transpose()*I3*R3*J3_o + J4_o.transpose()*R4.transpose()*I4*R4*J4_o;
  BE.block(0, 3, 3, 6) = B23;
  BE.block(3, 0, 6, 3) = B23.transpose();
  ros::Time time = ros::Time::now();
  BE -= Bpr.transpose() * Bpr / uav_mass;
  CE = - Bpr.transpose() * (Bpr - Pre_Bpr_) / (time-time_).toSec() / (uav_mass * uav_mass);
 




  // std::cout<<"CE"<<CE<<std::endl;
  

  // Parameters for impedance control
  // Setting Kd as uav_mass + (-Cx + 2 * sqrt(Bx * Kp) place the system at critical damping
  // If you set Kd so small(at underdamped), you can see the UAV jumping like inertia-spring system
  // Here Bx = uav_mass and Cx = -Bx
  // See Exploiting Redundancy in Cartesian Impedance Control of UAVs  Equipped with a Robotic Arm, Equation (10)

  Kp.block(0, 0, 3, 3) = 0.5 * Eigen::Matrix3d::Identity();
  Kp.block(3, 3, 6, 6) = 4.0 * Eigen::MatrixXd::Identity(6, 6);

  
  tf::Vector3 target_vel_ = navigator_->getTargetVel();
  tf::Vector3 target_acc_ = navigator_->getTargetAcc();
  tf::Vector3 target_omega_ = navigator_->getTargetOmega();
  tf::Vector3 target_ang_acc_ = navigator_->getTargetAngAcc();
 

  x(0) = pid_controllers_.at(ROLL).getErrP();
  x(1) = pid_controllers_.at(PITCH).getErrP();
  x(2) = pid_controllers_.at(YAW).getErrP();
  if (mode_.data == 1)
  {
    x(3) = pos_cmd_.x - (Rc.inverse() * (Pe - Pc))[0];
    x(4) = pos_cmd_.y - (Rc.inverse() * (Pe - Pc))[1];
    x(5) = pos_cmd_.z - (Rc.inverse() * (Pe - Pc))[2];
  }
  else
  {  
    x(3) = target_joint_pos_[0] - joint_pos_[0];
    x(4) = target_joint_pos_[1] - joint_pos_[1];
    x(5) = target_joint_pos_[2] - joint_pos_[2];
    x(6) = target_joint_pos_[3] - joint_pos_[3];
    x(7) = target_joint_pos_[4] - joint_pos_[4];
    x(8) = target_joint_pos_[5] - joint_pos_[5];
  }

  x_dot(0) = pid_controllers_.at(ROLL).getErrD();
  x_dot(1) = pid_controllers_.at(PITCH).getErrD();
  x_dot(2) = pid_controllers_.at(YAW).getErrD();
  if (mode_.data == 1)
  {
    x_dot(3) = - ((Rc.inverse() * (Pe - Pc))[0] - Pre_Pe_[0]) / (time-time_).toSec();
    x_dot(4) = - ((Rc.inverse() * (Pe - Pc))[1] - Pre_Pe_[1]) / (time-time_).toSec();
    x_dot(5) = - ((Rc.inverse() * (Pe - Pc))[2] - Pre_Pe_[2]) / (time-time_).toSec();
  }
  else
  {  
    x_dot(3) = target_joint_vel_[0] - joint_vel_[0];
    x_dot(4) = target_joint_vel_[1] - joint_vel_[1];
    x_dot(5) = target_joint_vel_[2] - joint_vel_[2];
    x_dot(6) = target_joint_vel_[3] - joint_vel_[3];
    x_dot(7) = target_joint_vel_[4] - joint_vel_[4];
    x_dot(8) = target_joint_vel_[5] - joint_vel_[5];
  }

  x_d_dot(0) = target_omega_.x();
  x_d_dot(1) = target_omega_.y();
  x_d_dot(2) = target_omega_.z();
  if (mode_.data == 1)
  {

  }
  else
  {
    x_d_dot(3) = target_joint_vel_[0];
    x_d_dot(4) = target_joint_vel_[1];
    x_d_dot(5) = target_joint_vel_[2];
    x_d_dot(6) = target_joint_vel_[3];
    x_d_dot(7) = target_joint_vel_[4];
    x_d_dot(8) = target_joint_vel_[5];

  }

  x_d_ddot(0) = target_ang_acc_.x();
  x_d_ddot(1) = target_ang_acc_.y();
  x_d_ddot(2) = target_ang_acc_.z();
  if (mode_.data == 1)
  {

  }
  else
  {
    x_d_ddot(3) = target_joint_acc_[0];
    x_d_ddot(4) = target_joint_acc_[1];
    x_d_ddot(5) = target_joint_acc_[2];
    x_d_ddot(6) = target_joint_acc_[3];
    x_d_ddot(7) = target_joint_acc_[4];
    x_d_ddot(8) = target_joint_acc_[5];
  }



  // Suppose C = 0, then Cx = -Bx,  see Exploiting Redundancy in Cartesian Impedance Control of UAVs Equipped with a Robotic Arm, Equation (9)
  
  Eigen::MatrixXd Bx = aerial_robot_model::pseudoinverse(J).transpose() * BE * aerial_robot_model::pseudoinverse(J);
  Eigen::MatrixXd Cx = aerial_robot_model::pseudoinverse(J).transpose() * (CE - BE * aerial_robot_model::pseudoinverse(J) * (J - Pre_J_) / (time-time_).toSec()) * aerial_robot_model::pseudoinverse(J);
  
  
  //Kd = -Cx + 2 * (Kp * Bx).sqrt();

  //Kd_.block(0, 0, 3, 3) = -Cx.block(0, 0, 3, 3) + 2 * 0.9 * (Kp_.block(0, 0, 3, 3) * Bx.block(0, 0, 3, 3)).sqrt();
  Kd.block(0, 0, 3, 3) = 1.0 * Eigen::Matrix3d::Identity();
  Kd.block(3, 3, 6, 6) = 0.3 * Eigen::MatrixXd::Identity(6, 6);
  //Kd.block(3, 3, 3, 3) = -Cx.block(3, 3, 3, 3) + 2 * 0.2 * (Kp.block(3, 3, 3, 3) * Bx.block(3, 3, 3, 3)).sqrt();

  

  // Exploiting Redundancy in Cartesian Impedance Control of UAVs Equipped with a Robotic Arm, Equation (9)
  u = J.transpose() * (Bx * x_d_ddot + Cx * x_d_dot + Kd * x_dot + Kp * x);
  // Gravity compensation

  double Kpz = 20.0;
  double Kdz = 2 * sqrt(uav_mass * Kpz);
  double uz = Kdz * pid_controllers_.at(Z).getErrP() + Kpz * pid_controllers_.at(Z).getErrD() + uav_mass * aerial_robot_estimation::G;
  //


  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_inv_ = aerial_robot_model::pseudoinverse(P);



  target_thrust_z_term_ = P_inv_.col(2) * uz;
  target_thrust_yaw_term_ =  P_inv_.col(5) * u(2); 



  UnderActuatedImpedanceController::controlCore();
  target_pitch_ = 0; // reset
  target_roll_ = 0;  // reset

  gimbalControl();
  for (int i = 0; i < target_gimbal_angles_.size(); i++)
  {
    //std::cout<<"i: "<<target_gimbal_angles_.at(i)<<std::endl;
  }

  std_msgs::Float64 j1_term, j2_term, j3_term, j4_term, j5_term, j6_term;

  // j1_term.data = 0.0;
  // j2_term.data = 0.0;
  // j3_term.data = 0.0;
  // j4_term.data = 0.0;
  // j5_term.data = 0.0;
  // j6_term.data = 0.0;

  j1_term.data = u(3) + cos(target_gimbal_angles_.at(0)) * cos(target_gimbal_angles_.at(1)) * 0.212 * (target_thrust_yaw_term_[0] - M1 * aerial_robot_estimation::G);
  j2_term.data = u(4);
  j3_term.data = u(5);
  j4_term.data = u(6);
  j5_term.data = u(7) + cos(target_gimbal_angles_.at(4)) * cos(target_gimbal_angles_.at(5)) * 0.212 * (target_thrust_yaw_term_[3] - M4 * aerial_robot_estimation::G);
  j6_term.data = u(8);



  std::cout<<sin(target_gimbal_angles_.at(0)) * cos(target_gimbal_angles_.at(1)) * 0.212 * (target_thrust_yaw_term_[0] - M1 * aerial_robot_estimation::G)<<std::endl;

  joint_cmd_pubs_[0].publish(j1_term);
  joint_cmd_pubs_[1].publish(j2_term);
  joint_cmd_pubs_[2].publish(j3_term);
  joint_cmd_pubs_[3].publish(j4_term);
  joint_cmd_pubs_[4].publish(j5_term);
  joint_cmd_pubs_[5].publish(j6_term);
 
  std::cout<<"x: "<< x<<std::endl;
  std::cout<<"target_joint_pos_: "<< target_joint_pos_<<std::endl;
  std::cout<<"joint_pos_: "<<joint_pos_<<std::endl;
  // std::cout<<"x_d_ddot_: "<< x_d_ddot<<std::endl;
  // std::cout<<"Bx: "<< BE<<std::endl;
  // std::cout<<"Cx: "<< CE<<std::endl;
  // std::cout<<"Kd_: "<< Kd<<std::endl;
  // std::cout<<"Kp_: "<< Kp<<std::endl;
  // std::cout<<"j: "<<  J_<<std::endl;
  // std::cout<<"j: "<<  Pre_J_<<std::endl;
  // std::cout<<"Jdot: "<<  (J_ - Pre_J_) / (time-time_).toSec()<<std::endl;
  std::cout<<"u: "<< u<<std::endl;

  Pre_J_ = J;
  Pre_Pe_ = Rc.inverse() * (Pe - Pc);
  Pre_Bpr_ = Bpr;
  time_ = time;


 

}

void DragonImpedanceGimbalController::gimbalControl()
{
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::Matrix3d links_inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  const std::vector<float>& z_control_terms = target_base_thrust_;

  double max_x = 1e-6;
  double max_y = 1e-6;
  double max_z = 1e-6;

  Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3, motor_num_  * 2);
  double acc_z = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      P_att(0, 2 * i + 1) = -rotors_origin_from_cog.at(i)(2); //x(roll)
      P_att(1, 2 * i) = rotors_origin_from_cog.at(i)(2); //y(pitch)
      P_att(2, 2 * i) = -rotors_origin_from_cog.at(i)(1);
      P_att(2, 2 * i + 1) = rotors_origin_from_cog.at(i)(0);

      /* roll pitch condition */
      if(fabs(rotors_origin_from_cog.at(i)(0)) > max_x) max_x = fabs(rotors_origin_from_cog.at(i)(0));
      if(fabs(rotors_origin_from_cog.at(i)(1)) > max_y) max_y = fabs(rotors_origin_from_cog.at(i)(1));
      if(fabs(rotors_origin_from_cog.at(i)(2)) > max_z) max_z = fabs(rotors_origin_from_cog.at(i)(2));

      acc_z += (lqi_att_terms_.at(i) + z_control_terms.at(i));
    }
  acc_z /= robot_model_->getMass();

  Eigen::MatrixXd P_att_orig = P_att;
  P_att = links_inertia.inverse() * P_att_orig;

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, motor_num_  * 2);
  P.block(0, 0, 3, motor_num_ * 2) = P_att;
  P.block(3, 0, 2, motor_num_ * 2) = P_xy_ / robot_model_->getMass();

  double P_det = (P * P.transpose()).determinant();

  if(control_verbose_)
    {
      std::cout << "gimbal P original: \n"  << std::endl << P_att_orig << std::endl;
      std::cout << "gimbal P: \n"  << std::endl << P << std::endl;
      std::cout << "P det: "  << std::endl << P_det << std::endl;
      std::cout << "acc_z: " << acc_z  << std::endl;
    }

  Eigen::VectorXd f_xy;
  tf::Vector3 target_linear_acc_w(pid_controllers_.at(X).result(),
                                  pid_controllers_.at(Y).result(),
                                  pid_controllers_.at(Z).result());
  tf::Vector3 target_linear_acc_cog = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_linear_acc_w;

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  pid_msg_.roll.p_term.at(0) = 0;
  pid_msg_.roll.i_term.at(0) = 0;
  pid_msg_.roll.d_term.at(0) = 0;
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = target_rpy_.x() - rpy_.x();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = target_omega_.x() - omega_.x();
  pid_msg_.pitch.p_term.at(0) = 0;
  pid_msg_.pitch.i_term.at(0) = 0;
  pid_msg_.pitch.d_term.at(0) = 0;
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = target_rpy_.y() - rpy_.y();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = target_omega_.y() - omega_.y();

  if(P_det < gimbal_roll_pitch_control_p_det_thresh_)
    {
      // no pitch roll
      if(control_verbose_) ROS_ERROR("low P_det: %f", P_det);
      P = Eigen::MatrixXd::Zero(3, motor_num_  * 2);
      P.block(0, 0, 1, motor_num_ * 2) = P_att.block(2, 0, 1, motor_num_ * 2);
      P.block(1, 0, 2, motor_num_ * 2) = P_xy_ / robot_model_->getMass();

      f_xy = pseudoinverse(P) * Eigen::Vector3d(target_ang_acc_z, target_linear_acc_cog.x() - (rpy_.y() * acc_z), target_linear_acc_cog.y() - (-rpy_.x() * acc_z));

      // reset  roll pitch control
      pid_controllers_.at(ROLL).reset();
      pid_controllers_.at(PITCH).reset();
      target_ang_acc_x = 0;
      target_ang_acc_y = 0;
    }
  else
    {
      if(max_z > gimbal_roll_pitch_control_rate_thresh_ * max_y)
        {
          if(control_verbose_) ROS_INFO("perform gimbal roll control, max_z: %f, max_y: %f", max_z, max_y);
          pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
          pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
          pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
        }
      else
        {
          pid_controllers_.at(ROLL).reset(); // reset
          target_ang_acc_x = 0;
        }

      if(max_z > gimbal_roll_pitch_control_rate_thresh_ * max_x)
        {
          if(control_verbose_) ROS_INFO("perform gimbal pitch control, max_z: %f, max_x: %f", max_z, max_y);
          pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
          pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
          pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
        }
      else
        {
          pid_controllers_.at(PITCH).reset(); // reset
          target_ang_acc_y = 0;
        }

      Eigen::VectorXd pid_values(5);
      /* F = P# * [roll_pid, pitch_pid, yaw_pid, x_pid, y_pid] */
      pid_values << target_ang_acc_x, target_ang_acc_y, target_ang_acc_z, target_linear_acc_cog.x() - (rpy_.y() * acc_z), target_linear_acc_cog.y() - (-rpy_.x() * acc_z);
      f_xy = pseudoinverse(P) * pid_values;
    }

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;

  std_msgs::Float32MultiArray target_force_msg;
  for(int i = 0; i < motor_num_; i++)
    {
      target_force_msg.data.push_back(f_xy(2 * i));
      target_force_msg.data.push_back(f_xy(2 * i + 1));
    }
  gimbal_target_force_pub_.publish(target_force_msg);

  if(control_verbose_)
    {
      std::cout << "gimbal P_pseudo_inverse:"  << std::endl << pseudoinverse(P) << std::endl;
      std::cout << "gimbal force for horizontal control:"  << std::endl << f_xy << std::endl;
    }

  /* external wrench compensation */
  if(boost::dynamic_pointer_cast<aerial_robot_navigation::DragonNavigator>(navigator_)->getLandingFlag())
    {
      dragon_robot_model_->resetExternalStaticWrench(); // clear the external wrench
      extra_vectoring_force_.setZero(); // clear the extra vectoring force
    }

  Eigen::MatrixXd cog_rot_inv = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(rpy_.x(), rpy_.y(), rpy_.z()).Inverse());
  Eigen::MatrixXd extended_cog_rot_inv = Eigen::MatrixXd::Zero(6, 6);
  extended_cog_rot_inv.topLeftCorner(3,3) = cog_rot_inv;
  extended_cog_rot_inv.bottomRightCorner(3,3) = cog_rot_inv;
  std::map<std::string, Dragon::ExternalWrench> external_wrench_map = dragon_robot_model_->getExternalWrenchMap();
  for(auto& wrench: external_wrench_map) wrench.second.wrench = extended_cog_rot_inv * wrench.second.wrench;

  dragon_robot_model_->calcExternalWrenchCompThrust(external_wrench_map);
  const Eigen::VectorXd& wrench_comp_thrust = dragon_robot_model_->getExWrenchCompensateVectoringThrust();
  if(control_verbose_)
    {
      std::cout << "external wrench  compensate vectoring thrust: " << wrench_comp_thrust.transpose() << std::endl;
    }


  // integrate
  for(int i = 0; i < motor_num_; i++)
    {
      /* vectoring force */
      tf::Vector3 f_i(f_xy(2 * i) + wrench_comp_thrust(3 * i) + extra_vectoring_force_(3 * i),
                      f_xy(2 * i + 1) + wrench_comp_thrust(3 * i + 1) + extra_vectoring_force_(3 * i + 1),
                      z_control_terms.at(i) + lqi_att_terms_.at(i) + wrench_comp_thrust(3 * i + 2) + extra_vectoring_force_(3 * i + 2));


      /* calculate ||f||, but omit pitch and roll term, which will be added in spinal */
      target_base_thrust_.at(i) = (f_i - tf::Vector3(0, 0, lqi_att_terms_.at(i))).length();
      if(control_verbose_)
        ROS_INFO("[gimbal control]: rotor%d, target_thrust vs target_z: [%f vs %f]", i+1, target_base_thrust_.at(i), z_control_terms.at(i));

      if(!start_rp_integration_ || navigator_->getForceLandingFlag())
        f_i.setValue(f_xy(2 * i), f_xy(2 * i + 1), robot_model_->getStaticThrust()[i]);

      /* f -> gimbal angle */
      std::vector<KDL::Rotation> links_frame_from_cog = dragon_robot_model_->getLinksRotationFromCog<KDL::Rotation>();

      /* [S_pitch, -S_roll * C_pitch, C_roll * C_roll]^T = R.transpose * f_i / |f_i| */
      tf::Quaternion q;  tf::quaternionKDLToTF(links_frame_from_cog.at(i), q);
      tf::Vector3 r_f_i = tf::Matrix3x3(q).transpose() * f_i;
      if(control_verbose_) ROS_INFO("gimbal%d r f: [%f, %f, %f]", i + 1, r_f_i.x(), r_f_i.y(), r_f_i.z());

      double gimbal_i_roll = atan2(-r_f_i[1], r_f_i[2]);
      double gimbal_i_pitch = atan2(r_f_i[0], -r_f_i[1] * sin(gimbal_i_roll) + r_f_i[2] * cos(gimbal_i_roll));

      target_gimbal_angles_.at(2 * i) = gimbal_i_roll;
      target_gimbal_angles_.at(2 * i + 1) = gimbal_i_pitch;

      if(control_verbose_) std::cout << "gimbal" << i + 1 <<"r & p: " << gimbal_i_roll << ", "<< gimbal_i_pitch  << std::endl;
    }
}

void DragonImpedanceGimbalController::sendCmd()
{
  HydrusImpedanceController::sendCmd();

  /* send gimbal control command */
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  if (gimbal_vectoring_check_flag_)
    {
      gimbal_control_msg.position = dragon_robot_model_->getGimbalNominalAngles();
    }
  else
    {
      for(int i = 0; i < motor_num_ * 2; i++)
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
    }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

/* external wrench */
bool DragonImpedanceGimbalController::addExternalWrenchCallback(gazebo_msgs::ApplyBodyWrench::Request& req, gazebo_msgs::ApplyBodyWrench::Response& res)
{
  if(dragon_robot_model_->addExternalStaticWrench(req.body_name, req.reference_frame, req.reference_point, req.wrench))
    res.success  = true;
  else
    res.success  = false;

  return true;
}

bool DragonImpedanceGimbalController::clearExternalWrenchCallback(gazebo_msgs::BodyRequest::Request& req, gazebo_msgs::BodyRequest::Response& res)
{
  dragon_robot_model_->removeExternalStaticWrench(req.body_name);
  return true;
}

/* extra vectoring force  */
void DragonImpedanceGimbalController::extraVectoringForceCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE || navigator_->getForceLandingFlag() || boost::dynamic_pointer_cast<aerial_robot_navigation::DragonNavigator>(navigator_)->getLandingFlag()) return;

  if(extra_vectoring_force_.size() != msg->data.size())
    {
      ROS_ERROR_STREAM("gimbal control: can not assign the extra vectroing force, the size is wrong: " << msg->data.size() << "; reset");
      extra_vectoring_force_.setZero();
      return;
    }

  extra_vectoring_force_ = (Eigen::Map<const Eigen::VectorXf>(msg->data.data(), msg->data.size())).cast<double>();

  ROS_INFO_STREAM("add extra vectoring force is: \n" << extra_vectoring_force_.transpose());
}

void DragonImpedanceGimbalController::rosParamInit()
{
  HydrusImpedanceController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "add_lqi_result", add_lqi_result_, false);
  getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false); // check the gimbal vectoring function without position and yaw control

  ros::NodeHandle roll_pitch_nh(control_nh, "roll_pitch");
  getParam<double>(roll_pitch_nh, "gimbal_control_rate_thresh", gimbal_roll_pitch_control_rate_thresh_, 1.0);
  getParam<double>(roll_pitch_nh, "gimbal_control_p_det_thresh", gimbal_roll_pitch_control_p_det_thresh_, 1e-3);
}


Eigen::MatrixXd DragonImpedanceGimbalController::getPositionJacobian(std::string name)
{
    Eigen::MatrixXd jacobian = robot_model_->getJacobians(name);
    Eigen::MatrixXd p_jacobian = Eigen::MatrixXd::Zero(3, 6);
    p_jacobian.block(0, 0, 3, 2) = jacobian.block(0, 2, 3, 2);
    p_jacobian.block(0, 2, 3, 2) = jacobian.block(0, 6, 3, 2);
    p_jacobian.block(0, 4, 3, 2) = jacobian.block(0, 10, 3, 2);
    return p_jacobian;
}

Eigen::MatrixXd DragonImpedanceGimbalController::getOrientationJacobian(std::string name)
{
    Eigen::MatrixXd jacobian = robot_model_->getJacobians(name);
    Eigen::MatrixXd o_jacobian = Eigen::MatrixXd::Zero(3, 6);
    o_jacobian.block(0, 0, 3, 2) = jacobian.block(3, 2, 3, 2);
    o_jacobian.block(0, 2, 3, 2) = jacobian.block(3, 6, 3, 2);
    o_jacobian.block(0, 4, 3, 2) = jacobian.block(3, 10, 3, 2);
    return o_jacobian;
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DragonImpedanceGimbalController, aerial_robot_control::ControlBase);
