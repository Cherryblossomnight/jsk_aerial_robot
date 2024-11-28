#include <mini_quadrotor/quad_impedance_controller.h>

using namespace aerial_robot_control;

QuadImpedanceController::QuadImpedanceController():
  UnderActuatedImpedanceController()
{
}


void QuadImpedanceController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedImpedanceController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller1/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller2/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller3/simulation/command", 1));
  target_joint_pos_[0] = 0.0;
  target_joint_pos_[1] = 0.0;
  target_joint_pos_[2] = 0.0;
  pos_cmd_.x = 0.0;
  pos_cmd_.y = 0.0;
  pos_cmd_.z = -0.3;
}

// bool QuadImpedanceController::checkRobotModel()
// {
//   // boost::shared_ptr<HydrusRobotModel> hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model_);
//   // lqi_mode_ = hydrus_robot_model->getWrenchDof();
//   lqi_mode_ = 3;
//   if(!robot_model_->initialized())
//     {
//       ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
//       return false;
//     }

//   if(!robot_model_->stabilityCheck(verbose_))
//     {
//       ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");
//       // if(hydrus_robot_model->getWrenchDof() == 4 && hydrus_robot_model->getFeasibleControlRollPitchMin() > hydrus_robot_model->getFeasibleControlRollPitchMinThre())
//       //   {
//           ROS_WARN_NAMED("LQI gain generator", "LQI gain generator: change to three axis stable mode");
//           lqi_mode_ = 3;
//           return true;
//         // }

//       //return false;
//     }
 
//   return true;
// }

void QuadImpedanceController::controlCore()
{
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(9, 9);
  Eigen::VectorXd x =  Eigen::VectorXd::Zero(9); 
  Eigen::VectorXd x_dot =  Eigen::VectorXd::Zero(9); 
  Eigen::VectorXd x_d_dot =  Eigen::VectorXd::Zero(9); 
  Eigen::VectorXd x_d_ddot =  Eigen::VectorXd::Zero(9); 
  Eigen::VectorXd u = Eigen::VectorXd::Zero(9); 
 

  Eigen::Matrix3d J1_p = robot_model_->getPositionJacobian("link1");
  Eigen::Matrix3d J2_p = robot_model_->getPositionJacobian("link2");
  Eigen::Matrix3d J3_p = robot_model_->getPositionJacobian("link3");
  Eigen::Matrix3d Je_p = robot_model_->getPositionJacobian("end_effector");
  Eigen::Matrix3d J1_o = robot_model_->getOrientationJacobian("link1");
  Eigen::Matrix3d J2_o = robot_model_->getOrientationJacobian("link2");
  Eigen::Matrix3d J3_o = robot_model_->getOrientationJacobian("link3");

  Eigen::Vector3d P1 = robot_model_->getPosition("link1");
  Eigen::Vector3d P2 = robot_model_->getPosition("link2");
  Eigen::Vector3d P3 = robot_model_->getPosition("link3");
  Eigen::Vector3d Pe = robot_model_->getPosition("end_effector");
  Eigen::Matrix3d R1 = robot_model_->getRotation("link1");
  Eigen::Matrix3d R2 = robot_model_->getRotation("link2");
  Eigen::Matrix3d R3 = robot_model_->getRotation("link3");
  double M1 = robot_model_->getInertiaMap().at("link1").getMass();
  double M2 = robot_model_->getInertiaMap().at("link2").getMass();
  double M3 = robot_model_->getInertiaMap().at("link3").getMass();
  Eigen::Matrix3d I1 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link1").getRotationalInertia());
  Eigen::Matrix3d I2 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link2").getRotationalInertia());
  Eigen::Matrix3d I3 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link3").getRotationalInertia());

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

 
  if (mode_.data == 1) // position_control
    J.block(6, 6, 3, 3) = Je_p;

  double uav_mass = robot_model_->getMass();
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  // Cartesian Impedance Control of a UAV with a Robotic Arm, Equation (14)

  Eigen::Matrix3d B12 = - M1*aerial_robot_model::skew(R*(P1+P2)/2)*T - M2*aerial_robot_model::skew(R*(P2+P3)/2)*T - M3*aerial_robot_model::skew(R*(P3+Pe)/2)*T;
  Eigen::Matrix3d B13 = M1*R*J1_p + M2*R*J2_p + M3*R*J3_p;
  Eigen::Matrix3d B23 = Q.transpose()*R1.transpose()*I1*R1*J1_o + Q.transpose()*R2.transpose()*I2*R2*J2_o + Q.transpose()*R3.transpose()*I3*R3*J3_o - T.transpose()*M1*aerial_robot_model::skew(R*(P1+P2)/2).transpose()*R*J1_p - T.transpose()*M2*aerial_robot_model::skew(R*(P2+P3)/2).transpose()*R*J2_p - T.transpose()*M3*aerial_robot_model::skew(R*(P3+Pe)/2).transpose()*R*J3_p;
 
  B.block(0, 0, 3, 3) = uav_mass * Eigen::Matrix3d::Identity();
  B.block(3, 3, 3, 3) = Q.transpose() * inertia * Q;
  B.block(6, 6, 3, 3) = M1*J1_p.transpose()*J1_p + M2*J2_p.transpose()*J2_p + M3*J3_p.transpose()*J3_p + J1_o.transpose()*R1.transpose()*I1*R1*J1_o + J2_o.transpose()*R2.transpose()*I2*R2*J2_o + J3_o.transpose()*R3.transpose()*I3*R3*J3_o;
  B.block(0, 3, 3, 3) = B12;
  B.block(3, 0, 3, 3) = B12.transpose();
  B.block(0, 6, 3, 3) = B13;
  B.block(6, 0, 3, 3) = B13.transpose();
  B.block(3, 6, 3, 3) = B23;
  B.block(6, 3, 3, 3) = B23.transpose();
  ros::Time time = ros::Time::now();
 
  // std::cout<<J<<std::endl;
  // std::cout<<Pre_J_<<std::endl;
  // std::cout<<  (J - Pre_J_) / (time-time_).toSec()<<std::endl;



  
  

  // Parameters for impedance control
  // Setting Kd as uav_mass + (-Cx + 2 * sqrt(Bx * Kp) place the system at critical damping
  // If you set Kd so small(at underdamped), you can see the UAV jumping like inertia-spring system
  // Here Bx = uav_mass and Cx = -Bx
  // See Exploiting Redundancy in Cartesian Impedance Control of UAVs  Equipped with a Robotic Arm, Equation (10)

  Kp.block(0, 0, 3, 3) = 5 * Eigen::Matrix3d::Identity();
  Kp.block(3, 3, 3, 3) = 0.5 * Eigen::Matrix3d::Identity();
  if (mode_.data == 1)
    Kp.block(6, 6, 3, 3) = 2 * Eigen::Matrix3d::Identity();
  else
    Kp.block(6, 6, 3, 3) = 0.02 * Eigen::Matrix3d::Identity();

  
  tf::Vector3 target_vel_ = navigator_->getTargetVel();
  tf::Vector3 target_acc_ = navigator_->getTargetAcc();
  tf::Vector3 target_omega_ = navigator_->getTargetOmega();
  tf::Vector3 target_ang_acc_ = navigator_->getTargetAngAcc();
 

  x(0) = pid_controllers_.at(X).getErrP();
  x(1) = pid_controllers_.at(Y).getErrP();
  x(2) = pid_controllers_.at(Z).getErrP();
  x(3) = pid_controllers_.at(ROLL).getErrP();
  x(4) = pid_controllers_.at(PITCH).getErrP();
  x(5) = pid_controllers_.at(YAW).getErrP();
  if (mode_.data == 1)
  {
    x(6) = pos_cmd_.x - Pe[0];
    x(7) = pos_cmd_.y - Pe[1];
    x(8) = pos_cmd_.z - Pe[2];
  }
  else
  {  
    x(6) = target_joint_pos_[0] - joint_pos_[0];
    x(7) = target_joint_pos_[1] - joint_pos_[1];
    x(8) = target_joint_pos_[2] - joint_pos_[2];
  }

  x_dot(0) = pid_controllers_.at(X).getErrD();
  x_dot(1) = pid_controllers_.at(Y).getErrD();
  x_dot(2) = pid_controllers_.at(Z).getErrD();
  x_dot(3) = pid_controllers_.at(ROLL).getErrD();
  x_dot(4) = pid_controllers_.at(PITCH).getErrD();
  x_dot(5) = pid_controllers_.at(YAW).getErrD();
  if (mode_.data == 1)
  {
    x_dot(6) = - (Pe[0] - Pre_Pe_[0]) / (time-time_).toSec();
    x_dot(7) = - (Pe[1] - Pre_Pe_[1]) / (time-time_).toSec();
    x_dot(8) = - (Pe[2] - Pre_Pe_[2]) / (time-time_).toSec();
  }
  else
  {  
    x_dot(6) = target_joint_vel_[0] - joint_vel_[0];
    x_dot(7) = target_joint_vel_[1] - joint_vel_[1];
    x_dot(8) = target_joint_vel_[2] - joint_vel_[2];
  }

  x_d_dot(0) = target_vel_.x();
  x_d_dot(1) = target_vel_.y();
  x_d_dot(2) = target_vel_.z();
  x_d_dot(3) = target_omega_.x();
  x_d_dot(4) = target_omega_.y();
  x_d_dot(5) = target_omega_.z();
  if (mode_.data == 1)
  {

  }
  else
  {
    x_d_dot(6) = target_joint_vel_[0];
    x_d_dot(7) = target_joint_vel_[1];
    x_d_dot(8) = target_joint_vel_[2];

  }
  x_d_ddot(0) = target_acc_.x();
  x_d_ddot(1) = target_acc_.y();
  x_d_ddot(2) = target_acc_.z();
  x_d_ddot(3) = target_ang_acc_.x();
  x_d_ddot(4) = target_ang_acc_.y();
  x_d_ddot(5) = target_ang_acc_.z();
  if (mode_.data == 1)
  {

  }
  else
  {
    x_d_ddot(6) = target_joint_acc_[0];
    x_d_ddot(7) = target_joint_acc_[1];
    x_d_ddot(8) = target_joint_acc_[2];
  }



  // Suppose C = 0, then Cx = -Bx,  see Exploiting Redundancy in Cartesian Impedance Control of UAVs Equipped with a Robotic Arm, Equation (9)
  
  Eigen::MatrixXd Bx = aerial_robot_model::pseudoinverse(J).transpose() * B * aerial_robot_model::pseudoinverse(J);
  Eigen::MatrixXd Cx = aerial_robot_model::pseudoinverse(J).transpose() * (C - B * aerial_robot_model::pseudoinverse(J) * (J - Pre_J_) / (time-time_).toSec()) * aerial_robot_model::pseudoinverse(J);
  
  
  //Kd = -Cx + 2 * (Kp * Bx).sqrt();

  Kd.block(0, 0, 3, 3) = -Cx.block(0, 0, 3, 3) + 2 * 0.8 * (Kp.block(0, 0, 3, 3) * Bx.block(0, 0, 3, 3)).sqrt();
  Kd.block(3, 3, 3, 3) = -Cx.block(3, 3, 3, 3) + 2 * 0.8 * (Kp.block(3, 3, 3, 3) * Bx.block(3, 3, 3, 3)).sqrt();
  if (mode_.data == 1)
    Kd.block(6, 6, 3, 3) = 0.1 * Eigen::Matrix3d::Identity();
  else
    Kd.block(6, 6, 3, 3) = -Cx.block(6, 6, 3, 3) + 2 * 0.8 * (Kp.block(6, 6, 3, 3) * Bx.block(6, 6, 3, 3)).sqrt();
  

  // Exploiting Redundancy in Cartesian Impedance Control of UAVs Equipped with a Robotic Arm, Equation (9)
  //u = J.transpose() * (Bx * x_d_ddot + Cx * x_d_dot + Kd * x_dot + Kp * x);
  u = J.transpose() * (Bx * x_d_ddot + Cx * x_d_dot + Kd * x_dot + Kp * x);
  // Gravity compensation



  std::cout<<"Pe"<<Pe<<std::endl;
  std::cout<<"-------------------------"<<std::endl;



  u(2) += uav_mass * aerial_robot_estimation::G;


  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_inv_ = aerial_robot_model::pseudoinverse(P);

  target_thrust_z_term_ = P_inv_.col(2) * u(2); 
  target_thrust_yaw_term_ =  P_inv_.col(5) * u(5); 

  std_msgs::Float64 j1_term, j2_term, j3_term;
  j1_term.data = u(6);
  j2_term.data = u(7) + M2*aerial_robot_estimation::G*sin(joint_pos_[1])*0.05 + M3*aerial_robot_estimation::G*(sin(joint_pos_[1] + joint_pos_[2])*0.05+sin(joint_pos_[1])*0.1);
  j3_term.data = u(8) + M3*aerial_robot_estimation::G*sin(joint_pos_[1] + joint_pos_[2])*0.05;


  joint_cmd_pubs_[0].publish(j1_term);
  joint_cmd_pubs_[1].publish(j2_term);
  joint_cmd_pubs_[2].publish(j3_term);
 
  // std::cout<<"x_: "<< x_<<std::endl;
  // std::cout<<"x_dot_: "<< x_dot_<<std::endl;
  // std::cout<<"x_d_dot_: "<< x_d_dot_<<std::endl;
  // std::cout<<"x_d_ddot_: "<< x_d_ddot_<<std::endl;
  // std::cout<<"Bx: "<< Bx<<std::endl;
  // std::cout<<"Cx: "<< Cx<<std::endl;
  // std::cout<<"Kd_: "<< Kd_<<std::endl;
  // std::cout<<"Kp_: "<< Kp_<<std::endl;
  // std::cout<<"j: "<<  J_<<std::endl;
  // std::cout<<"j: "<<  Pre_J_<<std::endl;
  // std::cout<<"Jdot: "<<  (J_ - Pre_J_) / (time-time_).toSec()<<std::endl;
  //std::cout<<"u: "<< u<<std::endl;

  Pre_J_ = J;
  Pre_Pe_ = Pe;
  time_ = time;

  UnderActuatedImpedanceController::controlCore();
}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::QuadImpedanceController, aerial_robot_control::ControlBase);
