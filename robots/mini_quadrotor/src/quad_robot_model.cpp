#include <mini_quadrotor/quad_robot_model.h>

using namespace aerial_robot_model::transformable;

QuadRobotModel::QuadRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double fc_rp_min_thre, double epsilon):
  RobotModel(init_with_rosparam, verbose, 0, fc_t_min_thre, epsilon)
{
    
}


void QuadRobotModel::updateJacobians()
{
  aerial_robot_model::transformable::RobotModel::updateJacobians();
}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(QuadRobotModel, aerial_robot_model::RobotModel);
