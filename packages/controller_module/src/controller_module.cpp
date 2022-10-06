/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <random>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

#include <dynamic_reconfigure/server.h>
#include <controller_module/controller_paramsConfig.h>

#include <eigen3/Eigen/Eigen>

#include <mrs_uav_managers/controller.h>

//}

namespace controller_module
{

/* class ControllerModule //{ */

class ControllerModule : public mrs_uav_managers::Controller {

public:
  ~ControllerModule(){};

  void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space, const double uav_mass,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::PositionCommand::ConstPtr &control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

private:
  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  double _uav_mass_;

  double hover_thrust_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                             mutex_drs_;
  typedef controller_module::controller_paramsConfig DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t>   Drs_t;
  boost::shared_ptr<Drs_t>                           drs_;
  void                                               callbackDrs(controller_module::controller_paramsConfig &params, uint32_t level);
  DrsParams_t                                        params_;
  std::mutex                                         mutex_params_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void ControllerModule::initialize(const ros::NodeHandle &parent_nh, [[maybe_unused]] const std::string name, const std::string name_space,
                                  const double uav_mass, std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
  _uav_mass_       = uav_mass;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "ControllerModule");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ControllerModule]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------- calculate the default hover thrust ----------- |

  hover_thrust_ = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, _uav_mass_ * common_handlers_->g);

  // | --------------------------- drs -------------------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  Drs_t::CallbackType f = boost::bind(&ControllerModule::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[ControllerModule]: initialized");

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool ControllerModule::activate(const mrs_msgs::AttitudeCommand::ConstPtr &last_attitude_cmd) {

  if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[ControllerModule]: activated without getting the last controller's command");

    return false;
  }

  is_active_ = true;

  return true;
}

//}

/* deactivate() //{ */

void ControllerModule::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[ControllerModule]: deactivated");
}

//}

/* update() //{ */

const mrs_msgs::AttitudeCommand::ConstPtr ControllerModule::update([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &       uav_state,
                                                                   [[maybe_unused]] const mrs_msgs::PositionCommand::ConstPtr &control_reference) {

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (control_reference == mrs_msgs::PositionCommand::Ptr()) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  // current state
  Eigen::Vector3d cur_pos(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d cur_vel(uav_state->velocity.linear.x, uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // reference
  Eigen::Vector3d ref_pos(control_reference->position.x, control_reference->position.y, control_reference->position.z);
  Eigen::Vector3d ref_vel(control_reference->velocity.x, control_reference->velocity.y, control_reference->velocity.z);
  Eigen::Vector3d ref_acc(control_reference->acceleration.x, control_reference->acceleration.y, control_reference->acceleration.z);

  // controller errors
  Eigen::Vector3d ep = ref_pos - cur_pos;
  Eigen::Vector3d ev = ref_vel - cur_vel;

  // feedforward
  Eigen::Vector3d feed_forward = _uav_mass_ * (Eigen::Vector3d(0, 0, common_handlers_->g) + ref_acc);

  // gains
  Eigen::Array3d kp = Eigen::Array3d(params.kpxy, params.kpxy, params.kpz);
  Eigen::Array3d kv = Eigen::Array3d(params.kvxy, params.kvxy, params.kvz);

  // feedbacks
  Eigen::Vector3d position_feedback = kp * ep.array();
  Eigen::Vector3d velocity_feedback = kv * ev.array();

  // desired force
  Eigen::Vector3d f = position_feedback + velocity_feedback + feed_forward;

  // desired heading vector
  Eigen::Vector3d bxd = Eigen::Vector3d(cos(control_reference->heading), sin(control_reference->heading), 0);

  // create desired orientation
  Eigen::Matrix3d Rd;

  Rd.col(2) = f.normalized();
  Rd.col(1) = Rd.col(2).cross(bxd);
  Rd.col(1).normalize();
  Rd.col(0) = Rd.col(1).cross(Rd.col(2));
  Rd.col(0).normalize();

  double des_throttle = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, f.norm());

  // | --------------- prepare the attitude output -------------- |

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  output_command->thrust    = des_throttle;
  output_command->mode_mask = output_command->MODE_ATTITUDE;

  output_command->mass_difference = 0;
  output_command->total_mass      = _uav_mass_;

  output_command->attitude = mrs_lib::AttitudeConverter(Rd);

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "ControllerModule";

  return output_command;
}

//}

// | ------------------- DO NOT MODIFY BELOW ------------------ |

/* //{ callbackDrs() */

void ControllerModule::callbackDrs(controller_module::controller_paramsConfig &params, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_params_, params, params_);

  ROS_INFO("[ControllerModule]: DRS updated");
}

//}

/* getStatus() //{ */

const mrs_msgs::ControllerStatus ControllerModule::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void ControllerModule::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState::ConstPtr &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void ControllerModule::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr ControllerModule::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
}

//}

}  // namespace controller_module

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(controller_module::ControllerModule, mrs_uav_managers::Controller)
