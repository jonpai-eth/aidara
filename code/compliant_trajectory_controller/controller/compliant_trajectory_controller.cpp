/*
 * Copyright 2022 Matthias Mayr and Julian M. Salt-Ducaju
 *
 * Modified 2023 by aidara,
 *   - Adapted for ROS2 based on the C++ Implementation of a Cartesian Impedance
 * Controller for Robotic Manipulators by Matthias Mayr and Julian M.
 * Salt-Ducaju.
 *   - Original work: A C++ Implementation of a Cartesian Impedance Controller
 * for Robotic Manipulators (https://arxiv.org/abs/2212.11215)
 *
 * This code is licensed under the Creative Commons Attribution Share Alike 4.0
 * International license. See https://creativecommons.org/licenses/by-sa/4.0/
 * for details.
 */

#include "compliant_trajectory_controller/compliant_trajectory_controller.hpp"

#include <stddef.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "compliant_trajectory_controller/pseudo_inversion.hpp"
#include "eigen3/Eigen/QR"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;
using GoalHandleFollowJointTrajectory =
    rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;

namespace compliant_trajectory_controller {
CompliantTrajectoryController::CompliantTrajectoryController() : controller_interface::ControllerInterface() {
  this->setStiffness(200., 200., 200., 20., 20., 20., 0.);
  cartesian_stiffness_ = cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
}

Eigen::Vector3d calculateOrientationError(const Eigen::Quaterniond &orientation_d,
                                          Eigen::Quaterniond orientation) {
  // Orientation error
  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  const Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  return error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
}
/*! \brief Calculates a filtered percental update
 *
 * \param[in] target Target value
 * \param[in] current Current value
 * \param[in] filter Percentage of the target value
 * \return Calculated value
 */
template <typename T>
inline T filteredUpdate(T target, T current, double filter) {
  return (1.0 - filter) * current + filter * target;
}

/*! \brief Calculates the filter step
 *
 * \param[in] update_frequency   Update frequency in Hz
 * \param[in] filter_percentage  Filter percentage
 * \return Filter step
 */
inline double filterStep(const double &update_frequency, const double &filter_percentage) {
  const double kappa = -1 / (std::log(1 - std::min(filter_percentage, 0.999999)));
  return 1.0 / (kappa * update_frequency + 1.0);
}

/*! \brief Saturate a variable x with the limits x_min and x_max
 *
 * \param[in] x Value
 * \param[in] x_min Minimal value
 * \param[in] x_max Maximum value
 * \return Saturated value
 */
inline double saturateValue(double x, double x_min, double x_max) {
  return std::min(std::max(x, x_min), x_max);
}

/*! Saturate the torque rate to not stress the motors
 *
 * \param[in] tau_d_calculated Calculated input torques
 * \param[out] tau_d_saturated Saturated torque values
 * \param[in] delta_tau_max
 */
inline void saturateTorqueRate(const Eigen::VectorXd &tau_d_calculated, Eigen::VectorXd *tau_d_saturated,
                               double delta_tau_max) {
  for (size_t i = 0; i < tau_d_calculated.size(); i++) {
    const double difference = tau_d_calculated[i] - tau_d_saturated->operator()(i);
    tau_d_saturated->operator()(i) += saturateValue(difference, -delta_tau_max, delta_tau_max);
  }
}

void CompliantTrajectoryController::initDesiredPose(const Eigen::Vector3d &position_d_target,
                                                    const Eigen::Quaterniond &orientation_d_target) {
  this->setReferencePose(position_d_target, orientation_d_target);
  position_d_ = position_d_target;
  orientation_d_ = orientation_d_target;
}

void CompliantTrajectoryController::initNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target) {
  this->setNullspaceConfig(q_d_nullspace_target);
  q_d_nullspace_ = q_d_nullspace_target_;
}

void CompliantTrajectoryController::setNumberOfJoints(size_t n_joints) {
  if (n_joints < 0) {
    throw std::invalid_argument("Number of joints must be none negative");
  }
  n_joints_ = n_joints;
  q_ = Eigen::VectorXd::Zero(n_joints_);
  dq_ = Eigen::VectorXd::Zero(n_joints_);
  jacobian_ = Eigen::MatrixXd::Zero(6, n_joints_);
  jacobian_pinv_ = Eigen::MatrixXd::Zero(n_joints_, 6);
  q_d_nullspace_ = Eigen::VectorXd::Zero(n_joints_);
  q_d_nullspace_target_ = q_d_nullspace_;
  tau_c_ = Eigen::VectorXd::Zero(n_joints_);
}

void CompliantTrajectoryController::calculateWrench() {
  // Calculate the wrench to compensate for weight of grasped objects.
  double z_diff = position_d_[2] - position_[2];
  cartesian_wrench_target_[2] = -z_diff * cartesian_stiffness_(2, 2);
}

void CompliantTrajectoryController::setStiffness(const Eigen::Matrix<double, 7, 1> &stiffness,
                                                 bool auto_damping) {
  for (int i = 0; i < 6; i++) {
    // Set diagonal values of stiffness matrix
    if (stiffness(i) < 0.0) {
      assert(stiffness(i) >= 0 && "Stiffness values need to be positive.");
      this->cartesian_stiffness_target_(i, i) = 0.0;
    } else {
      this->cartesian_stiffness_target_(i, i) = stiffness(i);
    }
  }
  if (stiffness(6) < 0.0) {
    assert(stiffness(6) >= 0.0 && "Stiffness values need to be positive.");
    nullspace_stiffness_target_ = 0.0;
  } else {
    nullspace_stiffness_target_ = stiffness(6);
  }
  if (auto_damping) {
    this->applyDamping();
  }
}

void CompliantTrajectoryController::setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y,
                                                 double r_z, double n, bool auto_damping) {
  Eigen::Matrix<double, 7, 1> stiffness_vector(7);
  stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, n;
  this->setStiffness(stiffness_vector, auto_damping);
}

void CompliantTrajectoryController::setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y,
                                                 double r_z, bool auto_damping) {
  Eigen::Matrix<double, 7, 1> stiffness_vector(7);
  stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, nullspace_stiffness_target_;
  this->setStiffness(stiffness_vector, auto_damping);
}

void CompliantTrajectoryController::setDampingFactors(double d_x, double d_y, double d_z, double d_a,
                                                      double d_b, double d_c, double d_n) {
  Eigen::Matrix<double, 7, 1> damping_new;
  damping_new << d_x, d_y, d_z, d_a, d_b, d_c, d_n;
  for (size_t i = 0; i < damping_new.size(); i++) {
    if (damping_new(i) < 0) {
      assert(damping_new(i) >= 0 && "Damping factor must not be negative.");
      damping_new(i) = damping_factors_(i);
    }
  }
  damping_factors_ = damping_new;
  this->applyDamping();
}

void CompliantTrajectoryController::applyDamping() {
  for (int i = 0; i < 6; i++) {
    assert(damping_factors_(i) >= 0.0 && "Damping values need to be positive.");
    cartesian_damping_target_(i, i) =
        damping_factors_(i) * this->dampingRule(cartesian_stiffness_target_(i, i));
  }
  assert(damping_factors_(6) >= 0.0 && "Damping values need to be positive.");
  nullspace_damping_target_ = damping_factors_(6) * this->dampingRule(nullspace_stiffness_target_);
}

void CompliantTrajectoryController::setReferencePose(const Eigen::Vector3d &position_d_target,
                                                     const Eigen::Quaterniond &orientation_d_target) {
  position_d_target_ << position_d_target;
  orientation_d_target_.coeffs() << orientation_d_target.coeffs();
  orientation_d_target_.normalize();
}

void CompliantTrajectoryController::setNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target) {
  assert(q_d_nullspace_target.size() == n_joints_ && "Nullspace target needs to same size as n_joints_");
  q_d_nullspace_target_ << q_d_nullspace_target;
}

void CompliantTrajectoryController::setFiltering(double update_frequency,
                                                 double filter_params_nullspace_config,
                                                 double filter_params_stiffness, double filter_params_pose,
                                                 double filter_params_wrench) {
  if (update_frequency != -1) {
    this->setUpdateFrequency(update_frequency);
  }
  if (filter_params_nullspace_config != -1) {
    this->setFilterValue(filter_params_nullspace_config, &filter_params_nullspace_config_);
  }
  if (filter_params_stiffness != -1) {
    this->setFilterValue(filter_params_stiffness, &filter_params_stiffness_);
  }
  if (filter_params_pose != -1) {
    this->setFilterValue(filter_params_pose, &filter_params_pose_);
  }
  if (filter_params_wrench != -1) {
    this->setFilterValue(filter_params_wrench, &filter_params_wrench_);
  }
}

void CompliantTrajectoryController::setMaxTorqueDelta(double d) {
  assert(d >= 0.0 && "Allowed torque change must be positive");
  delta_tau_max_ = d;
}

void CompliantTrajectoryController::setMaxTorqueDelta(double d, double update_frequency) {
  this->setMaxTorqueDelta(d);
  this->setUpdateFrequency(update_frequency);
}

Eigen::VectorXd CompliantTrajectoryController::calculateCommandedTorques() {
  // Perform a filtering step
  updateFilteredNullspaceConfig();
  updateFilteredStiffness();
  updateFilteredPose();
  updateFilteredWrench();

  // Compute error term
  error_.head(3) << position_ - position_d_;
  error_.tail(3) << calculateOrientationError(orientation_d_, orientation_);

  pseudoInverse(jacobian_.transpose(), &jacobian_transpose_pinv_);
  Eigen::VectorXd tau_task(n_joints_), tau_nullspace(n_joints_), tau_ext(n_joints_);

  // Torque calculated for Cartesian impedance control with respect to a
  // Cartesian pose reference in the end, in the frame of the end-effector
  // of the robot.
  tau_task << jacobian_.transpose() *
                  (-cartesian_stiffness_ * error_ - cartesian_damping_ * (jacobian_ * dq_));

  // Torque for joint impedance control with respect to a desired
  // configuration and projected in the null-space of the robot's
  // Jacobian, so it should not affect the Cartesian motion of the robot's
  // end-effector.
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian_.transpose() * jacobian_transpose_pinv_) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q_) - nullspace_damping_ * dq_);

  // Torque to achieve the desired external force command in the frame of
  // the end-effector of the robot.
  tau_ext = jacobian_.transpose() * cartesian_wrench_;

  // Torque commanded to the joints of the robot is composed by the
  // superposition of these three joint-torque signals:
  tau_d_ = tau_task + tau_nullspace + tau_ext;

  // Compensate the friction forces on the robot's joints
  if (friction_compensation_enabled_) {
    tau_d_ = tau_d_ + tau_friction();
  }

  saturateTorqueRate(tau_d_, &tau_c_, delta_tau_max_);
  return tau_c_;
}

// Get the state of the robot.Updates when "calculateCommandedTorques" is called
void CompliantTrajectoryController::getState(Eigen::VectorXd *q, Eigen::VectorXd *dq,
                                             Eigen::Vector3d *position, Eigen::Quaterniond *orientation,
                                             Eigen::Vector3d *position_d, Eigen::Quaterniond *orientation_d,
                                             Eigen::Matrix<double, 6, 6> *cartesian_stiffness,
                                             double *nullspace_stiffness, Eigen::VectorXd *q_d_nullspace,
                                             Eigen::Matrix<double, 6, 6> *cartesian_damping) const {
  *q << q_;
  *dq << dq_;
  *position << position_;
  orientation->coeffs() << orientation_.coeffs();
  this->getState(position_d, orientation_d, cartesian_stiffness, nullspace_stiffness, q_d_nullspace,
                 cartesian_damping);
}

void CompliantTrajectoryController::getState(Eigen::Vector3d *position_d, Eigen::Quaterniond *orientation_d,
                                             Eigen::Matrix<double, 6, 6> *cartesian_stiffness,
                                             double *nullspace_stiffness, Eigen::VectorXd *q_d_nullspace,
                                             Eigen::Matrix<double, 6, 6> *cartesian_damping) const {
  *position_d = position_d_;
  orientation_d->coeffs() << orientation_d_.coeffs();
  *cartesian_stiffness = cartesian_stiffness_;
  *nullspace_stiffness = nullspace_stiffness_;
  *q_d_nullspace = q_d_nullspace_;
  *cartesian_damping << cartesian_damping_;
}

Eigen::VectorXd CompliantTrajectoryController::getLastCommands() const { return tau_c_; }

Eigen::Matrix<double, 6, 1> CompliantTrajectoryController::getAppliedWrench() const {
  return cartesian_wrench_;
}

Eigen::Matrix<double, 6, 1> CompliantTrajectoryController::getPoseError() const { return error_; }

double CompliantTrajectoryController::dampingRule(double stiffness) const { return 2 * sqrt(stiffness); }

void CompliantTrajectoryController::setUpdateFrequency(double freq) {
  assert(freq >= 0.0 && "Update frequency needs to be greater or equal to zero");
  update_frequency_ = std::max(freq, 0.0);
}

void CompliantTrajectoryController::setFilterValue(double val, double *saved_val) {
  assert(val > 0 && val <= 1.0 && "Filter params need to be between 0 and 1.");
  *saved_val = saturateValue(val, 0.0000001, 1.0);
}

void CompliantTrajectoryController::updateFilteredNullspaceConfig() {
  if (filter_params_nullspace_config_ == 1.0) {
    q_d_nullspace_ = q_d_nullspace_target_;
  } else {
    const double step = filterStep(update_frequency_, filter_params_nullspace_config_);
    q_d_nullspace_ = filteredUpdate(q_d_nullspace_target_, q_d_nullspace_, step);
  }
}

void CompliantTrajectoryController::updateFilteredStiffness() {
  if (filter_params_stiffness_ == 1.0) {
    cartesian_stiffness_ = cartesian_stiffness_target_;
    cartesian_damping_ = cartesian_damping_target_;
    nullspace_stiffness_ = nullspace_stiffness_target_;
    q_d_nullspace_ = q_d_nullspace_target_;
    nullspace_damping_ = nullspace_damping_target_;
  } else {
    const double step = filterStep(update_frequency_, filter_params_stiffness_);

    cartesian_stiffness_ = filteredUpdate(cartesian_stiffness_target_, cartesian_stiffness_, step);
    cartesian_damping_ = filteredUpdate(cartesian_damping_target_, cartesian_damping_, step);
    nullspace_stiffness_ = filteredUpdate(nullspace_stiffness_target_, nullspace_stiffness_, step);
    nullspace_damping_ = filteredUpdate(nullspace_damping_target_, nullspace_damping_, step);
  }
}

void CompliantTrajectoryController::updateFilteredPose() {
  if (filter_params_pose_ == 1.0) {
    position_d_ << position_d_target_;
    orientation_d_.coeffs() << orientation_d_target_.coeffs();
  } else {
    const double step = filterStep(update_frequency_, filter_params_pose_);

    position_d_ = filteredUpdate(position_d_target_, position_d_, step);
    orientation_d_ = orientation_d_.slerp(step, orientation_d_target_);
  }
}

void CompliantTrajectoryController::updateFilteredWrench() {
  if (filter_params_wrench_ == 1.0) {
    cartesian_wrench_ = cartesian_wrench_target_;
  } else {
    const double step = filterStep(update_frequency_, filter_params_wrench_);
    cartesian_wrench_ = filteredUpdate(cartesian_wrench_target_, cartesian_wrench_, step);
  }
}

controller_interface::CallbackReturn CompliantTrajectoryController::on_init() {
  std::filesystem::path source_file_path = __FILE__;  // Full path to current source file
  std::filesystem::path parent_path = source_file_path.parent_path();
  const std::string urdf_filename = (parent_path / "urdf/panda.urdf").string();

  pinocchio::urdf::buildModel(urdf_filename, model_);
  pinocchio::Data data(model_);
  data_ = data;

  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);

  command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);

  state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  bool hasPosition = std::find(state_interface_types_.begin(), state_interface_types_.end(), "position") !=
                     state_interface_types_.end();
  bool hasVelocity = std::find(state_interface_types_.begin(), state_interface_types_.end(), "velocity") !=
                     state_interface_types_.end();

  if (joint_names_.size() == 0 || command_interface_types_[0] != "effort" || !hasPosition || !hasVelocity) {
    RCLCPP_ERROR(get_node()->get_logger(), "Interfaces were not assigned properly.");
    return CallbackReturn::ERROR;
  }

  point_interp_.positions.assign(joint_names_.size(), 0);
  point_interp_.velocities.assign(joint_names_.size(), 0);
  point_interp_.effort.assign(joint_names_.size(), 0);
  setNumberOfJoints(joint_names_.size());

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CompliantTrajectoryController::command_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto &joint_name : joint_names_) {
    for (const auto &interface_type : command_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration CompliantTrajectoryController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto &joint_name : joint_names_) {
    for (const auto &interface_type : state_interface_types_) {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::CallbackReturn CompliantTrajectoryController::on_configure(
    const rclcpp_lifecycle::State &) {
  auto callback = [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) -> void {
    traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
    new_msg_ = true;
  };
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CompliantTrajectoryController::on_activate(
    const rclcpp_lifecycle::State &) {
  // clear out vectors in case of restart
  joint_effort_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  // assign command interfaces
  for (auto &interface : command_interfaces_) {
    command_interface_map_[interface.get_interface_name()]->push_back(interface);
  }
  // assign state interfaces
  auto it = state_interfaces_.begin();
  for (int i = 0; i < 21 && it != state_interfaces_.end(); ++i, ++it) {
    auto &interface = *it;
    state_interface_map_[interface.get_interface_name()]->push_back(interface);
  }
  relax_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "compliant_trajectory_controller/relax",
      std::bind(&CompliantTrajectoryController::relax, this, std::placeholders::_1, std::placeholders::_2));

  tighten_up_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "compliant_trajectory_controller/tighten_up",
      std::bind(&CompliantTrajectoryController::tighten_up, this, std::placeholders::_1,
                std::placeholders::_2));
  // creating a service to change parameters at runtime
  param_service_ = get_node()->create_service<aidara_msgs::srv::UpdateParams>(
      "compliant_trajectory_controller/update_params",
      std::bind(&CompliantTrajectoryController::updateParams, this, std::placeholders::_1,
                std::placeholders::_2));

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(get_node());
  timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&CompliantTrajectoryController::publish_ee_position, this));

  eef_publisher_ = get_node()->create_publisher<geometry_msgs::msg::TransformStamped>("/eef_pos", 10);

  // creating a client to increase allowed torques. Franka specific
  limit_client_ = this->get_node()->create_client<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
      "/service_server/set_force_torque_collision_behavior");

  auto request = std::make_shared<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request>();
  request->lower_torque_thresholds_nominal = {80, 80, 80, 80, 80, 80, 80};
  request->upper_torque_thresholds_nominal = {120, 120, 120, 120, 120, 120, 120};
  request->lower_force_thresholds_nominal = {40, 40, 40, 40, 40, 40};
  request->upper_force_thresholds_nominal = {80, 80, 80, 80, 80, 80};

  auto result = limit_client_->async_send_request(request);

  // creating an action server to receive trajectories
  trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      get_node(), "/compliant_trajectory_controller/follow_joint_trajectory",
      std::bind(&CompliantTrajectoryController::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&CompliantTrajectoryController::handle_cancel, this, std::placeholders::_1),
      std::bind(&CompliantTrajectoryController::handle_accepted, this, std::placeholders::_1));

  // get the current states
  this->updateState();
  // Set reference pose to current pose and q_d_nullspace
  this->initDesiredPose(position_, orientation_);
  this->initNullspaceConfig(q_);
  // setting Cartesian stiffness
  setStiffness(200., 200., 200., 80., 80., 80., 5., false);
  // setting additional Cartesian damping
  setDampingFactors(1., 1., 1., 0.1, 0.1, 0.1, 1.);

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CompliantTrajectoryController::update(const rclcpp::Time & /*time*/,
                                                                        const rclcpp::Duration & /*period*/) {
  // Check if trajectory is active
  updateState();
  if (traj_running_) {
    trajUpdate();
  }

  // Apply control law in base library
  this->calculateCommandedTorques();

  for (size_t i = 0; i < joint_effort_command_interface_.size(); i++) {
    joint_effort_command_interface_[i].get().set_value(tau_c_[i]);
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CompliantTrajectoryController::on_deactivate(
    const rclcpp_lifecycle::State &) {
  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CompliantTrajectoryController::on_cleanup(
    const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CompliantTrajectoryController::on_error(
    const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CompliantTrajectoryController::on_shutdown(
    const rclcpp_lifecycle::State &) {
  return CallbackReturn::SUCCESS;
}
void CompliantTrajectoryController::publish_ee_position() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = get_node()->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "eef";

  t.transform.translation.x = position_[0];
  t.transform.translation.y = position_[1];
  t.transform.translation.z = position_[2];

  t.transform.rotation.x = orientation_.x();
  t.transform.rotation.y = orientation_.y();
  t.transform.rotation.z = orientation_.z();
  t.transform.rotation.w = orientation_.w();

  // Send the transformation
  eef_publisher_->publish(t);
  tf_broadcaster_->sendTransform(t);
}

// Callback methods for the action server
// Goal handle callback
rclcpp_action::GoalResponse CompliantTrajectoryController::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
  RCLCPP_INFO(get_node()->get_logger(), "Received goal request");
  // Cancel running trajectories first
  traj_running_ = false;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Cancel handle callback
rclcpp_action::CancelResponse CompliantTrajectoryController::handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
  RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");
  // You can add logic here to handle a cancellation request
  traj_running_ = false;

  return rclcpp_action::CancelResponse::ACCEPT;
}

// Accepted goal handle callback
void CompliantTrajectoryController::handle_accepted(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
  // This needs to run in a separate thread to avoid blocking
  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

    // Initialize trajectory
    trajStart(goal->trajectory);
    while (traj_running_) {
      // Populate joint names
      feedback->joint_names = joint_names_;

      // Populate the desired point
      feedback->desired.positions = std::vector<double>(
          q_d_nullspace_target_.data(), q_d_nullspace_target_.data() + q_d_nullspace_target_.size());

      // Populate the actual point
      feedback->actual.positions = std::vector<double>(q_.data(), q_.data() + q_.size());

      // Calculate and populate the error point
      Eigen::VectorXd error = q_d_nullspace_target_ - q_;
      feedback->error.positions = std::vector<double>(error.data(), error.data() + error.size());

      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Set the result
    result->set__error_code(control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
    result->set__error_string("Goal Reached");
    goal_handle->succeed(result);
    RCLCPP_INFO(get_node()->get_logger(), "Completed trajectory");
  }).detach();
}

void CompliantTrajectoryController::trajUpdate() {
  Eigen::Vector3d position_d_error = (position_d_target_) - (position_);

  auto time_now = rclcpp::Clock().now() - time_delay_;
  auto time_last = traj_start_ + trajectory_.points.at(traj_index_).time_from_start;

  if (trajectory_.points.size() == 1 || time_last > time_now) {
    std::vector<double> positions_next = trajectory_.points[0].positions;
    Eigen::VectorXd q_next = Eigen::Map<Eigen::VectorXd>(positions_next.data(), positions_next.size());
    getFk(q_next, &position_d_target_, &orientation_d_target_);
    this->setNullspaceConfig(q_next);

    if (trajectory_.points.size() == 1) {
      RCLCPP_INFO(get_node()->get_logger(), "Received a single point as a trajectory.");
      traj_running_ = false;
      return;
    }
  }

  auto time_next = traj_start_ + trajectory_.points.at(traj_index_ + 1).time_from_start;

  // End timer if a disturbance ends, updates time delay
  if (error_above_threshold_ && position_d_error.norm() < 0.1) {
    time_delay_ += rclcpp::Clock().now() - time_delay_start_;
    error_above_threshold_ = false;
  }

  // First point must have time different from zero
  if (time_now > time_last && time_now < time_next) {
    std::vector<double> positions_next = trajectory_.points[traj_index_ + 1].positions;
    std::vector<double> positions_last = trajectory_.points[traj_index_].positions;

    Eigen::VectorXd q_next = Eigen::Map<Eigen::VectorXd>(positions_next.data(), positions_next.size());
    Eigen::VectorXd q_last = Eigen::Map<Eigen::VectorXd>(positions_last.data(), positions_last.size());

    // Linear interpolation of the state q
    auto time_av = (time_now - time_last).seconds() / (time_next - time_last).seconds();
    Eigen::VectorXd q_av = q_last + (q_next - q_last) * time_av;
    getFk(q_av, &position_d_target_, &orientation_d_target_);
    this->setNullspaceConfig(q_av);
  } else if (position_d_error.norm() > 0.1) {
    // Start timer if a disturbance occurs
    if (!error_above_threshold_) {
      error_above_threshold_ = true;
      time_delay_start_ = rclcpp::Clock().now();
    }
  } else if (traj_index_ < trajectory_.points.size() - 2) {
    traj_index_++;
  } else {
    traj_running_ = false;
  }
}

bool CompliantTrajectoryController::getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position,
                                          Eigen::Quaterniond *orientation) {
  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name_);

  // Update member variables
  *position = data_.oMf[frame_id].translation();
  *orientation = Eigen::Quaterniond(data_.oMf[frame_id].rotation());
  return true;
}
void CompliantTrajectoryController::relax(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (traj_running_) {
    RCLCPP_INFO(get_node()->get_logger(), "Aborting trajectory");
    traj_running_ = false;
  }
  this->calculateWrench();
  friction_compensation_enabled_ = false;
  pre_relax_stiffness_.head<6>() = cartesian_stiffness_.diagonal();
  pre_relax_stiffness_[6] = nullspace_stiffness_;
  setStiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);
  response->success = true;
}

void CompliantTrajectoryController::tighten_up(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (traj_running_) {
    RCLCPP_INFO(get_node()->get_logger(), "Robot is not relaxed");
    return;
  }

  this->initDesiredPose(position_, orientation_);
  this->initNullspaceConfig(q_);

  setStiffness(pre_relax_stiffness_(0), pre_relax_stiffness_(1), pre_relax_stiffness_(2),
               pre_relax_stiffness_(3), pre_relax_stiffness_(4), pre_relax_stiffness_(5),
               pre_relax_stiffness_(6), false);

  friction_compensation_enabled_ = true;
  response->success = true;
}

void CompliantTrajectoryController::updateParams(
    const std::shared_ptr<aidara_msgs::srv::UpdateParams::Request> request,
    std::shared_ptr<aidara_msgs::srv::UpdateParams::Response> response) {
  RCLCPP_INFO(get_node()->get_logger(), "Got param msg from updateParams topic.");

  response->success = true;
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't update params. Controller is not active.");
    response->success = false;
    return;
  }

  const auto &stiffness = request->stiffness;
  const auto &damping = request->damping;
  const auto &filter = request->filter;

  if (stiffness[0] != -1) {
    if (stiffness[6] != -1) {
      setStiffness(stiffness[0], stiffness[1], stiffness[2], stiffness[3], stiffness[4], stiffness[5],
                   stiffness[6], request->autodamping);
    } else {
      setStiffness(stiffness[0], stiffness[1], stiffness[2], stiffness[3], stiffness[4], stiffness[5],
                   request->autodamping);
    }
  }

  if (damping[0] != -1) {
    setDampingFactors(damping[0], damping[1], damping[2], damping[3], damping[4], damping[5], damping[6]);
  }

  if (request->tau_delta != -1) {
    setMaxTorqueDelta(request->tau_delta);
  }

  setFiltering(filter[0], filter[1], filter[2], filter[3], filter[4]);
  return;
}

void CompliantTrajectoryController::updateState() {
  for (size_t i = 0; i < joint_names_.size(); i++) {
    q_[i] = joint_position_state_interface_.at(i).get().get_value();
    dq_[i] = joint_velocity_state_interface_.at(i).get().get_value();
  }
  getFk(q_, &position_, &orientation_);
  getJacobian();
}

bool CompliantTrajectoryController::getJacobian() {
  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::getJointJacobian(model_, data_, 7, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);

  return true;
}

void CompliantTrajectoryController::trajStart(const trajectory_msgs::msg::JointTrajectory trajectory) {
  // Reset values

  this->initDesiredPose(position_, orientation_);

  if (pre_relax_stiffness_.norm() > 0) {
    setStiffness(pre_relax_stiffness_(0), pre_relax_stiffness_(1), pre_relax_stiffness_(2),
                 pre_relax_stiffness_(3), pre_relax_stiffness_(4), pre_relax_stiffness_(5),
                 pre_relax_stiffness_(6), false);
  }

  friction_compensation_enabled_ = true;
  trajectory_ = trajectory;
  time_delay_ = rclcpp::Duration(0, 0);
  error_above_threshold_ = false;
  traj_running_ = true;
  traj_index_ = 0;
  traj_start_ = rclcpp::Clock().now();

  // Run trajectory
  this->trajUpdate();
  return;
}

// Friction compensation from the ETH pdz lab
Eigen::Matrix<double, 7, 1> CompliantTrajectoryController::tau_friction() {
  const auto logger = get_node()->get_logger();
  double alpha = 0.01;  // constant for exponential filter in relation to static friction moment
  dq_filtered = alpha * dq_ + (1 - alpha) * dq_filtered;
  tau_impedance_filtered = alpha * tau_d_ + (1 - alpha) * tau_impedance_filtered;
  N = (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv_ * jacobian_);
  dq_imp = dq_filtered - N * dq_filtered;
  f = beta.cwiseProduct(dq_imp) + offset_friction;
  dz = dq_imp.array() - dq_imp.array().abs() / g.array() * sigma_0.array() * z.array() +
       0.025 * tau_impedance_filtered.array();

  dz(6) -= 0.02 * tau_impedance_filtered(6);
  z = 0.001 * dz + z;
  return sigma_0.array() * z.array() + 100 * sigma_1.array() * dz.array() + f.array();
}
}  // namespace compliant_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(compliant_trajectory_controller::CompliantTrajectoryController,
                       controller_interface::ControllerInterface)
