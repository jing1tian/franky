#pragma once

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <exception>
#include <future>
#include <optional>
#include <stdexcept>
#include <variant>

#include "franky/cartesian_state.hpp"
#include "franky/control_signal_type.hpp"
#include "franky/dynamics_limit.hpp"
#include "franky/joint_state.hpp"
#include "franky/model.hpp"
#include "franky/motion/motion.hpp"
#include "franky/motion/motion_generator.hpp"
#include "franky/relative_dynamics_factor.hpp"
#include "franky/robot_pose.hpp"
#include "franky/robot_state_estimator.hpp"
#include "franky/robot_velocity.hpp"
#include "franky/types.hpp"
#include "franky/util.hpp"

namespace franky {

/**
 * @brief Exception thrown when an invalid motion type is used.
 *
 * This exception is thrown when a motion is asynchronously executed and a new motion of a different type is set before
 * the previous one finished.
 */
struct InvalidMotionTypeException : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/**
 * @brief A class representing a Franka robot.
 *
 * This class extends the franka::Robot class and adds additional functionality to it.
 */
class Robot : public franka::Robot {
 public:
  /**
   * @brief Global parameters for the robot.
   */
  struct Params {
    /**
     * @brief Relative dynamics factor for the robot.
     *
     * The maximum velocity, acceleration and jerk of the robot are scaled by the factors specified here.
     */
    RelativeDynamicsFactor relative_dynamics_factor{1.0};

    /**
     * @brief Default torque threshold for collision behavior.
     */
    double default_torque_threshold{20.0};

    /**
     * @brief Default force threshold for collision behavior.
     */
    double default_force_threshold{30.0};

    /**
     * @brief Default controller mode for the robot.
     *
     * See libfranka documentation for details.
     */
    franka::ControllerMode controller_mode{franka::ControllerMode::kJointImpedance};

    /**
     * @brief Default realtime configuration for the robot.
     *
     * See libfranka documentation for details.
     */
    franka::RealtimeConfig realtime_config{franka::RealtimeConfig::kEnforce};

    /**
     * @brief Window size of the median filter of the joint velocity estimator.
     *
     * This filter is applied before exponential smoothing.
     */
    size_t dq_estimator_window_size{9};

    /**
     * @brief Joint velocity estimator exponential decay.
     *
     * This parameter is used to smooth the joint velocity estimates. After the median filter, the velocity estimates
     * are computed as
     * \f$ \dot{q}_{t + \Delta t} = \alpha \dot{q}_{t}
     * + ( 1 - \alpha)\left(\frac{\dot{q}_{t + \Delta t} - \dot{q}_{t}}{\Delta t}\right) \f$
     */
    double dq_estimator_alpha{0.9};

    /**
     * @brief Window size of the median filter of the joint acceleration estimator.
     *
     * This filter is applied before exponential smoothing.
     */
    size_t ddq_estimator_window_size{9};

    /**
     * @brief Joint acceleration estimator exponential decay.
     *
     * This parameter is used to smooth the joint acceleration estimates. After the median filter, the acceleration
     * estimates are computed as
     * \f$ \ddot{q}_{t + \Delta t} = \alpha \ddot{q}_{t}
     * + ( 1 - \alpha)\left(\frac{\dot{q}_{t + \Delta t} - \dot{q}_{t}}{\Delta t}\right) \f$
     */
    double ddq_estimator_alpha{0.99};
  };

  /** Number of degrees of freedom of the robot */
  static constexpr size_t degrees_of_freedoms{7};

  /** Control rate of the robot [s] */
  static constexpr double control_rate{0.001};

  /**
   * @param fci_hostname The hostname or IP address of the robot.
   */
  explicit Robot(const std::string &fci_hostname);

  /**
   * @param fci_hostname The hostname or IP address of the robot.
   * @param params The parameters for the robot.
   */
  explicit Robot(const std::string &fci_hostname, const Params &params);

  using franka::Robot::setCollisionBehavior;

  /**
   * @brief Set the collision behavior of the robot.
   *
   * @param torque_threshold The torque threshold for the collision behavior in Nm.
   * @param force_threshold The force threshold for the collision behavior in N.
   */
  void setCollisionBehavior(const ScalarOrArray<7> &torque_threshold, const ScalarOrArray<6> &force_threshold);

  /**
   * @brief Set the collision behavior of the robot.
   *
   * @param lower_torque_threshold The lower torque threshold for the collision behavior in Nm.
   * @param upper_torque_threshold The upper torque threshold for the collision behavior in Nm.
   * @param lower_force_threshold The lower force threshold for the collision behavior in N.
   * @param upper_force_threshold The upper force threshold for the collision behavior in N.
   */
  void setCollisionBehavior(
      const ScalarOrArray<7> &lower_torque_threshold, const ScalarOrArray<7> &upper_torque_threshold,
      const ScalarOrArray<6> &lower_force_threshold, const ScalarOrArray<6> &upper_force_threshold);

  /**
   * @brief Set the collision behavior of the robot.
   *
   * @param lower_torque_threshold_acceleration The lower torque threshold for the collision behavior in Nm during
   * acceleration.
   * @param upper_torque_threshold_acceleration The upper torque threshold for the collision behavior in Nm during
   * acceleration.
   * @param lower_torque_threshold_nominal The lower torque threshold for the collision behavior in Nm during nominal
   * operation.
   * @param upper_torque_threshold_nominal The upper torque threshold for the collision behavior in Nm during nominal
   * operation.
   * @param lower_force_threshold_acceleration The lower force threshold for the collision behavior in N during
   * acceleration.
   * @param upper_force_threshold_acceleration The upper force threshold for the collision behavior in N during
   * acceleration.
   * @param lower_force_threshold_nominal The lower force threshold for the collision behavior in N during nominal
   * operation.
   * @param upper_force_threshold_nominal The upper force threshold for the collision behavior in N during nominal
   * operation.
   */
  void setCollisionBehavior(
      const ScalarOrArray<7> &lower_torque_threshold_acceleration,
      const ScalarOrArray<7> &upper_torque_threshold_acceleration,
      const ScalarOrArray<7> &lower_torque_threshold_nominal, const ScalarOrArray<7> &upper_torque_threshold_nominal,
      const ScalarOrArray<6> &lower_force_threshold_acceleration,
      const ScalarOrArray<6> &upper_force_threshold_acceleration, const ScalarOrArray<6> &lower_force_threshold_nominal,
      const ScalarOrArray<6> &upper_force_threshold_nominal);

  /**
   * @brief Calls the automatic error recovery of the robot and returns whether the recovery was successful.
   * @return Whether the recovery was successful.
   */
  bool recoverFromErrors();

  /**
   * @brief Returns whether the robot has errors.
   * @return Whether the robot has errors.
   */
  [[nodiscard]] bool hasErrors();

  /**
   * @brief Returns the current pose of the robot.
   * @return The current pose of the robot.
   */
  [[nodiscard]] RobotPose currentPose() { return currentCartesianState().pose(); }

  /**
   * @brief Returns the current cartesian velocity of the robot.
   * @return The current cartesian velocity of the robot.
   */
  [[nodiscard]] RobotVelocity currentCartesianVelocity() { return currentCartesianState().velocity(); }

  /**
   * @brief Returns the current cartesian state of the robot.
   * @return The current cartesian state of the robot.
   */
  [[nodiscard]] CartesianState currentCartesianState() {
    auto s = state();
    return {
        {Affine(Eigen::Matrix4d::Map(s.O_T_EE.data())), ElbowState{s.elbow}}, RobotVelocity(s.O_dP_EE_c, s.delbow_c)};
  }

  /**
   * @brief Returns the current joint state of the robot.
   * @return The current joint state of the robot.
   */
  [[nodiscard]] JointState currentJointState() {
    auto s = state();
    return {s.q, s.dq};
  }

  /**
   * @brief Returns the current joint positions of the robot.
   * @return The current joint positions of the robot.
   */
  [[nodiscard]] Vector7d currentJointPositions() { return currentJointState().position(); }

  /**
   * @brief Returns the current joint velocities of the robot.
   * @return The current joint velocities of the robot.
   */
  [[nodiscard]] Vector7d currentJointVelocities() { return currentJointState().velocity(); }

  /**
   * @brief Returns the current state of the robot.
   * @return The current state of the robot.
   */
  [[nodiscard]] RobotState state();

  /**
   * @brief Returns the current global relative dynamics factor of the robot.
   * @return The current relative dynamics factor of the robot.
   */
  [[nodiscard]] RelativeDynamicsFactor relative_dynamics_factor();

  /**
   * @brief Sets the global relative dynamics factor of the robot.
   * @param relative_dynamics_factor The relative dynamics factor to set.
   */
  void setRelativeDynamicsFactor(const RelativeDynamicsFactor &relative_dynamics_factor);

  /**
   * @brief Whether the robot is currently in control, i.e. a motion is being executed.
   */
  [[nodiscard]] bool is_in_control();

  /**
   * @brief The hostname of the robot.
   */
  [[nodiscard]] std::string fci_hostname() const;

  /**
   * @brief The type of the current control signal.
   */
  [[nodiscard]] std::optional<ControlSignalType> current_control_signal_type();

  /**
   * @brief The model of the robot.
   *
   * The model is loaded in the constructor, so calling this function does not incur any overhead.
   */
  [[nodiscard]] std::shared_ptr<const Model> model() const { return model_; }

#ifdef FRANKA_0_15
  /**
   * @brief The URDF of the robot model as string.
   *
   */
  [[nodiscard]] std::string model_urdf() const { return model_urdf_; }
#endif

  /**
   * @brief Wait for the current motion to finish. Throw any exceptions that occurred during the motion.
   */
  bool joinMotion() {
    std::unique_lock lock(*control_mutex_);
    return joinMotionUnsafe(lock);
  }

  /**
   * @brief Wait for the current motion to finish with a timeout. Throw any exceptions that occurred during the motion.
   *
   * After the timeout has expired, the function will return false.
   * @param timeout The timeout to wait for the motion to finish.
   * @return Whether the motion finished before the timeout expired.
   */
  template <class Rep, class Period>
  bool joinMotion(const std::chrono::duration<Rep, Period> &timeout) {
    std::unique_lock lock(*control_mutex_);
    return joinMotionUnsafe<Rep, Period>(lock, timeout);
  }

  /**
   * @brief Check whether the robot is still in motion. This function is non-blocking and returns immediately. Throw any
   * exceptions that occurred during the motion.
   * @return Whether the robot is still in motion.
   */
  [[nodiscard]]
  bool pollMotion() {
    return joinMotion(std::chrono::milliseconds(0));
  }

  // These helper functions are needed as the implicit template deduction does not work on subclasses of Motion

  /**
   * @brief Execute the given motion
   * @param motion The motion to execute.
   * @param async Whether to execute the motion asynchronously.
   */
  void move(const std::shared_ptr<Motion<franka::CartesianPose>> &motion, bool async = false) {
    moveInternal<franka::CartesianPose>(
        motion, [this](const ControlFunc<franka::CartesianPose> &m) { control(m, params_.controller_mode); }, async);
  }

  /**
   * @brief Execute the given motion
   * @param motion The motion to execute.
   * @param async Whether to execute the motion asynchronously.
   */
  void move(const std::shared_ptr<Motion<franka::CartesianVelocities>> &motion, bool async = false) {
    moveInternal<franka::CartesianVelocities>(
        motion,
        [this](const ControlFunc<franka::CartesianVelocities> &m) { control(m, params_.controller_mode); },
        async);
  }

  /**
   * @brief Execute the given motion
   * @param motion The motion to execute.
   * @param async Whether to execute the motion asynchronously.
   */
  void move(const std::shared_ptr<Motion<franka::JointPositions>> &motion, bool async = false) {
    moveInternal<franka::JointPositions>(
        motion, [this](const ControlFunc<franka::JointPositions> &m) { control(m, params_.controller_mode); }, async);
  }

  /**
   * @brief Execute the given motion
   * @param motion The motion to execute.
   * @param async Whether to execute the motion asynchronously.
   */
  void move(const std::shared_ptr<Motion<franka::JointVelocities>> &motion, bool async = false) {
    moveInternal<franka::JointVelocities>(
        motion, [this](const ControlFunc<franka::JointVelocities> &m) { control(m, params_.controller_mode); }, async);
  }

  /**
   * @brief Execute the given motion
   * @param motion The motion to execute.
   * @param async Whether to execute the motion asynchronously.
   */
  void move(const std::shared_ptr<Motion<franka::Torques>> &motion, bool async = false) {
    moveInternal<franka::Torques>(motion, [this](const ControlFunc<franka::Torques> &m) { control(m); }, async);
  }

 private:
  std::shared_ptr<const Model> model_;
#ifdef FRANKA_0_15
  std::string model_urdf_;
#endif

  template <typename ControlSignalType>
  using ControlFunc = std::function<ControlSignalType(const franka::RobotState &, franka::Duration)>;
  using MotionGeneratorVariant = std::variant<
      std::nullopt_t, MotionGenerator<franka::Torques>, MotionGenerator<franka::JointVelocities>,
      MotionGenerator<franka::JointPositions>, MotionGenerator<franka::CartesianVelocities>,
      MotionGenerator<franka::CartesianPose>>;

  //! The robot's hostname / IP address
  std::string fci_hostname_;
  Params params_;
  RobotState current_state_;
  std::mutex state_mutex_;
  std::shared_ptr<std::mutex> control_mutex_;
  std::condition_variable control_finished_condition_;
  std::exception_ptr control_exception_;
  std::thread control_thread_;
  MotionGeneratorVariant motion_generator_{std::nullopt};
  bool motion_generator_running_{false};

  [[nodiscard]] bool is_in_control_unsafe() const;

 public:
  // IMPORTANT: this has to come after control_mutex_ as otherwise control_mutex_ will be uninitialized when passed to
  // the constructor of the DynamicsLimit class
  // clang-format off
  /**
   * @brief Translational velocity limit [m/s].
   */
  DynamicsLimit<double> translation_velocity_limit{
    "translational velocity", 1.7, 0.7, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
 * @brief Rotational velocity limit [rad/s].
 */
  DynamicsLimit<double> rotation_velocity_limit{
    "rotational velocity", 2.5, 2.5, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
   * @brief Elbow velocity limit [rad/s].
   */
  DynamicsLimit<double> elbow_velocity_limit{
    "elbow velocity", 2.175, 2.175, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
   * @brief Translational acceleration limit [m/s²].
   */
  DynamicsLimit<double> translation_acceleration_limit{
    "translational acceleration", 13.0, 2.0, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
   * @brief Rotational acceleration limit [rad/s²].
   */
  DynamicsLimit<double> rotation_acceleration_limit{
    "rotational acceleration", 25.0, 10.0, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
   * @brief Elbow acceleration limit [rad/s²].
   */
  DynamicsLimit<double> elbow_acceleration_limit{
    "elbow acceleration", 10.0, 4.0, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
   * @brief Translational jerk limit [m/s³].
   */
  DynamicsLimit<double> translation_jerk_limit{
    "translational jerk", 6500.0, 500.0, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
   * @brief Rotational jerk limit [rad/s³].
   */
  DynamicsLimit<double> rotation_jerk_limit{
    "rotational jerk", 12500.0, 2000.0, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
   * @brief Elbow jerk limit [rad/s³].
   */
  DynamicsLimit<double> elbow_jerk_limit{
    "elbow jerk", 5000.0, 800.0, control_mutex_, [this] { return is_in_control_unsafe(); }};

  /**
   * @brief Joint velocity limit [rad/s].
   */
#define MAX_JOINT_VEL toEigenD<7>({2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610})
  DynamicsLimit<Vector7d> joint_velocity_limit{
    "joint_velocity", MAX_JOINT_VEL, MAX_JOINT_VEL, control_mutex_,
    [this] { return is_in_control_unsafe(); }
  };

  /**
   * @brief Joint acceleration limit [rad/s²].
   */
#define MAX_JOINT_ACC toEigenD<7>({15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0})
  DynamicsLimit<Vector7d> joint_acceleration_limit{
    "joint_acceleration", MAX_JOINT_ACC, MAX_JOINT_ACC * 0.3, control_mutex_,
    [this] { return is_in_control_unsafe(); }
  };

  /**
   * @brief Joint jerk limit [rad/s³].
   */
#define MAX_JOINT_JERK toEigenD<7>({7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0})
  DynamicsLimit<Vector7d> joint_jerk_limit{
    "joint_jerk", MAX_JOINT_JERK, MAX_JOINT_JERK * 0.3, control_mutex_,
    [this] { return is_in_control_unsafe(); }
  };

  // clang-format on

 private:
  template <class Rep = long, class Period = std::ratio<1>>
  bool joinMotionUnsafe(
      std::unique_lock<std::mutex> &lock,
      const std::optional<std::chrono::duration<Rep, Period>> &timeout = std::nullopt) {
    while (motion_generator_running_) {
      if (timeout.has_value()) {
        if (control_finished_condition_.wait_for(lock, timeout.value()) == std::cv_status::timeout) {
          return false;
        }
      } else {
        control_finished_condition_.wait(lock);
      }
    }
    if (control_thread_.joinable()) control_thread_.join();
    if (control_exception_ != nullptr) {
      auto control_exception = control_exception_;
      control_exception_ = nullptr;
      std::rethrow_exception(control_exception);
    }
    return true;
  }

  template <typename ControlSignalType>
  void moveInternal(
      const std::shared_ptr<Motion<ControlSignalType>> &motion,
      const std::function<void(const ControlFunc<ControlSignalType> &)> &control_func_executor, bool async) {
    if (motion == nullptr) {
      throw std::invalid_argument("The motion must not be null.");
    }
    {  // Do not remove brace, it is needed to scope the lock
      std::unique_lock lock(*control_mutex_);
      if (is_in_control_unsafe() && motion_generator_running_) {
        if (!std::holds_alternative<MotionGenerator<ControlSignalType>>(motion_generator_)) {
          throw InvalidMotionTypeException(
              "The type of motion cannot change during runtime. Please ensure that the "
              "previous motion finished before using a new type of motion.");
        }
        std::get<MotionGenerator<ControlSignalType>>(motion_generator_).updateMotion(motion);
      } else {
        joinMotionUnsafe(lock);

        motion_generator_.emplace<MotionGenerator<ControlSignalType>>(this, motion);
        auto motion_generator = &std::get<MotionGenerator<ControlSignalType>>(motion_generator_);
        motion_generator->registerUpdateCallback(
            [this](const RobotState &robot_state, franka::Duration duration, franka::Duration time) {
              std::lock_guard lock(state_mutex_);
              current_state_ = robot_state;
            });
        motion_generator_running_ = true;
        control_thread_ = std::thread([this, control_func_executor, motion_generator]() {
          try {
            bool done = false;
            RobotStateEstimator robot_state_estimator(
                params_.dq_estimator_window_size,
                params_.ddq_estimator_window_size,
                params_.dq_estimator_alpha,
                params_.ddq_estimator_alpha);
            while (!done) {
              control_func_executor(
                  [this, motion_generator, &robot_state_estimator](const franka::RobotState &rs, franka::Duration d) {
                    return (*motion_generator)(robot_state_estimator.update(rs, *model_), d);
                  });
              std::unique_lock lock(*control_mutex_);

              // This code is just for the case that a new motion is set just after the old one terminates. If this
              // happens, we need to continue with this motion, unless an exception occurs.
              done = !motion_generator->has_new_motion();
              if (motion_generator->has_new_motion()) {
                motion_generator->resetTimeUnsafe();
              } else {
                done = true;
                motion_generator_running_ = false;
                control_finished_condition_.notify_all();
              }
            }
          } catch (...) {
            std::unique_lock lock(*control_mutex_);
            control_exception_ = std::current_exception();
            motion_generator_running_ = false;
            control_finished_condition_.notify_all();
          }
        });
      }
    }
    if (!async) joinMotion();
  }
};

}  // namespace franky
