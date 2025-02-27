/**
 * @file data.hpp
 * @brief Header file containing various data structs.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DDK_DATA_HPP_
#define FLEXIV_DDK_DATA_HPP_

#include <array>
#include <ostream>
#include <string>
#include <variant>
#include <vector>
namespace flexiv {
namespace ddk {

/** Robot Cartesian-space degrees of freedom \f$ m \f$ */
constexpr size_t kCartDoF = 6;

/** Size of pose array (3 position + 4 quaternion) */
constexpr size_t kPoseSize = 7;

/** Number of digital IO ports (16 on control box + 2 inside the wrist
 * connector) */
constexpr size_t kIOPorts = 18;

/**
 * @struct JointStates
 * @brief Data structure containing the joint-space states of manipulator and
 * external axes (if any).
 */
struct JointStates {
  /**
   * Measured joint positions of the full system using link-side encoder: \f$ q
   * \in \mathbb{R}^{n \times 1} \f$. This is the direct measurement of joint
   * positions. Unit: \f$ [rad] or [m] \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   * @note If a joint has only one encoder, then \f$ \theta = q \f$.
   */
  std::vector<double> q = {};

  /**
   * Measured joint positions of the full system using motor-side encoder: \f$
   * \theta \in \mathbb{R}^{n \times 1} \f$. This is the indirect measurement of
   * joint positions. \f$ \theta = q + \Delta \f$, where \f$ \Delta \f$ is the
   * joint's internal deflection between motor and link. Unit: \f$ [rad] or [m]
   * \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   * @note If a joint has only one encoder, then \f$ \theta = q \f$.
   */
  std::vector<double> theta = {};

  /**
   * Measured joint velocities of the full system using link-side encoder: \f$
   * \dot{q} \in \mathbb{R}^{n \times 1} \f$. This is the direct but more noisy
   * measurement of joint velocities. Unit: \f$ [rad/s] or [m/s] \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   * @note If a joint has only one encoder, then \f$ \dot{\theta} = \dot{q} \f$.
   */
  std::vector<double> dq = {};

  /**
   * Measured joint velocities of the full system using motor-side encoder: \f$
   * \dot{\theta} \in \mathbb{R}^{n \times 1} \f$. This is the indirect but less
   * noisy measurement of joint velocities. Unit: \f$ [rad/s] or [m/s] \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   * @note If a joint has only one encoder, then \f$ \dot{\theta} = \dot{q} \f$.
   */
  std::vector<double> dtheta = {};

  /**
   * Measured joint torques of the full system: \f$ \tau \in \mathbb{R}^{n
   * \times 1} \f$. Unit: \f$ [Nm] \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   * @note If a joint has no torque measurement, then the corresponding value
   * will be 0.
   */
  std::vector<double> tau = {};

  /**
   * Numerical derivative of measured joint torques of the full system: \f$
   * \dot{\tau} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm/s] \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   * @note If a joint has no torque measurement, then the corresponding value
   * will be 0.
   */
  std::vector<double> tau_dot = {};

  /**
   * Estimated external joint torques of the full system: \f$ \hat \tau_{ext}
   * \in \mathbb{R}^{n \times 1} \f$. Produced by any external contact (with
   * robot body or end-effector) that does not belong to the known robot model.
   * Unit: \f$ [Nm] \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   * @note If a joint has no torque measurement, then the corresponding value
   * will be 0.
   */
  std::vector<double> tau_ext = {};
};

/**
 * @struct JointCommands
 * @brief Data structure containing the joint-space robot commands.
 */
struct JointCommands {
  /**
   * Desired joint positions of the full system using link-side encoder: \f$
   * q_{d} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] or [m] \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   */
  std::vector<double> q_des = {};

  /**
   * Desired joint velocities of the full system using link-side encoder: \f$
   * \dot{q}_{d} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] or [m/s]
   * \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   */
  std::vector<double> dq_des = {};

  /**
   * Desired joint torques of the full system: \f$ \tau_{d} \in \mathbb{R}^{n
   * \times 1} \f$. Compensation of nonlinear dynamics (gravity, centrifugal,
   * and Coriolis) is excluded. Unit: \f$ [Nm] \f$.
   * @note This contains values for both the external axes (if any) and the
   * robot manipulator. If there is an external axis, the joint data of the
   * external axis is in front of the vector, and the joint data of the
   * manipulator is appended at the end.
   * @note If a joint has no torque control capability, then the corresponding
   * value will be 0.
   */
  std::vector<double> tau_des = {};
};

/**
 * @struct CartesianStates
 * @brief Data structure containing the Cartesian-space robot states.
 */
struct CartesianStates {
  /**
   * Measured TCP pose expressed in world frame: \f$ ^{O}T_{TCP} \in
   * \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
   * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w,
   * q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
   */
  std::array<double, kPoseSize> tcp_pose = {};

  /**
   * Measured TCP velocity expressed in world frame: \f$ ^{O}\dot{X} \in
   * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
   * linear velocity and \f$ \mathbb{R}^{3 \times 1} \f$ angular velocity: \f$
   * [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$. Unit: \f$
   * [m/s]~[rad/s] \f$.
   */
  std::array<double, kCartDoF> tcp_vel = {};

  /**
   * Measured flange pose expressed in world frame: \f$ ^{O}T_{flange} \in
   * \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
   * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w,
   * q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
   */
  std::array<double, kPoseSize> flange_pose = {};

  /**
   * Force-torque (FT) sensor raw reading in flange frame: \f$ ^{flange}F_{raw}
   * \in \mathbb{R}^{6 \times 1} \f$. The value is 0 if no FT sensor is
   * installed. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
   * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T
   * \f$. Unit: \f$ [N]~[Nm] \f$.
   */
  std::array<double, kCartDoF> ft_sensor_raw = {};

  /**
   * Low-pass filtered estimated external wrench applied on TCP and expressed in
   * TCP frame: \f$ ^{TCP}F_{ext} \in \mathbb{R}^{6 \times 1} \f$. Consists of
   * \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$
   * moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
   */
  std::array<double, kCartDoF> ext_wrench_in_tcp = {};

  /**
   * Low-pass filtered estimated external wrench applied on TCP and expressed in
   * world frame: \f$ ^{O}F_{ext} \in \mathbb{R}^{6 \times 1} \f$. Consists of
   * \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$
   * moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
   */
  std::array<double, kCartDoF> ext_wrench_in_world = {};

  /**
   * Raw estimated external wrench applied on TCP and expressed in TCP frame:
   * \f$ ^{TCP}F_{ext} \in \mathbb{R}^{6 \times 1} \f$. Consists of \f$
   * \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$
   * moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
   */
  std::array<double, kCartDoF> ext_wrench_in_tcp_raw = {};

  /**
   * Raw estimated external wrench applied on TCP and expressed in world frame:
   * \f$ ^{O}F_{ext} \in \mathbb{R}^{6 \times 1} \f$. Consists of \f$
   * \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$
   * moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
   */
  std::array<double, kCartDoF> ext_wrench_in_world_raw = {};
};

/**
 * @struct CartesianCommands
 * @brief Data structure containing the Cartesian-space robot commands.
 */
struct CartesianCommands {
  /**
   * Desired TCP pose expressed in world frame: \f$ {^{O}T_{TCP}}_{d} \in
   * \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
   * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w,
   * q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
   */
  std::array<double, kPoseSize> tcp_pose_des = {};

  /**
   * Measured TCP velocity expressed in world frame: \f$ ^{O}\dot{X} \in
   * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
   * linear velocity and \f$ \mathbb{R}^{3 \times 1} \f$ angular velocity: \f$
   * [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$. Unit: \f$
   * [m/s]~[rad/s] \f$.
   */
  std::array<double, kCartDoF> tcp_vel_des = {};

  /**
   * Desired wrench applied on TCP and expressed in force control frame: \f$
   * ^{O}F_{des} \in \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3
   * \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x,
   * f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
   */
  std::array<double, kCartDoF> wrench_des_in_ctrl_frame = {};
};

/**
 * @struct Manipulability
 * @brief Data structure containing indicators of the robot’s manipulability.
 */
struct Manipulability {

  /**
   *  Score of the robot's current configuration {translation_score,
   * orientation_score}. It's the same as the configuration score shown in the
   * Elements. The quality of configuration based on score is mapped as: poor =
   * [0, 20), medium = [20, 40), good = [40, 100].
   */
  std::pair<double, double> configuration_score = {};

  /**
   * The robot’s current manipulability in translational directions: \f$ W_t
   * \f$. This is a scalar value that represents the robot's capability to
   * freely translate its end effector at the current joint configuration. A
   * higher manipulability measure signifies a broader range of potential
   * movements.
   */
  double translation = {};

  /**
   * The robot’s current manipulability in rotational directions: \f$ W_r \f$.
   * This is a scalar value that represents the robot's capability to freely
   * rotate its end effector at the current joint configuration. A higher
   * manipulability measure signifies a broader range of potential movements.
   */
  double rotation = {};

  /**
   * Gradient of the translational manipulability with respect to joint
   * positions: \f$ \frac{\partial {W_t}_{\text{trans}}}{\partial q} \in
   * \mathbb{R}^{n \times 1} \f$. A vector that represents how the translational
   * manipulability changes with small variations in the robot's joint
   * positions. It provides insight into how to modify joint configurations to
   * improve translational manipulability. Unit: \f$
   * [\text{rad}^{-1}] \f$.
   */
  std::vector<double> translation_gradient = {};

  /**
   * Gradient of the rotational manipulability with respect to joint positions:
   * \f$ \frac{\partial {W_r}_{\text{rot}}}{\partial q} \in \mathbb{R}^{n \times
   * 1} \f$. A vector that represents how the rotational manipulability changes
   * with small variations in the robot's joint positions. It provides insight
   * into how to modify joint configurations to improve rotational
   * manipulability. Unit: \f$ [\text{rad}^{-1}] \f$.
   */
  std::vector<double> rotation_gradient = {};
};

/**
 * @struct PlanInfo
 * @brief Data structure containing information of the on-going primitive/plan.
 */
struct PlanInfo {
  /** Current primitive name */
  std::string pt_name = {};

  /** Current node name */
  std::string node_name = {};

  /** Current node path */
  std::string node_path = {};

  /** Current node path time period */
  std::string node_path_time_period = {};

  /** Current node path number */
  std::string node_path_number = {};

  /** Assigned plan name */
  std::string assigned_plan_name = {};

  /** Velocity scale */
  double velocity_scale = {};

  /** Waiting for user signal to step the breakpoint */
  bool waiting_for_step = {};
};

/**
 * @struct ServerTime
 * @brief Data structure containing information of the time on server
 */
struct ServerTime {
  /** Current seconds since epoch */
  int sec = {};

  /** Number of nanoseconds since last full second */
  int nano_sec = {};
};

/** Alias of the variant that holds all possible types of flexiv primitive
 * states */
using FlexivPrimitiveStatesType = std::variant<int, double, std::string>;

/**
 * @brief Operator overloading to out stream all robot states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] joint_states JointStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream &operator<<(std::ostream &ostream,
                         const JointStates &joint_states);

/**
 * @brief Operator overloading to out stream all robot states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] cartesian_states CartesianStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream &operator<<(std::ostream &ostream,
                         const CartesianStates &cartesian_states);

/**
 * @brief Operator overloading to out stream all plan info in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] plan_info PlanInfo data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream &operator<<(std::ostream &ostream, const PlanInfo &plan_info);

/**
 * @brief Operator overloading to out stream server time in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param ostream Ostream instance.
 * @param server_time ServerTime data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream &operator<<(std::ostream &ostream, const ServerTime &server_time);

/**
 * @brief Operator overloading to out stream manipulability in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param ostream Ostream instance.
 * @param manipulability Manipulability data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream &operator<<(std::ostream &ostream,
                         const Manipulability &manipulability);

/**
 * @brief Operator overloading to out stream Cartesian commands in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param ostream Ostream instance.
 * @param cartesian_commands CartesianCommands data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream &operator<<(std::ostream &ostream,
                         const CartesianCommands &cartesian_commands);

/**
 * @brief Operator overloading to out stream joint commands in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param ostream Ostream instance.
 * @param joint_commands JointCommands data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream &operator<<(std::ostream &ostream,
                         const JointCommands &joint_commands);
} /* namespace ddk */
} /* namespace flexiv */

#endif /* FLEXIV_DDK_DATA_HPP_ */
