/**
 * @file data.hpp
 * @brief Header file containing various data structs.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DDK_DATA_HPP_
#define FLEXIV_DDK_DATA_HPP_

#include <array>
#include <ostream>
#include <string>
#include <vector>
#include <variant>
#include <map>

namespace flexiv {
namespace ddk {

/** Robot Cartesian-space degrees of freedom \f$ m \f$ */
constexpr size_t kCartDoF = 6;

/** Size of pose array (3 position + 4 quaternion) */
constexpr size_t kPoseSize = 7;

/** Number of digital IO ports: 16 on the control box + 2 in each wrist connector * maximum 2 wrists
 */
constexpr size_t kIOPorts = 16 + 2 * 2;

/**
 * @brief All possible joint groups of the robot.
 */
enum class JointGroup
{
    UNKNOWN = -1, ///< Unknown group
    ALL = 0,      ///< The full system, including all actuated joints
    ARMS = 2,     ///< The dual arms as a whole, only applicable to dual-arm robots
    ARM_1 = 3,    ///< The 1st single arm in a dual-arm robot or the only arm in a single-arm robot
    ARM_2 = 4,    ///< The 2nd single arm in a dual-arm robot, not applicable to single-arm robots
    EXT_AXIS = 5, ///< External axis(es) for workspace extension

    FIRST = ALL,
    LAST = EXT_AXIS,
};

/** Map JointGroup enums to strings */
inline const std::map<JointGroup, std::string> kJointGroupNames {
    {JointGroup::UNKNOWN, "UNKNOWN"},
    {JointGroup::ALL, "ALL"},
    {JointGroup::ARMS, "ARMS"},
    {JointGroup::ARM_1, "ARM_1"},
    {JointGroup::ARM_2, "ARM_2"},
    {JointGroup::EXT_AXIS, "EXT_AXIS"},
};

/**
 * @struct RobotStates
 * @brief Robot states data in joint and Cartesian space for a joint group.
 */
struct RobotStates
{
    /** Current time since epoch of the robot system. The pair consists of {seconds since epoch,
     * nanoseconds since last full second} */
    std::pair<int, int> timestamp = {};

    /**
     * Measured joint positions from the link-side encoder: \f$ q \in \mathbb{R}^{n \times 1} \f$.
     * This is the direct measurement of joint positions. Unit: \f$ [rad] or [m] \f$.
     * @note If a joint has only one encoder, then \f$ \theta = q \f$.
     */
    std::vector<double> q = {};

    /**
     * Measured joint positions from the motor-side encoder: \f$ \theta \in \mathbb{R}^{n \times 1}
     * \f$. This is the indirect measurement of joint positions. \f$ \theta = q + \Delta \f$, where
     * \f$ \Delta \f$ is the joint's internal deflection between motor and link. Unit: \f$ [rad] or
     * [m] \f$.
     * @note If a joint has only one encoder, then \f$ \theta = q \f$.
     */
    std::vector<double> theta = {};

    /**
     * Measured joint velocities from the link-side encoder: \f$ \dot{q} \in \mathbb{R}^{n \times
     * 1} \f$. This is the direct but more noisy measurement of joint velocities. Unit: \f$ [rad/s]
     * or [m/s] \f$.
     * @note If a joint has only one encoder, then \f$ \dot{\theta} = \dot{q} \f$.
     */
    std::vector<double> dq = {};

    /**
     * Measured joint velocities from the motor-side encoder: \f$ \dot{\theta} \in \mathbb{R}^{n
     * \times 1} \f$. This is the indirect but less noisy measurement of joint velocities. Unit: \f$
     * [rad/s] or [m/s] \f$.
     * @note If a joint has only one encoder, then \f$ \dot{\theta} = \dot{q} \f$.
     */
    std::vector<double> dtheta = {};

    /**
     * Measured joint torques: \f$ \tau \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     * @note If a joint has no torque measurement, the corresponding value will be 0.
     */
    std::vector<double> tau = {};

    /**
     * Numerical derivative of measured joint torques: \f$ \dot{\tau} \in \mathbb{R}^{n \times 1}
     * \f$. Unit: \f$ [Nm/s] \f$.
     * @note If a joint has no torque measurement, the corresponding value will be 0.
     */
    std::vector<double> tau_dot = {};

    /**
     * Estimated external joint torques: \f$ \hat \tau_{ext} \in \mathbb{R}^{n \times 1} \f$.
     * Produced by any external contact (with robot body or end-effector) that does not belong to
     * the known robot model. Unit: \f$ [Nm] \f$.
     * @note If a joint has no torque measurement, the corresponding value will be 0.
     */
    std::vector<double> tau_ext = {};

    /**
     * Estimated interaction joint torques: \f$ \hat \tau_{int} \in \mathbb{R}^{n \times 1} \f$.
     * Produced by any interaction forces at the TCP. Unit: \f$ [Nm] \f$.
     * @note If a joint has no torque measurement, the corresponding value will be 0.
     */
    std::vector<double> tau_interact = {};

    /**
     * Measured joint temperatures: \f$ temp \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [°C] \f$.
     * @note If a joint has no temperature measurement, the corresponding value will be 0.
     */
    std::vector<double> temperature = {};

    /**
     * Measured flange pose w.r.t. world frame: \f$ ^{O}T_{flange} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> flange_pose = {};

    /**
     * Measured TCP pose w.r.t. world frame: \f$ ^{O}T_{TCP} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose = {};

    /**
     * Measured TCP twist w.r.t. world frame: \f$ ^{O}\dot{X} \in \mathbb{R}^{6 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$ \mathbb{R}^{3 \times 1}
     * \f$ angular velocity: \f$ [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$. Unit: \f$
     * [m/s]:[rad/s] \f$.
     */
    std::array<double, kCartDoF> tcp_twist = {};

    /**
     * Measured or estimated external wrench applied on TCP w.r.t. world frame: \f$ ^{O}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> tcp_wrench = {};

    /**
     * Measured or estimated external wrench applied on TCP w.r.t. local frame: \f$ ^{TCP}F_{ext}
     * \in \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> tcp_wrench_local = {};

    /**
     * Unfiltered tcp_wrench. The data is more noisy but has no filter latency.
     */
    std::array<double, kCartDoF> raw_tcp_wrench = {};

    /**
     * Unfiltered tcp_wrench_local. The data is more noisy but has no filter latency.
     */
    std::array<double, kCartDoF> raw_tcp_wrench_local = {};

    /**
     * Raw reading from the force-torque (FT) sensor w.r.t. flange frame: \f$ ^{flange}F_{raw} \in
     * \mathbb{R}^{6 \times 1} \f$. The value is 0 if no FT sensor is installed. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y,
     * f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> raw_ft_sensor = {};
};

/**
 * @struct RobotActions
 * @brief Robot actions data in joint and Cartesian space for a joint group.
 */
struct RobotActions
{
    /** Current time since epoch of the robot system. The pair consists of {seconds since epoch,
     * nanoseconds since last full second} */
    std::pair<int, int> timestamp = {};

    /**
     * Desired joint positions: \f$ q_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] or [m] \f$.
     */
    std::vector<double> q_d = {};

    /**
     * Desired joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] or
     * [m/s] \f$.
     */
    std::vector<double> dq_d = {};

    /**
     * Desired joint torques excluding the compensation of nonlinear dynamics: \f$ \tau_d \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     * @note Nonlinear dynamics include: gravity, centrifugal, and Coriolis. If the robot is static,
     * tau_d will be zeros.
     * @note If a joint has no torque control capability, the corresponding value will be 0.
     */
    std::vector<double> tau_d = {};

    /**
     * Desired TCP pose w.r.t. world frame: \f$ {^{O}T_{TCP}}_d \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose_d = {};

    /**
     * Desired TCP twist w.r.t. world frame: \f$ {^{O}\dot{X}}_d \in \mathbb{R}^{6 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$ \mathbb{R}^{3 \times 1}
     * \f$ angular velocity: \f$ [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$. Unit: \f$
     * [m/s]:[rad/s] \f$.
     */
    std::array<double, kCartDoF> tcp_twist_d = {};

    /**
     * Desired TCP wrench w.r.t. the current force control frame: \f$ {^{ctrl}F_{ext}}_d \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$
     * [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> tcp_wrench_d = {};
};

/** Alias of the variant that holds all possible types of flexiv primitive states */
using FlexivPrimitiveStatesType = std::variant<int, double, std::string>;

/**
 * @struct PrimitiveStates
 * @brief States data of a primitive.
 * @see Client::primitive_states().
 */
struct PrimitiveStates
{
    /** Name of the currently running primitive */
    std::string pt_name = {};

    /** Names and corresponding values of the primitive's states. Booleans are represented by int 1
     * and 0. For example:
     * {{"reachedTarget", 1}, {"timePeriod", 5.6}, {"forceOffset", {0.1, 0.2, -1.3}}}.
     */
    std::map<std::string, FlexivPrimitiveStatesType> names_and_values = {};
};

/**
 * @struct PlanInfo
 * @brief Data structure containing information of the on-going primitive/plan.
 */
struct PlanInfo
{
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
 * @brief Operator overloading to out stream all members of RobotStates in JSON format.
 * @param[in] ostream Ostream instance.
 * @param[in] robot_states RobotStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const RobotStates& robot_states);

/**
 * @brief Operator overloading to out stream all members of RobotActions in JSON format.
 * @param[in] ostream Ostream instance.
 * @param[in] robot_actions RobotActions data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const RobotActions& robot_actions);
/**
 * @brief Operator overloading to out stream all plan info in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] plan_info PlanInfo data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const PlanInfo& plan_info);

/**
 * @brief Operator overloading to out stream configuration_score in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param ostream Ostream instance.
 * @param configuration_score configuration_score data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream,
    const std::map<JointGroup, std::pair<double, double>>& configuration_score);

/**
 * @brief Provide custom std::ostream& operator<< for JointGroup
 * @param ostream Ostream instance.
 * @param group JointGroup data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const JointGroup& group);

/**
 * @brief Operator overloading to out stream a map of JointGroup to any data type in JSON format
 * @tparam T Type of the values in the map.
 * @param ostream Ostream instance.
 * @param map Map of JointGroup to any data type.
 * @return Updated ostream instance.
 */
template <typename T>
std::ostream& operator<<(std::ostream& ostream, const std::map<JointGroup, T>& map);

} /* namespace ddk */
} /* namespace flexiv */

#endif /* FLEXIV_DDK_DATA_HPP_ */
