/**
 * @file data.hpp
 * @brief Header file containing various data structs.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DDK_DATA_HPP_
#define FLEXIV_DDK_DATA_HPP_

#include <array>
#include <vector>
#include <string>
#include <ostream>

namespace flexiv {
namespace ddk {

/** Robot Cartesian-space degrees of freedom \f$ m \f$ */
constexpr size_t kCartDoF = 6;

/** Size of pose array (3 position + 4 quaternion) */
constexpr size_t kPoseSize = 7;

/**
 * @struct JointStates
 * @brief Data structure containing the joint-space robot states.
 */
struct JointStates
{
    /**
     * Measured joint positions using link-side encoder: \f$ q \in \mathbb{R}^{n \times 1} \f$.
     * This is the direct measurement of joint positions, preferred for most cases.
     * Unit: \f$ [rad] \f$.
     */
    std::vector<double> q = {};

    /**
     * Measured joint positions using motor-side encoder: \f$ \theta \in \mathbb{R}^{n \times 1}
     * \f$. This is the indirect measurement of joint positions. \f$ \theta = q + \Delta \f$, where
     * \f$ \Delta \f$ is the joint's internal deflection between motor and link.
     * Unit: \f$ [rad] \f$.
     */
    std::vector<double> theta = {};

    /**
     * Measured joint velocities using link-side encoder: \f$ \dot{q} \in \mathbb{R}^{n \times 1}
     * \f$. This is the direct but more noisy measurement of joint velocities.
     * Unit: \f$ [rad/s] \f$.
     */
    std::vector<double> dq = {};

    /**
     * Measured joint velocities using motor-side encoder: \f$ \dot{\theta} \in \mathbb{R}^{n \times
     * 1} \f$. This is the indirect but less noisy measurement of joint velocities, preferred for
     * most cases. Unit: \f$ [rad/s] \f$.
     */
    std::vector<double> dtheta = {};

    /**
     * Measured joint torques: \f$ \tau \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau = {};

    /**
     * Desired joint torques: \f$ \tau_{d} \in \mathbb{R}^{n \times 1} \f$. Compensation of
     * nonlinear dynamics (gravity, centrifugal, and Coriolis) is excluded. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_des = {};

    /**
     * Numerical derivative of measured joint torques: \f$ \dot{\tau} \in \mathbb{R}^{n \times 1}
     * \f$. Unit: \f$ [Nm/s] \f$.
     */
    std::vector<double> tau_dot = {};

    /**
     * Estimated external joint torques: \f$ \hat \tau_{ext} \in \mathbb{R}^{n \times 1} \f$.
     * Produced by any external contact (with robot body or end-effector) that does not belong to
     * the known robot model. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_ext = {};
};

/**
 * @struct CartesianStates
 * @brief Data structure containing the Cartesian-space robot states.
 */
struct CartesianStates
{
    /**
     * Measured TCP pose expressed in world frame: \f$ ^{O}T_{TCP} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose = {};

    /**
     * Desired TCP pose expressed in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose_des = {};

    /**
     * Measured TCP velocity expressed in world frame: \f$ ^{O}\dot{X} \in \mathbb{R}^{6 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$ \mathbb{R}^{3 \times
     * 1} \f$ angular velocity: \f$ [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$.
     * Unit: \f$ [m/s]~[rad/s] \f$.
     */
    std::array<double, kCartDoF> tcp_vel = {};

    /**
     * Measured flange pose expressed in world frame: \f$ ^{O}T_{flange} \in \mathbb{R}^{7 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     */
    std::array<double, kPoseSize> flange_pose = {};

    /**
     * Force-torque (FT) sensor raw reading in flange frame: \f$ ^{flange}F_{raw} \in \mathbb{R}^{6
     * \times 1} \f$. The value is 0 if no FT sensor is installed. Consists of \f$ \mathbb{R}^{3
     * \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y,
     * m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     */
    std::array<double, kCartDoF> ft_sensor_raw = {};

    /**
     * Estimated external wrench applied on TCP and expressed in TCP frame: \f$ ^{TCP}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]~[Nm] \f$.
     */
    std::array<double, kCartDoF> ext_wrench_in_tcp = {};

    /**
     * Estimated external wrench applied on TCP and expressed in world frame: \f$ ^{0}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]~[Nm] \f$.
     */
    std::array<double, kCartDoF> ext_wrench_in_world = {};
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
};

/**
 * @brief Operator overloading to out stream all robot states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] joint_states JointStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const JointStates& joint_states);

/**
 * @brief Operator overloading to out stream all robot states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] cartesian_states CartesianStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const CartesianStates& cartesian_states);

/**
 * @brief Operator overloading to out stream all plan info in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] plan_info PlanInfo data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const PlanInfo& plan_info);

} /* namespace ddk */
} /* namespace flexiv */

#endif /* FLEXIV_DDK_DATA_HPP_ */