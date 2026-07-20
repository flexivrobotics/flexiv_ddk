/**
 * @file client.hpp
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DDK_CLIENT_HPP_
#define FLEXIV_DDK_CLIENT_HPP_

#include "data.hpp"
#include <exception>
#include <memory>
#include <vector>
#include <map>

namespace flexiv {
namespace ddk {

/**
 * @class Client
 * @brief Main interface to obtain data from the DDK server of the connected
 * robot.
 */
class Client
{
public:
    /**
     * @brief [Blocking] Create an instance as the data distribution client. DDK
     * services will initialize and connection with the robot will be established.
     * @param[in] robot_sn Serial number of the robot to connect. The accepted
     * format is: "Enlight-L-123456".
     * @param[in] verbose Enable/disable info and warning prints.
     * @throw std::invalid_argument if the format of [robot_sn] is invalid.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have a valid DDK
     * license; or this DDK library version is incompatible with the connected
     * robot.
     * @warning This constructor blocks until the initialization sequence is
     * successfully finished and connection with the robot is established.
     */
    Client(const std::string& robot_sn, bool verbose = true);
    virtual ~Client();

    /**
     * @brief [Blocking] Check whether the client is connected to the robot.
     * @return True: connected, false: disconnected.
     */
    bool connected() const;

    /**
     * @brief [Non-blocking] Joint groups that the connected robot has.
     * @return Existing joint groups mapped from enum value to string.
     */
    std::map<JointGroup, std::string> groups() const;

    /**
     * @brief [Non-blocking] Current states data of all existing joint groups of the robot.
     * @return Robot states data mapped by joint group.
     * @warning Cartesian states of non-single-arm joint groups are not populated.
     */
    std::map<JointGroup, RobotStates> states() const;

    /**
     * @brief [Non-blocking] Current actions data of all existing joint groups of the robot.
     * @return Robot actions data mapped by joint group.
     * @warning Cartesian actions of non-single-arm joint groups are not populated.
     */
    std::map<JointGroup, RobotActions> actions() const;

    /**
     * @brief [Blocking] States data of the primitive(s) that are currently running on each joint
     * group.
     * @return A map of JointGroup to PrimitiveStates. Only contains joint groups that exist.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    std::map<JointGroup, PrimitiveStates> primitive_states() const;

    /**
     * @brief [Blocking] Score of each joint group's current configuration (posture), calculated
     * from the manipulability measurements.
     * @return Configuration score mapped by joint group as {translation_score,
     * orientation_score}. The quality of configuration based on the score can be interpreted as:
     * poor = [0, 20), medium = [20, 40), good = [40, 100].
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     * @warning A poor configuration score means the robot is near or at singularity, which can lead
     * to degraded Cartesian performance. Use configuration with high scores for better
     * manipulability and task results.
     */
    std::map<JointGroup, std::pair<double, double>> configuration_score() const;

    /**
     * @brief [Blocking] Get detailed information about the currently executing
     * plan. Contains information like plan name, primitive name, node name, node
     * path, node path time period, etc.
     * @return PlanInfo data struct.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to get a reply from the connected
     * robot.
     * @note This function blocks until a reply is received.
     */
    PlanInfo plan_info() const;

    /**
     * @brief [Non-blocking] Whether all connected emergency stops (E-stops) are released.
     * @return True: released; false: pressed.
     */
    bool estop_released() const;

    /**
     * @brief [Non-blocking] Whether the 3-position enabling device (e.g. enabling button on the
     * motion bar) is in the ON position (the middle position).
     * @return True: ON; false: OFF.
     */
    bool enabling_device_on() const;

    /**
     * @brief [Non-blocking] Read all digital input ports on the control box, including 16 on the
     * control box plus 2 inside the wrist connector.
     * @return Digital input readings array whose index corresponds to the digital
     * input port index. True: port high, false: port low.
     */
    std::array<bool, ddk::kIOPorts> digital_inputs(void) const;

    /**
     * @brief [Non-blocking] Current reading from all digital output ports, including 16 on the
     * control box plus 2 in each wrist connector.
     * @return A boolean array whose index corresponds to that of the digital output ports.
     * True: port high; false: port low.
     */
    std::array<bool, kIOPorts> digital_outputs() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace ddk */
} /* namespace flexiv */

#endif /* FLEXIV_DDK_CLIENT_HPP_ */
