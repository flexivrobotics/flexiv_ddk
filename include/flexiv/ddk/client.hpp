/**
 * @file client.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_DDK_CLIENT_HPP_
#define FLEXIV_DDK_CLIENT_HPP_

#include "data.hpp"
#include <exception>
#include <map>
#include <memory>
#include <vector>

namespace flexiv {
namespace ddk {

/**
 * @class Client
 * @brief Main interface to obtain data from the DDK server of the connected
 * robot.
 */
class Client {
public:
  /**
   * @brief [Blocking] Create an instance as the data distribution client. DDK
   * services will initialize and connection with the robot will be established.
   * @param[in] robot_sn Serial number of the robot to connect. The accepted
   * formats are: "Rizon 4s-123456" and "Rizon4s-123456".
   * @param[in] network_interface_whitelist Limit the network interface(s) that
   * can be used to try to establish connection with the specified robot. The
   * whitelisted network interface is defined by its associated IPv4 address.
   * For example, {"10.42.0.1", "192.168.2.102"}. If left empty, all available
   * network interfaces will be tried when searching for the specified robot.
   * @throw std::invalid_argument if the format of [robot_sn] is invalid.
   * @throw std::runtime_error if the initialization sequence failed.
   * @throw std::logic_error if the connected robot does not have a valid DDK
   * license; or this DDK library version is incompatible with the connected
   * robot.
   * @warning This constructor blocks until the initialization sequence is
   * successfully finished and connection with the robot is established.
   */
  Client(const std::string &robot_sn,
         const std::vector<std::string> &network_interface_whitelist = {});
  virtual ~Client();

  /**
   * @brief [Blocking] Check whether the client is connected to the robot.
   * @return True: connected, false: disconnected.
   */
  bool connected() const;

  /**
   * @brief [Non-blocking] Access the current joint-space robot states.
   * @return JointStates value copy.
   */
  const JointStates joint_states() const;

  /**
   * @brief [Non-blocking] Access the current Cartesian-space robot states.
   * @return CartesianStates value copy.
   */
  const CartesianStates cartesian_states() const;

  /**
   * @brief [Non-blocking] Access the current joint-space robot commands.
   * @return JointCommands value copy.
   */
  const JointCommands joint_commands() const;

  /**
   * @brief [Non-blocking] Access the current Cartesian-space robot commands.
   * @return CartesianCommands value copy.
   */
  const CartesianCommands cartesian_commands() const;

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
  const PlanInfo plan_info() const;

  /**
   * @brief [Blocking] State parameters of the executing primitive and their
   * current values.
   * @return A map of {pt_state_name, pt_state_value(s)}. Booleans are
   * represented by int 1 and 0. For example,
   * {{"primitiveName","MoveL"},{"reachedTarget", 0}, {"timePeriod", 5.6}}.
   * @throw std::runtime_error if failed to get a reply from the connected
   * robot.
   * @note This function blocks until a reply is received.
   */
  std::map<std::string, FlexivPrimitiveStatesType> primitive_states() const;

  /**
   * @brief [Non-blocking] Access the current time from server. It contains
   * current seconds since epoch and nanoseconds since last full second
   * @return ServerTime value copy.
   */
  const ddk::ServerTime &server_time(void) const;

  /**
   * @brief [Non-blocking] Whether the emergency stop is released.
   * @return True: released; false: pressed.
   */
  bool estop_released() const;

  /**
   * @brief [Non-blocking] Whether the enabling button is pressed.
   * @return True: pressed; false: released.
   */
  bool enabling_button_pressed() const;

  /**
   * @brief [Non-blocking] Read all digital input ports on the control box,
   * including 16 on the control box plus 2 inside the wrist connector.
   * @return Digital input readings array whose index corresponds to the digital
   * input port index. True: port high, false: port low.
   */
  const std::array<bool, ddk::kIOPorts> digital_inputs(void) const;

  /**
   * @brief [Blocking] Access the current manipulability.
   * @return Manipulability data copy.
   */
  const Manipulability manipulability() const;

private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

} /* namespace ddk */
} /* namespace flexiv */

#endif /* FLEXIV_DDK_CLIENT_HPP_ */
