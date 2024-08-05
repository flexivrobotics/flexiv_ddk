/**
 * @example basics1_display_joint_states.cpp
 * This tutorial print received robot joint states.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/ddk/client.hpp>
#include <flexiv/ddk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <thread>

/** @brief Print program usage help */
void PrintHelp() {
  // clang-format off
    std::cout << "Required arguments: [robot SN]" << std::endl;
    std::cout << "    robot SN: Serial number of the robot to connect to. "
                 "Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
  // clang-format on
}

/** @brief Print robot joint states data @ 1Hz */
void printRobotStates(flexiv::ddk::Client &client) {
  while (true) {
    // Print all robot states in JSON format using the built-in ostream operator
    // overloading
    spdlog::info("Current robot states:");
    std::cout << client.joint_states() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int main(int argc, char *argv[]) {
  // Program Setup
  // =============================================================================================
  // Parse parameters
  if (argc < 2 ||
      flexiv::rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
    PrintHelp();
    return 1;
  }
  // Serial number of the robot to connect to. Remove any space, for example:
  // Rizon4s-123456
  std::string robot_sn = argv[1];

  // Print description
  spdlog::info(">>> Tutorial description <<<\nThis tutorial check connection "
               "with the robot server and print received robot joint states.");

  try {
    // DDK Initialization
    // =========================================================================================
    // Instantiate DDK client interface
    flexiv::ddk::Client client(robot_sn);

    // Clear fault on the connected robot if any
    if (client.fault()) {
      spdlog::warn(
          "Fault occurred on the connected robot, trying to clear ...");
      // Try to clear the fault
      if (!client.ClearFault()) {
        spdlog::error("Fault cannot be cleared, exiting ...");
        return 1;
      }
      spdlog::info("Fault on the connected robot is cleared");
    }

    // Enable the robot, make sure the E-stop is released before enabling
    spdlog::info("Enabling robot ...");
    client.Enable();

    // Wait for the robot to become operational
    while (!client.operational()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    spdlog::info("Robot is now operational");

    // Print States
    // =========================================================================================
    // Use std::thread to do scheduling so that this example can run on all OS,
    // since not all OS support flexiv::rdk::Scheduler
    std::thread low_priority_thread(
        std::bind(printRobotStates, std::ref(robot)));

    // Properly exit thread
    low_priority_thread.join();

  } catch (const std::exception &e) {
    spdlog::error(e.what());
    return 1;
  }

  return 0;
}
