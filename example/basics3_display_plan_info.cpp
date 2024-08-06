/**
 * @example basics3_display_cartesian_states.cpp
 * This tutorial check connection with the robot and print current running plan info.
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
            // Check connection with the robot
    if (!client.connected()) {
        throw std::runtime_error("Can not connect with robot, exiting ...");
      }
    // Print all robot states in JSON format using the built-in ostream operator
    // overloading
    spdlog::info("Current robot plan info:");
    std::cout << client.plan_info() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int main(int argc, char *argv[]) {
  // Program Setup
  // =============================================================================================
  // Parse parameters
  if (argc < 2 ||
      flexiv::ddk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
    PrintHelp();
    return 1;
  }
  // Serial number of the robot to connect to. Remove any space, for example:
  // Rizon4s-123456
  std::string robot_sn = argv[1];

  // Print description
  spdlog::info(">>> Tutorial description <<<\nThis tutorial check connection "
               "with the robot and print plan info.");

  try {
    // DDK Initialization
    // =========================================================================================
    // Instantiate DDK client interface
    flexiv::ddk::Client client(robot_sn);

    // Print States
    // =========================================================================================
    // Use std::thread to do scheduling so that this example can run on all OS
        std::thread low_priority_thread(
        std::bind(printRobotStates, std::ref(client)));

    // Properly exit thread
    low_priority_thread.join();

  } catch (const std::exception &e) {
    spdlog::error(e.what());
    return 1;
  }

  return 0;
}
