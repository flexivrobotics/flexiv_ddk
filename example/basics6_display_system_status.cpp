/**
 * @example basics6_display_system_status.cpp
 * This tutorial check connection with the robot and print current system
 * status.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/ddk/client.hpp>
#include <flexiv/ddk/utility.hpp>
#include <iostream>
#include <spdlog/spdlog.h>
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

/** @brief Print system status @ 1Hz */
void printSystemStatus(flexiv::ddk::Client &client) {
  while (true) {
    // Check connection with the robot
    if (!client.connected()) {
      throw std::runtime_error("Can not connect with robot, exiting ...");
    }
    // Print current system status
    spdlog::info("Current system status:");

    std::cout << "E-stop released states: " << client.estop_released()
              << std::endl;

    std::cout << "Enabling button released states: "
              << client.enabling_button_pressed() << std::endl;

    std::cout << "Current system states: "
              << getSystemStateName(client.system_state()) << std::endl;

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
               "with the robot and print plan infos.");

  try {
    // DDK Initialization
    // =========================================================================================
    // Instantiate DDK client interface
    flexiv::ddk::Client client(robot_sn);

    // Print States
    // =========================================================================================
    // Use std::thread to do scheduling so that this example can run on all OS
    std::thread low_priority_thread(
        std::bind(printSystemStatus, std::ref(client)));

    // Properly exit thread
    low_priority_thread.join();

  } catch (const std::exception &e) {
    spdlog::error(e.what());
    return 1;
  }

  return 0;
}