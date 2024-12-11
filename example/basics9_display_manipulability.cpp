/**
 * @example basics9_display_manipulability.cpp
 * This tutorial check connection with the robot and print received current
 * manipulability data.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */
#include <atomic>
#include <csignal>
#include <flexiv/ddk/client.hpp>
#include <flexiv/ddk/utility.hpp>
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>
namespace {
/** Atomic signal to stop periodic print tasks */
std::atomic<bool> keep_running(true);
} // namespace

/**
 * @brief Handler for user stop interaction
 */
void SignalHandler(int signal) {
  if (signal == SIGINT) {
    keep_running = false;
    spdlog::info("Signal received, stopping...");
  }
}

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

/** @brief Print robot manipulability data @ 1Hz */
void printManipulability(flexiv::ddk::Client &client) {
  while (keep_running.load()) {
    // Check connection with the robot
    if (!client.connected()) {
      spdlog::error("Cannot connect with robot, retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(5));
      continue;
    }
    // Print all robot commands in JSON format using the built-in ostream
    // operator overloading
    spdlog::info("Current manipulability:");
    std::cout << client.manipulability() << std::endl;
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
               "with the robot and print current manipulability.");

  // Setup signal handler for graceful exit
  std::signal(SIGINT, SignalHandler);

  try {
    // DDK Initialization
    // =========================================================================================
    // Instantiate DDK client interface
    flexiv::ddk::Client client(robot_sn);

    // Print Commands
    // =========================================================================================
    // Use std::thread to do scheduling so that this example can run on all OS
    std::thread low_priority_thread(
        std::bind(printManipulability, std::ref(client)));

    // Properly exit thread
    low_priority_thread.join();

  } catch (const std::exception &e) {
    spdlog::error(e.what());
    return 1;
  }

  return 0;
}
