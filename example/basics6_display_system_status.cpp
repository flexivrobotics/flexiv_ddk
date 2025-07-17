/**
 * @example basics6_display_system_status.cpp
 * This tutorial check connection with the robot and print current system
 * status.
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

/** @brief Print system status @ 1Hz */
void printSystemStatus(flexiv::ddk::Client &client) {
  while (keep_running.load()) {
    // Check connection with the robot
    if (!client.connected()) {
      spdlog::error("Cannot connect with robot, retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(5));
      continue;
    }
    // Print current system status
    spdlog::info("Current system status:");

    std::cout << "E-stop released states: " << client.estop_released()
              << std::endl;

    std::cout << "Enabling button released states: "
              << client.enabling_button_pressed() << std::endl;
    std::cout << "Digital inputs: ";
    for (size_t i = 0; i < flexiv::ddk::kIOPorts; i++) {
      std::cout << client.digital_inputs()[i] << " ";
    }
    std::cout << "\n";

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
               "with the robot and print system status.");

  // Setup signal handler for graceful exit
  std::signal(SIGINT, SignalHandler);

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
