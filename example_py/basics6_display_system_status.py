#!/usr/bin/env python

"""basics5_display_system_status.py

This tutorial will check connection with the robot server and print system_status.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
import flexivddk # pip install flexivddk


def print_system_status(client, logger, stop_event):
    """
    Print system_status data @ 1Hz.

    """

    while not stop_event.is_set():
        # Print system_status
        logger.info("Current system status:")
        print(f"estop_released: {client.estop_released()}")
        print(f"enabling_button_pressed: {client.enabling_button_pressed()}")
        print(f"digital_inputs: {client.digital_inputs()}")
        print("", flush=True)
        # fmt: on

        time.sleep(1)


def main():
    # Create an event to signal the thread to stop
    stop_event = threading.Event()

    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial will check connection "
        "with the robot server and print system_status.\n"
    )

    try:
        # Instantiate client interface
        # ==========================================================================================
        client = flexivddk.Client(args.robot_sn)

        if not client.connected():
            logger.warn("Cannot get connected with robot, retrying ...")
            if not client.connected()():
                logger.error("Exiting ...")
                return 1
        logger.info(f"Connected with robot {args.robot_sn}")
    except Exception as e:
        # Print exception error message
        logger.error(str(e))

    # Thread for printing robot data
    # =============================================================================
    print_thread = threading.Thread(
        target=print_system_status, args=[client, logger, stop_event]
    )
    print_thread.start()

    # Use main thread to catch keyboard interrupt and exit thread
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        # Send signal to exit thread
        logger.info("Stopping print thread")
        stop_event.set()

    # Wait for thread to exit
    print_thread.join()
    logger.info("Print thread exited")


if __name__ == "__main__":
    main()
