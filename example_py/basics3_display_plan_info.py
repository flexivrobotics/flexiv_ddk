#!/usr/bin/env python

"""basics3_display_plan_info.py

This tutorial will check connection with the robot server and print plan_info.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
import flexivddk # pip install flexivddk


def print_plan_info(client, logger, stop_event):
    """
    Print plan_info data @ 1Hz.

    """

    while not stop_event.is_set():
        # Print plan_info
        plan_info = client.plan_info()
        logger.info("Current plan info:")
        print(f"assigned_plan_name: {plan_info.assigned_plan_name}")
        print(f"pt_name: {plan_info.pt_name}")
        print(f"node_name: {plan_info.node_name}")
        print(f"node_path: {plan_info.node_path}")
        print(f"node_path_time_period: {plan_info.node_path_time_period}")
        print(f"node_path_number: {plan_info.node_path_number}")
        print(f"velocity_scale: {plan_info.velocity_scale}")
        print(f"waiting_for_step: {plan_info.waiting_for_step}")
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
        "with the robot server and print plan_info.\n"
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
        target=print_plan_info, args=[client, logger, stop_event]
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
