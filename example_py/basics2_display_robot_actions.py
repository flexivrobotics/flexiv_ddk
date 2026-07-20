#!/usr/bin/env python

"""basics2_display_robot_actions.py

This tutorial will check connection with the robot server and print robot actions.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
import flexivddk # pip install flexivddk


def print_robot_actions(client, logger, stop_event):
    """
    Print robot actions by joint group @ 1Hz.

    """

    while not stop_event.is_set():
        logger.info("Current robot actions by joint group:")
        for group, actions in client.actions().items():
            print(f"{group}:")
            print(f"  q_d: {['%.2f' % value for value in actions.q_d]}")
            print(f"  dq_d: {['%.2f' % value for value in actions.dq_d]}")
            print(f"  tau_d: {['%.2f' % value for value in actions.tau_d]}")
            print(f"  tcp_pose_d: {['%.2f' % value for value in actions.tcp_pose_d]}")
            print(f"  tcp_twist_d: {['%.2f' % value for value in actions.tcp_twist_d]}")
            print(f"  tcp_wrench_d: {['%.2f' % value for value in actions.tcp_wrench_d]}")
        print(flush=True)

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
        help="Serial number of the robot to connect. Remove any space, e.g. Enlight-L-123456",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial will check connection "
        "with the robot server and print robot actions.\n"
    )

    try:
        # Instantiate client interface
        # ==========================================================================================
        client = flexivddk.Client(args.robot_sn)

        if not client.connected():
            logger.warn("Cannot get connected with robot, retrying ...")
            if not client.connected():
                logger.error("Exiting ...")
                return 1
        logger.info(f"Connected with robot {args.robot_sn}")
    except Exception as e:
        # Print exception error message
        logger.error(str(e))

    # Thread for printing robot data
    # =============================================================================
    print_thread = threading.Thread(
        target=print_robot_actions, args=[client, logger, stop_event]
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
