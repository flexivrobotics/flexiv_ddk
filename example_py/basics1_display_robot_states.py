#!/usr/bin/env python

"""basics1_display_robot_states.py

This tutorial will check connection with the robot server and print robot states.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
import flexivddk  # pip install flexivddk


def print_robot_states(client, logger, stop_event):
    """
    Print robot states by joint group @ 1Hz.

    """

    while not stop_event.is_set():
        logger.info("Current robot states by joint group:")
        for group, states in client.states().items():
            print(f"{group}:")
            print(f"  q: {['%.2f' % value for value in states.q]}")
            print(f"  theta: {['%.2f' % value for value in states.theta]}")
            print(f"  dq: {['%.2f' % value for value in states.dq]}")
            print(f"  dtheta: {['%.2f' % value for value in states.dtheta]}")
            print(f"  tau: {['%.2f' % value for value in states.tau]}")
            print(f"  tau_dot: {['%.2f' % value for value in states.tau_dot]}")
            print(f"  tau_ext: {['%.2f' % value for value in states.tau_ext]}")
            print(f"  tau_interact: {['%.2f' % value for value in states.tau_interact]}")
            print(f"  temperature: {['%.2f' % value for value in states.temperature]}")
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
        "with the robot server and print robot states.\n"
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
        target=print_robot_states, args=[client, logger, stop_event]
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
