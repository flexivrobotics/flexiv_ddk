#!/usr/bin/env python

"""basics1_display_joint_states.py

This tutorial will check connection with the robot server and print joint states.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
import flexivddk # pip install flexivddk 


def print_joint_states(client, logger, stop_event):
    """
    Print joint states data @ 1Hz.

    """

    while not stop_event.is_set():
        # Print joint states, round all float values to 2 decimals
        logger.info("Current robot joint states:")
        # fmt: off
        print("{")
        print(f"q: {['%.2f' % i for i in client.joint_states().q]}",)
        print(f"theta: {['%.2f' % i for i in client.joint_states().theta]}")
        print(f"dq: {['%.2f' % i for i in client.joint_states().dq]}")
        print(f"dtheta: {['%.2f' % i for i in client.joint_states().dtheta]}")
        print(f"tau: {['%.2f' % i for i in client.joint_states().tau]}")
        print(f"tau_dot: {['%.2f' % i for i in client.joint_states().tau_dot]}")
        print(f"tau_ext: {['%.2f' % i for i in client.joint_states().tau_ext]}")
        print(f"tau_interact: {['%.2f' % i for i in client.joint_states().tau_interact]}")
        print(f"temperature: {['%.2f' % i for i in client.joint_states().temperature]}")
        print("}", flush= True)
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
        "with the robot server and print robot joint states.\n"
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
        target=print_joint_states, args=[client, logger, stop_event]
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
