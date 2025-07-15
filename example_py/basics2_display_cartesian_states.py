#!/usr/bin/env python

"""basics2_display_cartesian_states.py

This tutorial will check connection with the robot server and print cartesian states.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
import flexivddk # pip install flexivddk 



def print_cartesian_states(client, logger, stop_event):
    """
    Print cartesian states data @ 1Hz.

    """

    while not stop_event.is_set():
        # Print cartesian states, round all float values to 2 decimals
        logger.info("Current robot cartesian states:")
        # fmt: off
        print("{")
        print(f"tcp_pose: {['%.2f' % i for i in client.cartesian_states().tcp_pose]}")
        print(f"tcp_velocity: {['%.2f' % i for i in client.cartesian_states().tcp_vel]}")
        print(f"flange_pose: {['%.2f' % i for i in client.cartesian_states().flange_pose]}")
        print(f"ft_sensor_raw: {['%.2f' % i for i in client.cartesian_states().ft_sensor_raw]}")
        print(f"ext_wrench_in_tcp: {['%.2f' % i for i in client.cartesian_states().ext_wrench_in_tcp]}")
        print(f"ext_wrench_in_world: {['%.2f' % i for i in client.cartesian_states().ext_wrench_in_world]}")
        print(f"ext_wrench_in_tcp_raw: {['%.2f' % i for i in client.cartesian_states().ext_wrench_in_tcp_raw]}")
        print(f"ext_wrench_in_world_raw: {['%.2f' % i for i in client.cartesian_states().ext_wrench_in_world_raw]}")
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
        "with the robot server and print robot cartesian states.\n"
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
        target=print_cartesian_states, args=[client, logger, stop_event]
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
