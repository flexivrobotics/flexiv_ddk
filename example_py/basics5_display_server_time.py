#!/usr/bin/env python

"""basics5_display_server_time.py

This tutorial will check connection with the robot server and print server_time.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
from datetime import datetime

# Import Flexiv DDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivddk
# fmt: on


def convert_to_datetime_string(sec_since_epoch: int, nano_sec_since_full_sec: int,
                                 format: str = "%Y-%m-%d %H:%M:%S") -> str:
    # Convert second since epoch to local datetime object
    dt = datetime.fromtimestamp(sec_since_epoch)
    # Format time using strftime
    formatted_time = dt.strftime(format)
    # Format nano time width
    return f"{formatted_time}.{nano_sec_since_full_sec:09d}"


def print_server_time(client, logger, stop_event):
    """
    Print server_time data @ 1Hz.

    """

    while not stop_event.is_set():
        # Print server_time
        server_time = client.server_time()
        logger.info("Current server time:")
        print(convert_to_datetime_string(server_time.sec, server_time.nano_sec))
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
        "with the robot server and print primitive states.\n"
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
        target=print_server_time, args=[client, logger, stop_event]
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
