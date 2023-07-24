# Phoenix-Pilot
#
# File transfer over serial utility
#
# Performs commandline actions on both transmitter and receiver of the file.
#
# Copyright 2023 Phoenix Systems
# Author: Mateusz Niewiadomski
#
# This file is part of Phoenix-Pilot.
#
#

import argparse
import signal
import sys
import serial
import subprocess
import os
import atexit
import time
import signal

p = None
args = None


def main():
    global p, args

    cmdParse = argparse.ArgumentParser()
    cmdParse.add_argument("file")
    cmdParse.add_argument(
        "-p",
        "--porttx",
        required=False,
        default="/dev/ttyUSB0",
        help="host transfer port",
    )
    cmdParse.add_argument(
        "-P",
        "--portc",
        required=False,
        default="/dev/ttyUSB1",
        help="host commandline port",
    )
    cmdParse.add_argument(
        "-b",
        "--baudx",
        required=False,
        default=57600,
        type=int,
        help="transfer baudrate",
    )
    cmdParse.add_argument(
        "-B",
        "--baudc",
        required=False,
        default=57600,
        type=int,
        help="commandline baudrate",
    )
    cmdParse.add_argument(
        "-r",
        "--portrx",
        required=False,
        default="/dev/uart1",
        help="target receiving port",
    )
    cmdParse.add_argument("-c", "--cancel", action="store_true")
    args = cmdParse.parse_args()

    # Open commandline port and configure target for receiving:
    # 1) configure target receiving port `portrx`
    # 2) send 1 second sleep
    # 3) send `rx` command
    # This script will reconfigure host transfer port and start `sx`
    # during the 1s sleep. `sx` will wait for `rx` readiness flag.
    ser = None
    try:
        ser = serial.Serial(args.portc, baudrate=args.baudc, timeout=1)
    except serial.SerialException:
        print(f"Error while opening {args.portc}")
        sys.exit(1)

    if args.cancel is False:
        ser.write(
            bytes(
                f"stty -F {args.portrx} {args.baudx} cs8 -parenb -cstopb -ixoff\n",
                "ascii",
            )
        )
        ser.write(bytes(f"sleep 1\n", "ascii"))
        ser.write(
            bytes(
                f"rx {os.path.basename(args.file)} < {args.portrx} > {args.portrx}\n",
                "ascii",
            )
        )

    # Close commandline port
    ser.close()

    # Configure host transfer port
    cmd_str = f"stty -F {args.porttx} {args.baudx} cs8 -parenb -cstopb -ixoff"
    subprocess.run(cmd_str, shell=True)

    cmd_str = f"sx -X {args.file} < {args.porttx} > {args.porttx}"
    p = subprocess.Popen(cmd_str, stdout=subprocess.PIPE, shell=True)

    return_code = p.wait()


def cleanup():
    global p

    if p is not None:
        p.send_signal(signal.SIGINT)


atexit.register(cleanup)

# Run rxfile
if __name__ == "__main__":
    main()
