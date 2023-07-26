import argparse
import subprocess
import serial
import os
import re


PROMPT = r"root@\?:(.*) # "
EOL = "\r+\n"

TMP_FILE = "/tmp/file_transfer.bz2.b64"


def get_args():
    parser = argparse.ArgumentParser(description="Transfers file from device to host")

    parser.add_argument(
        "source_path",
        type=str,
        help="File from the device, which should be transferred"
    )
    parser.add_argument(
        "dest_path",
        type=str,
        help="Path to destination file on host"
    )
    parser.add_argument(
        "-p",
        "--portrx",
        required=False,
        default="/dev/ttyUSB0",
        help="host receiving port",
    )
    parser.add_argument(
        "-P",
        "--portc",
        required=False,
        default="/dev/ttyUSB1",
        help="host commandline port",
    )
    parser.add_argument(
        "-b",
        "--baudx",
        required=False,
        default=57600,
        type=int,
        help="transfer baudrate",
    )
    parser.add_argument(
        "-B",
        "--baudc",
        required=False,
        default=57600,
        type=int,
        help="commandline baudrate",
    )
    parser.add_argument(
        "-t",
        "--porttx",
        required=False,
        default="/dev/uart1",
        help="target transfer port",
    )

    return parser.parse_args()


def start_transition(console_port: str, baudrate: int, source_path: str, device_transfer_port: str):
    with serial.Serial(console_port, baudrate=baudrate, timeout=1) as device_console:
        command = f"sleep 1 ; bzip2 -k9fc {source_path} | base64 > {device_transfer_port}"

        device_console.write(bytes(command + "\n", "ascii"))
        device_console.readline()


def receive_data(receive_port: str, baudrate: int):
    with serial.Serial(receive_port, baudrate=baudrate, timeout=5) as receive:
        with open(TMP_FILE, "w", newline="") as file:
            while True:
                line = receive.readline().decode("ascii")

                if re.match(PROMPT, line) is not None or len(line) == 0:
                    break

                file.write(line.removesuffix("\r\n"))


def decode_data(dest_path: str):
    with open(dest_path, "w") as dest_file:
        ps = subprocess.Popen(["base64", "-d", TMP_FILE], stdout=subprocess.PIPE)
        subprocess.run(["bunzip2", "-fcq9"], stdin=ps.stdout, stdout=dest_file)

    os.remove(TMP_FILE)


def main() -> None:
    args = get_args()

    try:
        start_transition(args.portc, args.baudc, args.source_path, args.porttx)
    except Exception:
        print(f"Error while writing to {args.portc}")

    try:
        receive_data(args.portrx, args.baudx)
    except serial.SerialException as e:
        print(f"Error while writing to {args.portrx}: {e}")
    except IOError as e:
        print(f"Error while writing to file: {e}")

    try:
        decode_data(args.dest_path)
    except Exception as e:
        print(f"Error while decoding data: {e}")


if __name__ == "__main__":
    main()
