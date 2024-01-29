import argparse
import serial
import sys
import re
import base64
from bz2 import BZ2Decompressor
import hashlib


PROMPT = r"root@\?:(.*) # "


class StatusPrinter:
    def __init__(self, file_size) -> None:
        self.file_size = file_size
        self.i = 0
        self.print_suffix = ".  "

    def print(self, received_bytes):
        self.i = (self.i + 1) % 101

        if self.i == 0 or received_bytes == self.file_size:
            proc = received_bytes/self.file_size * 100.0
            print("\r{0:.2f} % {1}".format(proc, self.print_suffix), end="")
            self._next_suffix()

    def _next_suffix(self):
        if self.print_suffix == "   ":
            self.print_suffix = ".  "

        elif self.print_suffix == ".  ":
            self.print_suffix =   ".. "

        elif self.print_suffix == ".. ":
            self.print_suffix =   "..."

        elif self.print_suffix == "...":
            self.print_suffix =   "   "




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



def get_prompt_on_device(serial: serial):
    serial.write(bytes("cd .\n", "ascii"))

    # Catching new line form echo
    line = serial.readline().decode("ascii")

    # Catching prompt
    line = serial.readline().decode("ascii")

    if re.match(PROMPT, line) is None:
        raise Exception("cannot get prompt on device")


def get_file_size(serial: serial, source_path: str) -> int:
    command = f"stat -c %s {source_path}\n"
    serial.write(bytes(command, "ascii"))

    # Catching command echo
    line = serial.readline().decode("ascii")

    # Returned string
    line = serial.readline().decode("ascii")

    return int(line)


def get_file_hash(serial: serial, source_path: str):
    command = f"sha256sum {source_path}\n"
    serial.write(bytes(command, "ascii"))

    # Command echo
    line = serial.readline().decode("ascii")

    # Command output
    line = serial.readline().decode("ascii")

    # Parsing hash
    match = re.match(f"([0-9a-f]+)\s+{source_path}", line)
    if not match:
        raise Exception("cannot get file hash")

    return match.group(1)


def start_transition(serial: serial, source_path: str, device_transfer_port: str):
    command = f"bzip2 -k1fc {source_path} | base64 > {device_transfer_port}\n"
    serial.write(bytes(command, "ascii"))

    # Catch command echo
    serial.readline().decode("ascii")


def receive_data(receive_port: str, baudrate: int, dest_path: str, bytes):
    decompressor = BZ2Decompressor()
    hashCalc = hashlib.sha256()
    received = 0
    printer = StatusPrinter(bytes)

    with serial.Serial(receive_port, baudrate=baudrate, timeout=5) as receive:
        with open(dest_path, "wb") as dest_file:
            while True:
                line = receive.readline().decode("ascii")

                if re.match(PROMPT, line) is not None or len(line) == 0:
                    break

                line = line.removesuffix("\r\n")
                data = base64.b64decode(line)

                decomp_data = decompressor.decompress(data)
                data = b''

                hashCalc.update(decomp_data)
                dest_file.write(decomp_data)
                received += len(decomp_data)

                printer.print(received)

                if decompressor.eof:
                    return hashCalc.hexdigest()


def main() -> None:
    args = get_args()

    with serial.Serial(args.portc, args.baudc, timeout=5) as consoleSerial:

        try:
            get_prompt_on_device(consoleSerial)
        except Exception:
            print("Cannot get prompt on device")
            sys.exit()

        originalHash = get_file_hash(consoleSerial, args.source_path)

        try:
            file_size = get_file_size(consoleSerial, args.source_path)
        except Exception:
            print("Cannot get target file size")
            sys.exit()

        try:
            start_transition(consoleSerial, args.source_path, args.porttx)
        except Exception:
            print(f"Error while writing to {args.portc}")
            sys.exit()

    try:
        transferredFileHash = receive_data(args.portrx, args.baudx, args.dest_path, file_size)
    except serial.SerialException as e:
        print(f"Error while writing to {args.portrx}: {e}")
        sys.exit()
    except IOError as e:
        print(f"Error while writing to file: {e}")
        sys.exit()


    if originalHash == transferredFileHash:
        print("Successful file transfer")
    else:
        print("Error during file transfer. file {args.des_path} can be corrupted")



if __name__ == "__main__":
    main()
