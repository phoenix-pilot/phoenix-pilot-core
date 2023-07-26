import pexpect
import argparse
import subprocess
import os
import re


PROMPT = r"root@\?:(.*) # "
EOL = "\r+\n"

TMP_FILE = "/tmp/file_transfer.bz2.b64"


def get_args():
    parser = argparse.ArgumentParser(description="Transfers file from device to host")

    parser.add_argument("source_path", type=str,
                        help="File from the device, which should be transferred")
    parser.add_argument("dest_path", type=str,
                        help="Path to destination file on host")
    parser.add_argument("-b", "--baud", type=int, required=True, dest="baudrate",
                        help="Baudrate")
    parser.add_argument("-t", "--tty", type=str, required=True, dest="tty",
                        help="Path to TTY port device")

    return parser.parse_args()


def main() -> None:
    args = get_args()

    device = pexpect.spawn(f"picocom -b {args.baudrate} --imap lfcrlf {args.tty}", encoding="utf-8")
    device.expect("Terminal ready")

    device.sendline()
    device.expect(PROMPT, timeout=3)

    if device.match.group(1) != "~":
        device.sendline("cd ~")
        device.expect(PROMPT)

    with open(TMP_FILE, "w", newline="") as file:
        command = f"bzip2 -k9fc {args.source_path} | base64"
        device.sendline(command)

        espaced = re.escape(command)
        device.expect(f"{espaced}{EOL}")

        while (device.expect([f"{EOL}", PROMPT]) == 0):
            file.write(f"{device.before}")

    with open(args.dest_path, "w") as dest_file:
        ps = subprocess.Popen(["base64", "-di", TMP_FILE], stdout=subprocess.PIPE)
        subprocess.run(["bunzip2", "-fcq9"], stdin=ps.stdout, stdout=dest_file)

    os.remove(TMP_FILE)

    device.terminate()


if __name__ == "__main__":
    main()
