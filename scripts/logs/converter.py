import argparse

from common.formats import FormatFactory


def get_args():
    arg_parser = argparse.ArgumentParser(description="Ekf logs converter. Supported types: csv, binary.")
    arg_parser.add_argument("-i", "--input",
                            type=str,
                            help="Input file path",
                            dest="input_file",
                            required=True)
    arg_parser.add_argument("-o", "--output",
                            type=str,
                            help="Output file path",
                            dest="output_file",
                            required=True)

    return arg_parser.parse_args()


def main():
    args = get_args()

    input_file_handler = FormatFactory.from_path(args.input_file)
    output_file_handler = FormatFactory.from_path(args.output_file)

    logs = input_file_handler.import_logs(args.input_file)
    output_file_handler.export_logs(args.output_file, logs)


if __name__ == "__main__":
    main()
