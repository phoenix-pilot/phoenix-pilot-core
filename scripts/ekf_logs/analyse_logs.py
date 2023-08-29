import argparse

from common.formats import FormatFactory
from logs_evaluations import EvaluationsFactory, StudyContext


def get_args():
    arg_parser = argparse.ArgumentParser(description="Simple ekf logs analysis")
    arg_parser.add_argument("log_file", type=str, help="File with logs from ekf")

    return arg_parser.parse_args()


def main():
    args = get_args()

    handler = FormatFactory.from_path(args.log_file)
    logs = handler.import_logs(args.log_file)

    ctx = StudyContext(logs)

    for analysis in EvaluationsFactory.get_evaluations():
        analysis.run(ctx)


if __name__ == "__main__":
    main()
