import argparse

from common.formats import FormatFactory
from logs_analysis import AnalysesFactory, AnalysisContext


def get_args():
    arg_parser = argparse.ArgumentParser(description="Simple ekf logs analysis")
    arg_parser.add_argument("log_file", type=str, help="File with logs from ekf")

    return arg_parser.parse_args()


def main():
    args = get_args()

    file_handler = FormatFactory.from_path(args.log_file)
    logs = file_handler.import_logs(args.log_file)

    analyses = AnalysesFactory.get_analyses()

    analyses_context = AnalysisContext(logs)

    for analysis in analyses:
        analysis.run(analyses_context)


if __name__ == "__main__":
    main()
