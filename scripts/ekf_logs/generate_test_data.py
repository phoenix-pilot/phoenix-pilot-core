import argparse

from ekf_tests.const_data_test import generate_const_data_scenario


def get_args():
    argparse.ArgumentParser(description="Test scenario generator for EKF module tests.").parse_args()


def main():
    get_args()

    generate_const_data_scenario()


if __name__ == "__main__":
    main()
