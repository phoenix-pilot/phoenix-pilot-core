# EKF Logs Scripts

This directory contains scripts that assist with common operations related to EKF logs.

Available scripts:
 - `analyse_logs.py`: A simple tool for obtaining an overview of the contents of a specific log file.
 - `converter.py`: Enables the conversion of log file formats.
 - `generate_test_data.py`: Generates predefined scenarios for EKF tests.

## Logs analysis

### Usage

```bash
analyse_logs.py <log_file>
```

Script takes path to file, which contains EKF logs as positional CLI argument. Format of this file can either binary
or CSV.

### Performed analyses

Currently, analyses are performed in such order:
1. Information, about missing logs - it is possible, that not all logs were saved during EKF execution. In such case,
information about a number of missing entries is provided as well as precise graph, which shows where logs were lost.
If provided file has no gaps simple message is printed to stdout.

2. Number of logs, which are saved in file under examination. Moreover, pie chart containing the percentage of specific
log types.

3. EKF iterations durations. If file contains data about used timestamps, the program provides information about
execution time of every EKF iteration. Information is shown in form of a box plot with emphasized minimum, maximum
and mean value. Every loop time is also presented on additional plot.

4. GPS and EKF State Comparison: If the file contains data from GPS as well as EKF state, this evaluation compares
position and velocities. Additionally raw and filtered with IIR filter accelerations are showed in earth and body frame
of reference. In these figures clicking on legend lines toggles line visibility.

## Logs conversion

### Usage

```bash
converter.py -i <input_file> -o <output_file>
```

Script takes two required options.
 - `-i INPUT_FILE` or `--input INPUT_FILE` - specifies path to source file with saved logs
 - `-o OUTPUT_FILE` or `--output OUTPUT_FILE` - specifies path to result file

 Script recognizes type of file based on its extension. For example, if we want to convert logs from binary format to
 CSV one:
```bash
converter.py -i input.bin -o output.csv
```

### Supported formats

Currently, script supports:
 - binary format (`.bin`)
 - CSV format (`.csv`)

## Generating EKF test scenario

### Usage

```bash
generate_test_data.py
```

Script takes no additional arguments.

### Generated scenario

At the moment script generates `const_data_ekf_scenario.bin`, which simulates standing perfectly still at zero
degrees latitude and zero degrees longitude.
