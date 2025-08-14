# CANopen_socketCAN_Test

This repository is for testing and setting up an Axiomatic CANopen device using Python tools and SocketCAN on Linux.

## Purpose
- Validate and configure Axiomatic CANopen devices.
- Log, analyze, and automate CANopen communication for development and troubleshooting.

## Tools Used
- **Python**: Main scripting language for automation and analysis.
- **SocketCAN**: Linux CAN interface for sending/receiving CAN frames.
- **can-utils**: Python utilities for CAN communication and device control (see `utils/can_utils/`).
- **Database Logger**: Scripts for logging CAN data to databases (see `utils/database_logger/`).
- **DBC/EDS Files**: Device configuration and message decoding (see `config/`).

## Getting Started
1. Ensure you have Python 3.12+ and SocketCAN enabled on your Linux system.
2. Install dependencies using Pipenv:
	```bash
	pipenv install
	```
3. Review example scripts in `utils/` and configuration files in `config/`.

## Hardware Setup
- Setup Axiomatic AX020421 (12 inputs / 12 outputs) on the bench
- Default baud rate: 125000 (125 kbit/s)
- Default node ID: 127

Bring up the SocketCAN interface at 125 kbit/s:

```bash
sudo ip link set can0 txqueuelen 1000 up type can bitrate 125000
```

## How to Run
Run the main script with a YAML configuration file to apply CANopen SDO configuration and then start monitoring the device:

```bash
python axiomatic_dev.py -c config/test_axiomatic.yaml
```

- **What this does**: Configures the Axiomatic device using CANopen SDO communication based on the entries in the provided YAML file, then starts the device runtime.
- **Minimal config needed**: The YAML only needs to include the CANopen Object Dictionary addresses (indices/sub-indices from the `.eds` file) that you want to change. Any addresses not listed are left unchanged on the device.
- **When configuration happens**: Configuration is applied during device creation/initialization inside `axiomatic_dev.py`. After that, the script runs its own state machine to operate and monitor the Axiomatic CANopen device.

## Folder Structure
- `axiomatic_dev.py`: Main device setup and test script.
- `utils/`: Python utilities for CAN communication, state diagrams, and logging.
- `config/`: DBC/EDS files and YAML configs for device setup.
- `logs/`: Example CAN log files.

