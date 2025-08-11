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

## Folder Structure
- `axiomatic_dev.py`: Main device setup and test script.
- `utils/`: Python utilities for CAN communication, state diagrams, and logging.
- `config/`: DBC/EDS files and YAML configs for device setup.
- `logs/`: Example CAN log files.

