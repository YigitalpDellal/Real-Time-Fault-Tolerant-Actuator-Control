"""
Project:
Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed

Milestone:
Dual-Axis PAN/TILT Command Verification

Purpose:
- Verify Raspberry Pi -> TM4C123 actuator commands
- Test PAN and TILT command parsing
- Verify acknowledgement responses
- Verify CENTER command for both axes

UART configuration:
Port      : /dev/serial0
Baud rate : 115200
Format    : 8N1
"""

import serial
import time


UART_PORT = "/dev/serial0"
UART_BAUD_RATE = 115200
UART_TIMEOUT_SECONDS = 1


def send_command(serial_port, command):
    """
    Send one command to the TM4C123 and return its response.
    """

    # Remove any stale bytes before starting a new transaction.
    serial_port.reset_input_buffer()

    print(f"[PI -> TM4C] {command}")

    serial_port.write(
        f"{command}\r\n".encode()
    )

    response = (
        serial_port.readline()
        .decode(errors="replace")
        .strip()
    )

    print(f"[TM4C -> PI] {response}")

    return response


def verify_command(serial_port, command, expected_response):
    """
    Send a command and verify the expected TM4C acknowledgement.
    """

    response = send_command(
        serial_port,
        command
    )

    if response != expected_response:
        print(
            f"[FAIL] {command}: "
            f"expected {expected_response}, received {response}"
        )
        return False

    return True


def main():
    """
    Verify the complete dual-axis actuator command protocol.

    Sequence:
    1. Verify UART communication
    2. Test PAN movement commands
    3. Test TILT movement commands
    4. Return both axes to center
    """

    serial_port = serial.Serial(
        port=UART_PORT,
        baudrate=UART_BAUD_RATE,
        timeout=UART_TIMEOUT_SECONDS
    )

    # Allow the UART interface to settle after opening.
    time.sleep(1)

    print("=== DUAL-AXIS ACTUATOR TEST ===")

    tests = [
        ("PING", "ACK"),

        ("PAN 70", "PAN_OK"),
        ("PAN 110", "PAN_OK"),

        ("TILT 70", "TILT_OK"),
        ("TILT 110", "TILT_OK"),

        ("CENTER", "CENTER_OK")
    ]

    for command, expected_response in tests:
        if not verify_command(
            serial_port,
            command,
            expected_response
        ):
            serial_port.close()
            return

        # Small delay keeps the test sequence readable and later
        # gives the physical actuators time to reach each position.
        time.sleep(1)

    print("[PASS] Dual-axis actuator protocol verified.")

    serial_port.close()


if __name__ == "__main__":
    main()
