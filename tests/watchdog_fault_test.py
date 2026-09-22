"""
Project:
Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed

Test:
TM4C123 Local Communication-Watchdog Fault Injection

Purpose:
1. Verify UART communication.
2. Move PAN/TILT away from the safe center position.
3. Stop all Raspberry Pi communication.
4. Allow the TM4C123 local watchdog to expire.
5. Verify that the MCU returns both actuators to center
   without receiving a CENTER command from the Raspberry Pi.

Important:
The Raspberry Pi intentionally sends no messages during the
fault-injection interval.
"""

import serial
import time


UART_PORT = "/dev/serial0"
UART_BAUD_RATE = 115200
UART_TIMEOUT_SECONDS = 1


def send_command(serial_port, command):
    """Send one command and read one TM4C response."""

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


def main():

    serial_port = serial.Serial(
        port=UART_PORT,
        baudrate=UART_BAUD_RATE,
        timeout=UART_TIMEOUT_SECONDS
    )

    time.sleep(0.5)

    print("=== LOCAL WATCHDOG FAULT-INJECTION TEST ===")

    # Verify that communication is healthy before injecting the fault.
    if send_command(serial_port, "PING") != "ACK":
        print("[FAIL] UART communication unavailable.")
        serial_port.close()
        return

    # Move the mechanism away from its safe center position.
    if send_command(serial_port, "PAN 70") != "PAN_OK":
        print("[FAIL] PAN command rejected.")
        serial_port.close()
        return

    if send_command(serial_port, "TILT 110") != "TILT_OK":
        print("[FAIL] TILT command rejected.")
        serial_port.close()
        return

    print()
    print("[FAULT] Raspberry Pi communication intentionally stopped.")
    print("[FAULT] No PING, PAN, TILT or CENTER command will be sent.")
    print("[WAIT] TM4C watchdog timeout = 300 ms")

    # Intentionally remain completely silent.
    # The TM4C must independently detect communication loss
    # and command both actuators to the local safe center position.
    time.sleep(2.0)

    print()
    print("[CHECK] Watchdog interval complete.")
    print("[EXPECTED] PAN = 90 deg, TILT = 90 deg")
    print("[EXPECTED] Safe-center action performed locally by TM4C123.")

    # Verify that the communication link can recover afterward.
    # This is NOT a CENTER command.
    response = send_command(serial_port, "PING")

    if response == "ACK":
        print("[PASS] UART recovered after watchdog event.")
    else:
        print("[FAIL] UART recovery check failed.")

    serial_port.close()


if __name__ == "__main__":
    main()
