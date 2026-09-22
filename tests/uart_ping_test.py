"""
Project:
Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed

Current milestone:
- Verify bidirectional UART communication
- Send a PING command from Raspberry Pi
- Verify that TM4C123 responds with ACK

Hardware connection:
Raspberry Pi GPIO14 / TX -> TM4C123 PB0 / U1RX
Raspberry Pi GPIO15 / RX <- TM4C123 PB1 / U1TX
Raspberry Pi GND         -> TM4C123 GND

UART configuration:
Port      : /dev/serial0
Baud rate : 115200
Format    : 8N1
"""

import serial
import time


# Raspberry Pi serial-device alias.
# Using /dev/serial0 avoids depending directly on ttyAMA0/ttyS0 naming.
UART_PORT = "/dev/serial0"

# Must match the UART configuration used by the TM4C123 firmware.
UART_BAUD_RATE = 115200

# Maximum time to wait for a TM4C response.
UART_TIMEOUT_SECONDS = 1


def main():
    """
    Open the Raspberry Pi UART interface, transmit a PING command,
    and verify the ACK response returned by the TM4C123.
    """

    serial_port = serial.Serial(
        port=UART_PORT,
        baudrate=UART_BAUD_RATE,
        timeout=UART_TIMEOUT_SECONDS
    )

    # Allow the serial interface a short time to settle after opening.
    time.sleep(1)

    # Discard data that may already be waiting in the receive buffer,
    # such as the TM4C startup message.
    serial_port.reset_input_buffer()

    print("[PI] Sending: PING")

    # CRLF terminates the command because the TM4C parser accepts
    # carriage return or line feed as the end of a command.
    serial_port.write(b"PING\r\n")

    # Read one response line from the TM4C and convert it to text.
    response = (
        serial_port.readline()
        .decode(errors="replace")
        .strip()
    )

    print(f"[TM4C] Response: {response}")

    # Verify that communication worked in both directions.
    if response == "ACK":
        print("[PASS] UART communication verified.")
    else:
        print("[FAIL] Expected ACK.")

    # Release the UART device cleanly.
    serial_port.close()


if __name__ == "__main__":
    main()
