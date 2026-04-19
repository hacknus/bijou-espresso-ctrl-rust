#!/usr/bin/env python3
try:
    import serial
    import time
    import glob
    import sys
    import re
except ImportError:
    print("This script requires pyserial, continuing directly with dfu ...")


def find_device():
    """Find USB modem device on macOS"""
    devices = glob.glob('/dev/tty.usbmodem*')
    if devices:
        return devices[0]
    return None


# python
def wait_for_ack(ser, timeout=2.0, ack_pattern=r'\[ACK\] rebooting', poll_interval=0.05):
    import re, time
    ser.timeout = poll_interval
    deadline = time.time() + timeout
    buffer = ""

    try:
        while time.time() < deadline:
            try:
                chunk = ser.readline()
            except Exception as e:
                msg = str(e)
                # Treat "Device not configured" (errno 6) as a likely reboot / success
                if "Device not configured" in msg or getattr(e, "errno", None) == 6:
                    print("Device disconnected (errno 6) — likely rebooted into bootloader. Treating as success.")
                    return True
                print(f"Serial read error: {e}")
                return False

            if not chunk:
                # no data this cycle
                time.sleep(0.01)
                continue

            try:
                s = chunk.decode('utf-8', errors='ignore')
            except Exception:
                s = str(chunk)

            buffer += s

            if re.search(ack_pattern, buffer):
                print("ACK received - bootloader command acknowledged")
                return True

        print(f"No ACK received within {timeout} seconds. Received buffer:\n{buffer.strip()}")
        return False

    except KeyboardInterrupt:
        print("Interrupted by user")
        return False


def enter_bootloader():
    """Send bootloader command via serial"""
    device = find_device()

    if not device:
        print("No serial device found, continuing anyways")
        return

    print(f"Found device: {device}")

    try:
        with serial.Serial(device, 115200) as ser:
            print("Serial port opened at 115200 baud")

            # Clear buffers
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # Send command
            command = b"[CMD] enter bootloader\r\n"
            print("Sending: [CMD] enter bootloader")
            ser.write(command)
            ser.flush()

            # Usage (replace the original waiting block with this)
            wait_for_ack(ser, timeout=5.0)

    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    enter_bootloader()
