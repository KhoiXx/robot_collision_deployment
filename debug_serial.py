#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug script to check serial communication with ESP32
Run this ON THE JETSON to diagnose encoder issues
"""

import serial
import struct
import time
import sys

# Serial config (from settings.py)
ROBOT_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

START_BYTE = 0xAB
GET_SPEED = 0x04

def calculate_crc(data):
    """Calculate XOR checksum"""
    crc = 0
    for byte in data:
        crc ^= byte
    return crc

def send_get_speed(port):
    """Send GET_SPEED command"""
    frame_length = 1  # only CRC
    header = struct.pack('BBB', START_BYTE, GET_SPEED, frame_length)
    send_data = bytearray(header)
    crc = calculate_crc(send_data)
    send_data.append(crc)
    port.write(send_data)
    print("Sent GET_SPEED command: " + send_data.hex())

def main():
    print("=" * 60)
    print("ESP32 SERIAL DEBUG TOOL")
    print("=" * 60)

    # Step 1: Check if serial port exists
    print("\n[1] Checking serial port: " + ROBOT_PORT)
    try:
        import os
        if os.path.exists(ROBOT_PORT):
            print("[OK] Serial port " + ROBOT_PORT + " exists")
        else:
            print("[FAIL] Serial port " + ROBOT_PORT + " NOT FOUND!")
            print("\nAvailable serial ports:")
            import glob
            ports = glob.glob('/dev/tty[AU]*')
            if ports:
                for p in ports:
                    print("  - " + p)
            else:
                print("  No USB/ACM devices found!")
            return
    except Exception as e:
        print("Error checking port: " + str(e))
        return

    # Step 2: Try to open serial port
    print("\n[2] Opening serial port at " + str(BAUDRATE) + " baud...")
    try:
        ser = serial.Serial(ROBOT_PORT, BAUDRATE, timeout=1.0)
        print("[OK] Serial port opened successfully")
    except Exception as e:
        print("[FAIL] Failed to open serial port: " + str(e))
        return

    # Step 3: Wait for ESP to boot
    print("\n[3] Waiting 2 seconds for ESP to boot...")
    time.sleep(2)

    # Step 4: Check for incoming data (passive)
    print("\n[4] Checking for incoming data (5 seconds)...")
    print("    (ESP should auto-push speed data at 50Hz)")
    start_time = time.time()
    packet_count = 0

    while time.time() - start_time < 5.0:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            if byte:
                char = chr(byte[0]) if 32 <= byte[0] < 127 else '?'
                print("    Received byte: 0x{:02x} ('{}')".format(byte[0], char))
                packet_count += 1

                # Try to read more if available
                if ser.in_waiting > 0:
                    rest = ser.read(ser.in_waiting)
                    print("    + {} more bytes: {}...".format(len(rest), rest[:20].hex()))
        time.sleep(0.01)

    if packet_count == 0:
        print("    [FAIL] NO DATA RECEIVED!")
        print("    -> ESP is NOT auto-pushing data")
    else:
        print("    [OK] Received {} bytes".format(packet_count))

    # Step 5: Try manual GET_SPEED request
    print("\n[5] Sending manual GET_SPEED command...")
    ser.reset_input_buffer()
    send_get_speed(ser)

    print("    Waiting for response (2 seconds)...")
    time.sleep(0.5)

    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        print("    [OK] Received {} bytes: {}".format(len(data), data.hex()))

        # Try to parse speed packet
        if len(data) >= 11 and data[0:1] == b'B' and data[1] == GET_SPEED:
            try:
                left_vel, right_vel = struct.unpack('<ff', data[2:10])
                print("    -> Left vel: {:.4f} m/s".format(left_vel))
                print("    -> Right vel: {:.4f} m/s".format(right_vel))
            except:
                print("    Failed to parse speed data")
    else:
        print("    [FAIL] NO RESPONSE!")
        print("    -> ESP is not responding to GET_SPEED")

    # Step 6: Summary
    print("\n" + "=" * 60)
    print("DIAGNOSIS SUMMARY:")
    print("=" * 60)

    if packet_count == 0:
        print("[FAIL] ESP is NOT sending data automatically")
        print("  Possible causes:")
        print("  1. ESP firmware not flashed or crashed")
        print("  2. ESP not connected to Jetson")
        print("  3. Wrong serial port (check /dev/ttyUSB0 vs /dev/ttyACM0)")
        print("  4. Baudrate mismatch (check ESP is 115200)")
        print("\nNext steps:")
        print("  1. Check ESP power LED is on")
        print("  2. Re-flash ESP firmware")
        print("  3. Check USB cable connection")
    else:
        print("[OK] ESP is sending data")
        print("  -> Issue may be in robot_control.py parsing logic")

    print("=" * 60)
    ser.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print("\nUnexpected error: " + str(e))
        import traceback
        traceback.print_exc()
