"""
Copyright (c) 2021 Maker Portal LLC
Author: Joshua Hrisko
Code refactoring (to classes): Clément Nussbaumer, April 2021
Modernization, typing, and removal of context manager: Henry Martin, 2025

Reference manual for the sensor: https://github.com/May-DFRobot/DFRobot/blob/master/TF-Luna%20LiDAR%EF%BC%888m%EF%BC%89Product%20Manual.pdf

TF-Luna Mini LiDAR wired to a Raspberry Pi via UART
--- Configuring the TF-Luna's baudrate, sample rate,
--- and printing out the device version info
"""
import serial
import time
from typing import Dict, List, Optional, Tuple

class TFLuna:
    """
    A class to interact with the TF-Luna LiDAR sensor over a serial connection.
    """

    BAUD_CONFIG: Dict[int, List[int]] = {
        9600: [0x80, 0x25, 0x00],
        19200: [0x00, 0x4b, 0x00],
        38400: [0x00, 0x96, 0x00],
        57600: [0x00, 0xe1, 0x00],
        115200: [0x00, 0xc2, 0x01],
        230400: [0x00, 0x84, 0x03],
        460800: [0x00, 0x08, 0x07],
        921600: [0x00, 0x10, 0x0e],
    }

    def __init__(self, serial_name: str = "/dev/serial0", baud_speed: int = 115200) -> None:
        if baud_speed not in self.BAUD_CONFIG:
            raise ValueError(
                f"Invalid baud_speed setting used: {baud_speed}\n"
                f"Available baud speeds: {', '.join(map(str, self.BAUD_CONFIG.keys()))}"
            )
        self.serial_name: str = serial_name
        self.baud_speed: int = baud_speed
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        """Opens the serial port."""
        if self.ser is not None and self.ser.is_open:
            return
        self.ser = serial.Serial(self.serial_name, self.baud_speed, timeout=1)

    def close(self) -> None:
        """Closes the serial port."""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _assert_is_open(self) -> serial.Serial:
        """Checks if the serial port is open and returns the serial object."""
        if self.ser is None or not self.ser.is_open:
            raise serial.SerialException("Serial port is not open. Call open() first.")
        return self.ser

    def read(self, timeout: float = 5.0) -> Optional[Tuple[float, int, float]]:
        """
        Reads distance, strength, and temperature data from the TF-Luna sensor.

        Args:
            timeout: The maximum time to wait for data in seconds.

        Returns:
            A tuple containing distance (m), strength, and temperature (°C),
            or None if a complete frame is not received within the timeout.
        """
        ser = self._assert_is_open()
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Look for the start of a frame
            if ser.read(1) == b'\x59':
                if ser.read(1) == b'\x59':
                    frame = ser.read(7)
                    if len(frame) == 7:
                        distance = frame[0] + frame[1] * 256
                        strength = frame[2] + frame[3] * 256
                        temperature: int|float = frame[4] + frame[5] * 256
                        temperature = (temperature / 8.0) - 256.0
                        return distance / 100.0, strength, temperature
        return None

    def set_samp_rate(self, samp_rate: int = 100) -> None:
        """
        Sets the sample rate of the TF-Luna sensor.

        Args:
            samp_rate: The desired sample rate in Hz.
        """
        ser = self._assert_is_open()
        samp_rate_packet = [0x5a, 0x06, 0x03, samp_rate, 0x00, 0x00]
        ser.write(bytearray(samp_rate_packet))
        time.sleep(0.1)

    def get_version(self, timeout: float = 5.0) -> Optional[str]:
        """
        Gets the version information from the TF-Luna sensor.

        Args:
            timeout: The maximum time to wait for a response in seconds.

        Returns:
            The version string, or None if a valid response is not received.
        """
        ser = self._assert_is_open()
        info_packet = [0x5a, 0x04, 0x14, 0x00]
        ser.write(bytearray(info_packet))

        start_time = time.time()
        while time.time() - start_time < timeout:
            if ser.in_waiting >= 30:
                bytes_data = ser.read(30)
                if bytes_data[0] == 0x5a:
                    try:
                        version = bytes_data[3:-1].decode('utf-8')
                        return version
                    except UnicodeDecodeError:
                        continue
        return None

    def set_baudrate(self, baud_rate: int = 115200, timeout: float = 5.0) -> bool:
        """
        Sets the baud rate of the TF-Luna sensor.

        Args:
            baud_rate: The desired baud rate.
            timeout: The maximum time to wait for a confirmation in seconds.

        Returns:
            True if the baud rate was successfully set and confirmed, False otherwise.
        """
        ser = self._assert_is_open()
        if baud_rate not in self.BAUD_CONFIG:
            raise ValueError(
                f"Invalid baud_rate setting used: {baud_rate}\n"
                f"Available baud speeds: {', '.join(map(str, self.BAUD_CONFIG.keys()))}"
            )

        config_bytes = self.BAUD_CONFIG[baud_rate]
        info_packet = [0x5a, 0x08, 0x06, config_bytes[0], config_bytes[1], config_bytes[2], 0x00, 0x00]
        ser.write(bytearray(info_packet))
        time.sleep(0.1)

        # Reconfigure the serial port to the new baud rate
        ser.baudrate = baud_rate
        self.baud_speed = baud_rate

        start_time = time.time()
        while time.time() - start_time < timeout:
            if ser.in_waiting >= 8:
                response = ser.read(8)
                if response[0] == 0x5a and response[3:6] == bytearray(config_bytes):
                    return True
        return False