# TF-Luna LiDAR Python Driver

The [TF-Luna](https://www.dfrobot.com/product-1995.html) is a compact, single-point ranging LiDAR sensor that can be read over I2C or Serial interfaces.

This repository provides a modern, typed, and robust Python library for using the TF-Luna sensor with its serial interface on platforms like the Raspberry Pi.

This library is a modernized fork of [tfluna-python](https://github.com/makerportal/tfluna-python), refactored to provide a cleaner API, strict type safety, and simplified dependency management.

Modernized by **Henry Martin** for the **Aerospace Jam**.

## Usage Example

The API has been simplified to use `open()` and `close()` methods directly. It is recommended to use a `try...finally` block to ensure that the serial connection is always closed, even if an error occurs.

```python
import tfluna
import time

# Create a TfLuna instance
# Note: The serial port is not opened on instantiation.
sensor = tfluna.TfLuna(serial_name="/dev/serial0", baud_speed=115200)

try:
    # Open the serial port
    sensor.open()

    # Get the firmware version
    version = sensor.get_version()
    if version:
        print(f"TF-Luna firmware version: {version}")

    # Set the sample rate to 10Hz
    sensor.set_samp_rate(10)

    # Optional: Change the baud rate (and reconnect)
    # if sensor.set_baudrate(57600):
    #     print("Baud rate changed successfully.")
    # else:
    #     print("Failed to change baud rate.")

    # Read data from the sensor in a loop
    for _ in range(10):
        data = sensor.read_tfluna_data()
        if data:
            distance, strength, temperature = data
            print(f"Distance: {distance:.2f}m, Strength: {strength}, Temperature: {temperature:.2f}°C")
        time.sleep(0.1)

finally:
    # Always close the serial port
    sensor.close()
    print("Serial port closed.")

```

## Wiring - TF-Luna + Raspberry Pi

The TF-Luna can be easily wired to a Raspberry Pi's GPIO header using the mini UART port (`/dev/serial0`).

![TF-Luna RPi Wiring](https://static1.squarespace.com/static/59b037304c0dbfb092fbe894/t/6009f277b8566661c36dfa67/1611264637375/TF_luna_RPi_wiring.png?format=1500w)

Since the GPIO on the Pi 4 and Pi Zero (& 2|W|2W) are the same, this wiring will also work with your stock kit. Give the [docs](https://docs.aerospacejam.org/) a read for more up-to-date information.