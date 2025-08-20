# BLDC Teensy Control

## Overview

This project provides embedded code for **Teensy microcontrollers** to control BLDC motors using **Trapezoidal** and **FOC (Field-Oriented Control)** algorithms. It includes **PID velocity control** and is structured for easy integration with motor feedback systems.

## Features

* **Trapezoidal Control**: Simple, efficient control for basic BLDC operation.
* **FOC Control**: Smooth and precise field-oriented control.
* **PID Velocity Control**: Adjustable PID parameters for speed regulation.
* **Logging Support**: Motor and control data can be logged for analysis.

## Repository Structure

```
BldcFOC_teensy/
├── Bldc/          # Base motor control structures and helpers
├── Foc/           # FOC algorithm implementation
├── Trap/          # Trapezoidal control implementation
├── logs/          # Logging utilities
```

## Requirements

* **Teensy microcontroller** (compatible with Teensy 3.x or 4.x series)
* **Arduino IDE** or PlatformIO for building and uploading
* BLDC motor with Hall sensors or encoder for feedback

## Installation & Usage

1. Clone the repository:

   ```bash
   git clone https://github.com/Ineso1/bldcFOC_teensy.git
   cd bldcFOC_teensy
   ```

2. Open the desired folder (`Foc` or `Trap`) in **Arduino IDE** or **PlatformIO**.

3. Configure your **PID parameters** and motor setup in the main file.

4. Upload to your Teensy board.

5. Monitor the serial output for velocity and status logs.

## Demo

Watch the following video to see the BLDC motor control in action:

[![BLDC Motor Control Demo](https://img.youtube.com/vi/vhkOnLaiGKc/0.jpg)](https://www.youtube.com/watch?v=vhkOnLaiGKc)

## Notes

* **FOC** provides smoother operation and higher efficiency at low and high speeds.
* **Trapezoidal** control is faster to deploy for simple applications.
* Logging helps tune PID gains and monitor motor behavior in real-time.

## References

* Field-Oriented Control (FOC) principles
* Teensy microcontroller documentation
* BLDC motor datasheets
