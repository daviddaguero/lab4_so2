# Real-Time Application with FreeRTOS on Stellaris LM3S811

## Introduction

Real-time systems are an essential part of engineering applications that require strict timing constraints. These systems, controlled by computing platforms, utilize Real-Time Operating Systems (RTOS) to ensure reliable and predictable performance. One of the key characteristics of RTOS is its ability to leverage a preemptive kernel and a highly configurable scheduler. Such systems are widely used in fields such as avionics, radar systems, and satellites.

## Objective

The objective of this project is to design, create, test, and validate a real-time application using an RTOS. By completing this project, the goal is to develop the skills and expertise required to work on time-critical engineering applications.

## Project Overview

This project involves developing a real-time application based on FreeRTOS and emulated using QEMU on the Stellaris LM3S811 platform. The application will feature the following functionalities:

1. **Temperature Sensor Task**: Simulates a temperature sensor generating random values at a frequency of 10 Hz.
2. **Low-Pass Filter Task**: Processes the sensor's values using a low-pass filter, where each resulting value is the average of the last N measurements.
3. **Display Task**: Graphs temperature values over time on the display.
4. **UART Command Interface**: Enables changing the value of N in the filter through UART commands.
5. **Task Stack Analysis**: Includes calculations for the required stack size for each task, leveraging tools like `uxTaskGetStackHighWaterMark` or `vApplicationStackOverflowHook`.
6. **System Monitoring Task**: Implements a "top-like" task that periodically displays task statistics (e.g., CPU and memory usage).
