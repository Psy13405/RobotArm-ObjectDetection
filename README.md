# Robotic Arm for Object Detection on an Embedded System

The goal of this project was to design and develop a robotic arm that could
detect, track, and react accordingly to moving objects - specifically small sports balls
such as a ping pong ball - using computer vision. The arm is designed to respond in real
time by moving each joint in response to the ball's movement to intercept it.

## Overview

This project integrates an embedded system, a computer vision pipeline, some
mechanical design, and control systems.

## Repository Contents

- `Code/`- Contains src/ and test_scripts/ along with main.py and the script implemented on the arm: object_throw.py
- `src/` — Modules for initialization of servo motor control and movement boundaries 
- `test_scripts` - Scripts to iteratively test the arm
- `CAD_Files/` — 3D printable STL files

## Hardware and Software

Hardware: Raspberry Pi 4B, IMX708 Camera Module, 3 HiWonder HX-35H servo motors, and Hiwonder BusLinker V3.0 Servo Debugging Board
Software: Raspberry Pi OS, OpenCV Python Library

## Building and Running

Acquire hardware and properly wire units. Setup Raspberry Pi OS and install standard Python tools along with the OpenCV library. All files in src/ along with main.py are needed before writing more movement test scripts.

## Results

The arm responds to a ping pong ball when it enters the camera's field of view. Integrating a hand that would grab the ball was the biggest challenge and a good starting point for future iterations.

## Demo Video Link: https://www.youtube.com/shorts/wb952vX-y7Q
