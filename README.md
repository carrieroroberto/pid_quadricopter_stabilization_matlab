# Quadcopter PID Control Simulator in MATLAB®

This repository contains the source code for a simulator developed in **MATLAB®** to study and stabilize a quadcopter drone using PID controllers. This was developed as a term project for the Avionics Systems Programming course at the Polytechnic University of Bari.

The simulator models the complete 6-Degrees-of-Freedom (6-DoF) dynamics of a quadcopter and implements a dual-loop control system (for position and attitude) to hold the drone at a target position, while also compensating for external disturbances like simulated wind gusts.

## 🚀 Key Features

- **Realistic Dynamic Model**: The drone's dynamics are based on the Newton-Euler equations for a 6-DoF rigid body (3 translational, 3 rotational).
- **Dual-Loop PID Control**:
    - **Outer Loop (Position)**: Controls the altitude (Z coordinate) and horizontal position (X, Y) by generating desired roll and pitch angles.
    - **Inner Loop (Attitude)**: Stabilizes the drone's attitude (roll $\phi$, pitch $\theta$, and yaw $\psi$ angles) by directly manipulating the motor torques.
- **Interactive GUI**: A graphical user interface with sliders allows for real-time tuning of the PID gains ($K_p, K_i, K_d$) for both controllers, providing immediate feedback on system stability.
- **Disturbance Simulation**: The simulator can generate random wind gusts with variable speed, direction, and duration to test the robustness of the control system.
- **Real-Time Visualization**:
    - A 3D animation of the drone, its target position, and a vector indicating the wind's direction.
    - Live plots showing the position/attitude errors and the contribution of each PID term (Proportional, Integral, Derivative).
- **Advanced Control Techniques**: Implementation of an **anti-windup** mechanism to limit integral error accumulation and a **low-pass filter** on the derivative term to reduce noise amplification.

## 🛠️ Prerequisites

- **MATLAB®** (R2021b or later recommended).

## ⚙️ How to Run

1.  Clone this repository to your local machine.
2.  Open MATLAB®.
3.  Navigate to the project's directory.
4.  Run the main script:
    ```matlab
    quadricottero_pid_main
    ```
5.  The three figure windows (3D simulation, PID plots, error plots) will open, and the simulation will start.
6.  Use the sliders in the 3D simulation window to tune the PID gains and observe the drone's behavior.

## 📂 Code Structure

- `quadricottero_pid_main.m`: The main script that initializes the drone and simulation parameters, handles the graphical setup, and contains the main simulation loop.
- `quadricottero_pid_ui_callbacks.m`: An external function that handles events triggered by user interactions with the GUI (e.g., moving the sliders).
