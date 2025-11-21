# **IMU Coordinated Bot**
![Circuit Diagram](Circuit_diagram.png)
# 🤖 Zero-Speed Heading Lock Mode (Yaw Stabilization)

## 🎯 Project Overview

This project implements a differential drive robot capable of **active, stationary stabilization** using a Proportional-Integral-Derivative (PID) control loop. The primary goal is the **Zero-Speed Heading Lock**: the robot measures its initial orientation (Yaw angle) and actively applies corrective torque to resist any external rotation, holding its position without forward movement.

This system demonstrates dynamic stability, precise error correction, and robust sensor fusion techniques essential for advanced robotics.

## 📚 Dependencies

This project requires the following libraries installed in the Arduino IDE:
* **Adafruit MPU6050 Library:** For sensor communication and reading.
* **MadgwickAHRS Library:** For sensor fusion (Gyro/Accel) to calculate drift-compensated Yaw angle.
* **Wire Library:** For I2C communication (standard Arduino library).

## ✨ Key Features

  * **Dynamic Stabilization:** Actively applies motor torque to resist external disturbances and hold a static heading.
  * **5-Second Calibration:** Automatically determines the initial stable Yaw angle to serve as the system's permanent setpoint.
  * **Madgwick Filter:** Utilizes sensor fusion (Gyroscope and Accelerometer) to provide drift-compensated and highly accurate heading data.
  * **PID Control:** Uses Yaw error as input to calculate precise PWM outputs for differential steering.
  * **Deadband Implementation:** Motors shut off when the error is within a tight margin (e.g., ±1.0 degrees) to conserve power and prevent jitter.

## 🛠️ Hardware Components

| Component | Description |
| :--- | :--- |
| **Microcontroller** | Arduino Uno |
| **Inertial Sensor** | MPU-6050 (IMU) |
| **Motor Driver** | L298N |
| **Motors** | 2x N20 DC Motors (100 RPM) |
| **Power Supply** | External battery source (7V - 12V recommended for L298N) |

## ⚙️ Control System Architecture

The system operates on a continuous feedback loop:

1.  **Sensing:** The MPU-6050 provides angular velocity ($\text{Gyro } Z$) and acceleration data.
2.  **Filtering:** The MadgwickAHRS filter fuses this data to produce a stable, drift-free **Yaw Angle** (Current Heading).
3.  **Error Calculation:** $\text{Error} = \text{Target Yaw} - \text{Current Yaw}$.
4.  **PID Calculation:** The $\text{PID Output}$ (Correction Torque) is calculated based on $K_p, K_i, K_d$ gains.

$$\text{PID}_{\text{Output}} = (K_p \cdot \text{Error}) + (K_i \cdot \sum \text{Error} \cdot \Delta t) + (K_d \cdot \frac{d\text{Error}}{dt})$$

5.  **Actuation (Heading Lock):**
      * If $\text{Error} > \text{Deadband}$ (Turn Right needed), the **Left Motor** runs forward.
      * If $\text{Error} < -\text{Deadband}$ (Turn Left needed), the **Right Motor** runs forward.
      * If $\text{Error}$ is within the Deadband, both motors are stopped.

## 🔌 Wiring Diagram Summary

| Component | Arduino Pin | Notes |
| :--- | :--- | :--- |
| **MPU-6050 SDA** | A4 | I2C Data |
| **MPU-6050 SCL** | A5 | I2C Clock |
| **L298N ENA (Left PWM)** | D10 | PWM Output |
| **L298N IN1/IN2** | D8 / D9 | Direction Control |
| **L298N ENB (Right PWM)** | D11 | PWM Output |
| **L298N IN3/IN4** | D7 / D6 | Direction Control |
| **Grounds** | GND | All grounds must be common. |

## 🧪 Tuning and Calibration (Crucial Step)

The stability of the Heading Lock mode relies entirely on tuning the PID constants ($K_p, K_i, K_d$).

### Calibration Procedure

The code automatically performs a 5-second calibration at startup:

1.  Place the bot on a level surface, facing the desired "lock" direction.
2.  Power the Arduino (and the motors via L298N external power).
3.  The bot remains stationary for 5 seconds while averaging the initial Yaw angle. This average becomes the permanent `TARGET_YAW_CALIBRATED`.

### PID Tuning Steps

**0. Polarity Check (Crucial)**
Before tuning, verify that the correction polarity is correct (Negative Feedback).
* Set $K_p=0.5, K_i=0, K_d=0$.
* Manually rotate the bot $5^{\circ}$ to the right.
* The **Left Motor** must activate to push the bot back left.
* If the bot spins *further* to the right (positive feedback), you must reverse the motor connection or the polarity of the correction in the code (e.g., swapping which motor gets $\text{PID}_{\text{Output}}$ or which direction it runs).

Start with the initial values from the code (`Kp = 5.0, Ki = 0.01, Kd = 0.5`) and follow this sequence:

1.  **Tune $\mathbf{K_p}$ (Proportional):**
      * Set $K_i=0$ and $K_d=0$.
      * Increase $K_p$ slowly until the robot starts to **oscillate quickly** around the setpoint.
      * Set the final $K_p$ to $\mathbf{50\%}$ of the value that caused rapid oscillation.
2.  **Tune $\mathbf{K_d}$ (Derivative):**
      * Increase $K_d$ to **dampen** the oscillations caused by $K_p$. The correction should become firm and smooth.
3.  **Tune $\mathbf{K_i}$ (Integral):**
      * This is often less critical for stationary stability. Increase $K_i$ very slowly (e.g., $0.001$ increments) to eliminate any persistent, small offset after a disturbance.
