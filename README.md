# STM32 Platform Angle Control System

This project implements a closed-loop angle control system for a DC motor driven platform. The user enters a target angle from a 4x4 keypad, the current platform angle is measured with an MPU6050 IMU, and an STM32F4 microcontroller drives the motor with PWM according to either an on-off controller or a PID controller.

The system was designed for the project requirement of controlling the platform angle in the **-60° to +60°** range.

---

## Features

- Closed-loop angle control using IMU feedback
- Two control modes:
  - On-off control
  - PID control with filtered derivative term and PID deadband near the target
- Target angle input from a 4x4 keypad
- Positive and negative angle command support
- MAX7219 based 7-segment display interface
- MPU6050 accelerometer and gyroscope based angle estimation
- Complementary filter for stable angle feedback
- PWM motor driving with direction control
- Active motor braking in on-off mode for more consistent holding behavior
- Separate supply for control electronics and motor power
- FreeRTOS based task separation for UI and control loop
- MATLAB/Simulink closed-loop model and disturbance response simulation

---

## System Overview

The platform is driven by a DC motor. The user enters the desired reference angle from the keypad. The STM32 reads the MPU6050 sensor, estimates the current platform angle, calculates the control error, and generates PWM signals for the motor driver.

```text
Target angle from keypad
        |
        v
STM32F4 controller
        |
        |-- reads MPU6050 over I2C
        |-- estimates angle using complementary filter
        |-- calculates control error
        |-- applies on-off or PID control
        v
PWM motor driver
        |
        v
DC motor + platform
        |
        v
Measured angle feedback
```

---

## Hardware Components

| Component | Purpose |
|---|---|
| STM32F4 microcontroller | Main control unit |
| DC motor | Moves the platform/rod |
| Motor driver | Drives the DC motor in both directions |
| MPU6050 | Measures accelerometer and gyroscope data |
| 4x4 keypad | Target angle and mode selection input |
| MAX7219 + 7-segment display | Shows target and measured angle values |
| 3 V / 3.3 V control supply | Powers the control electronics |
| 6 V motor supply | Powers the DC motor |

The motor and control electronics are powered separately to reduce the effect of motor current spikes and electrical noise on the microcontroller and sensor readings. The grounds must be connected together so that the PWM control signals share the same reference.

---

## Embedded Software Architecture

The firmware is written in C for STM32F4 and uses direct register-level peripheral configuration. FreeRTOS is used to separate the user interface from the real-time control loop.

### Main Tasks

| Task | Period | Responsibility |
|---|---:|---|
| UI task | 20 ms | Scans the keypad and updates target/mode variables |
| Decision task | 10 ms | Reads IMU, filters angle, updates display, and drives the motor |

### Main Source Files

| File | Description |
|---|---|
| `main.c` | Main application, FreeRTOS tasks, on-off control, PID control |
| `config.c` | System clock, GPIO, I2C, SPI, TIM3 PWM configuration |
| `mpu6050_driver.c` | MPU6050 initialization, raw sensor read, calibration |
| `IMU.c` | Raw data scaling, complementary filter, motor PWM functions |
| `keypad.c` | 4x4 keypad scanning, target angle entry, mode selection |
| `MAX7219.c` | 7-segment display driver functions |
| `I2C_driver.c` | Blocking I2C register read/write functions |
| `spi_communication.c` | SPI transfer function for MAX7219 |
| `FreeRTOSConfig.h` | FreeRTOS configuration |

---

## Peripheral Configuration

| Peripheral | Usage |
|---|---|
| I2C1 | MPU6050 communication |
| SPI2 | MAX7219 display communication |
| TIM3 CH1 / CH2 | PWM outputs for motor direction control |
| GPIOC PC0-PC3 | Keypad row outputs |
| GPIOC PC4-PC7 | Keypad column inputs with pull-up |

The motor is controlled with two PWM channels. One channel is active for one direction and the other channel is active for the opposite direction.

```c
void motorRight(uint32_t speed)
{
    if (speed > 3500) speed = 3500;
    TIM3->CCR1 = speed;
    TIM3->CCR2 = 0;
}

void motorLeft(uint32_t speed)
{
    if (speed > 3500) speed = 3500;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = speed;
}
```

The PWM command is limited to **3500** to prevent overly aggressive motor movement and reduce mechanical overshoot.

In addition to normal direction control, the firmware also uses an active motor brake function:

```c
void motorBreak(void)
{
    TIM3->CCR1 = 2200;
    TIM3->CCR2 = 2200;
}
```

When both motor driver inputs receive the same PWM command, the motor is not commanded to rotate in a direction. Instead, the H-bridge enters a braking/holding condition depending on the motor driver behavior. This reduces free coasting after the rod reaches the target region. In this project, this braking behavior made the on-off controller more consistent around the reference angle than a simple `motorStop()` command.

---

## Keypad User Interface

The keypad is used for target angle entry and controller mode selection.

| Key | Function |
|---|---|
| `0-9` | Enter angle digits |
| `*` | Select negative sign |
| `#` | Confirm target angle |
| `C` | Clear current input |
| `A` | Toggle on-off control mode |
| `B` | Toggle PID control mode |

The confirmed target is limited in software to the project operating range:

```text
-60° <= target angle <= +60°
```

---

## Display Output

The MAX7219 driven 7-segment display is used to show both the target angle and the measured angle.

- Right digits: target angle entered from keypad
- Left digits: current angle estimated from MPU6050

This makes it possible to observe the reference and feedback values during operation.

---

## Angle Measurement and Filtering

The MPU6050 provides accelerometer and gyroscope measurements. In this project, accelerometer X/Y data and gyroscope Z data are used to estimate the platform angle.

### Why Filtering Is Needed

Using only the accelerometer is not reliable because motor vibration and mechanical movement can create noisy acceleration readings. Using only the gyroscope is also not reliable because gyroscope offset causes drift after integration.

Therefore, the project uses a complementary filter:

```text
Gyroscope: fast short-term response, but drifts over time
Accelerometer: long-term angle reference, but noisy under vibration
Complementary filter: combines both advantages
```

### Filtering Steps

1. MPU6050 internal DLPF is enabled.
2. At startup, 500 samples are collected for calibration.
3. Gyroscope Z offset is calculated and subtracted from future gyro measurements.
4. Initial accelerometer angle offset is calculated and used as the zero reference.
5. Raw accelerometer data is converted to `g` units.
6. Raw gyroscope data is converted to degrees/second.
7. Accelerometer angle is calculated using `atan2(Ay, Ax)`.
8. Gyro rate is integrated using the 10 ms control period.
9. Gyro prediction is slowly corrected with accelerometer angle.

### Complementary Filter Equation

The implemented filter is based on this logic:

```c
accelAngle = atan2(Accel_Y_g, Accel_X_g) * RAD_TO_DEG;
accurate_angle = wrap180(accelAngle - accel_angle_offset);

predicted = currentAngle + (-gyroRate) * dt;
acc_error = wrap180(accurate_angle - predicted);

currentAngle = predicted + (1.0f - ALPHA) * acc_error;
```

The filter coefficient is:

```c
#define ALPHA 0.98f
```

This means that the angle estimate mostly follows the gyroscope prediction, while the accelerometer provides a slow correction:

```text
Approximately 98% gyro-based prediction
Approximately 2% accelerometer-based correction
```

This choice is suitable for a motor-driven mechanism because the accelerometer can be affected by vibration, while the gyroscope gives smoother short-term motion information.

---

## On-Off Control Algorithm

The on-off controller uses the sign and magnitude of the angle error.

```text
error = target angle - measured angle
```

If the error is greater than the deadband, the motor is driven in one direction. If the error is smaller than the negative deadband, the motor is driven in the opposite direction. If the error is inside the deadband, the firmware does not simply leave the motor free. Instead, it calls `motorBreak()` and applies active braking.

```text
if error > deadband      -> motorRight()
if error < -deadband     -> motorLeft()
else                     -> motorBreak()
```

The deadband used in the firmware is:

```c
float deadband = 5.0f;
```

The reason for using a deadband is to avoid continuous right-left switching when the platform is already close to the desired angle. Without a deadband, even a very small sensor noise or mechanical vibration could make the motor change direction repeatedly.

### Active Brake Logic in On-Off Mode

A basic on-off controller usually stops the motor when the error enters the target region. In this project, the on-off controller uses `motorBreak()` inside the deadband instead of only setting both PWM channels to zero.

```c
void motorBreak(void)
{
    TIM3->CCR1 = 2200;
    TIM3->CCR2 = 2200;
}
```

This brake command helps reduce the remaining movement caused by inertia after the rod reaches the target band. It also prevents the motor from freely coasting around the reference angle. For this physical setup, the on-off mode was therefore observed to be more consistent than a basic on-off implementation, especially because the brake action helps the rod hold its position inside the deadband.

In summary, the on-off algorithm in this project is not only a simple direction switch. It has three states:

```text
1. Drive right if the error is clearly positive
2. Drive left if the error is clearly negative
3. Apply motor brake if the system is close enough to the target
```

This makes the on-off behavior simple, fast, and mechanically more stable for the current motor-platform system.

---

## PID Control Algorithm

The PID controller calculates the motor command from the angle error. Unlike on-off control, the PID controller adjusts the PWM command according to the magnitude and dynamics of the error.

Current PID gains:

```c
float Kp = 59.21f;
float Ki = 250.51f;
float Kd = 2.793f;
float N  = 50.0f;
```

### PID Terms

| Term | Purpose |
|---|---|
| P | Reacts to the current error |
| I | Reduces steady-state error |
| D | Reacts to the rate of error change and helps reduce overshoot |

The firmware includes several practical improvements for real hardware behavior.

### Target Ramping

The target angle is not applied instantly. Instead, it is moved gradually toward the final target angle.

```text
ramped_target -> final_target_angle
```

This reduces sudden motor commands and makes the motion smoother.

### PID Deadband Near the Target

The PID controller also includes a small deadband around the target angle. This is intentional and important for the real hardware. An ideal PID controller would keep reacting to very small errors, but in a physical motor system those very small errors may come from IMU noise, mechanical vibration, backlash, or display/measurement quantization rather than a meaningful position error.

The implemented PID deadband is approximately **±1.5°**:

```c
if(error > -1.5f && error < 1.5f)
{
    error = 0.0f;
    i_sum = 0.0f;
}
```

When the error is inside this range, the firmware treats the system as close enough to the target. The error is set to zero and the integral memory is cleared. This prevents the PID controller from producing very small alternating PWM commands near the reference angle.

The main reasons for adding a deadband even to the PID controller are:

- to reduce motor jitter near the target,
- to prevent unnecessary right-left micro-corrections,
- to avoid integral build-up caused by very small residual errors,
- to reduce sensitivity to IMU noise and mechanical vibration,
- to make the final holding behavior smoother and safer for the mechanism.

Outside this ±1.5° region, the PID controller works normally and calculates the PWM command from the P, I, and filtered D terms. Inside this region, the controller considers the target reached and the motor is stopped.



### Integral Anti-Windup

The integral sum is limited to prevent excessive integral accumulation:

```c
if(i_sum > 2.0f)  i_sum = 2.0f;
if(i_sum < -2.0f) i_sum = -2.0f;
```

This prevents integral windup and helps reduce overshoot.

### Filtered Derivative Term

The derivative term is filtered using the coefficient `N`:

```c
float d_out = (Kd * N * (error - prev_err) + prev_d_out) / (1.0f + N * dtPid);
```

`N` is not part of the complementary filter. It belongs to the PID derivative filter. The D term is sensitive to sensor noise, so filtering prevents sudden noisy changes from creating aggressive PWM commands.

```text
N = 50
```

A larger `N` generally makes the derivative term more responsive but more sensitive to noise. A smaller `N` makes the derivative term smoother but slower. The selected value is a practical compromise for this motor-platform system.

### Minimum PWM

A DC motor may not move at very low PWM values due to static friction. Therefore, the controller adds a minimum PWM when the error is outside the small dead zone:

```c
float min_pwm = 1900.0f;
```

---

## Safety and Recovery Logic

The target angle is limited to the project range of -60° to +60°. Additional software limits are also used to stop or recover the system if the measured angle goes outside the expected safe region.

In PID mode, if the platform angle becomes larger than approximately ±80°, recovery mode is activated. During recovery, PID memory variables are reset and the motor drives the platform back toward the safe region.

---

## MATLAB / Simulink Model

A MATLAB/Simulink closed-loop model was created to analyze the PID controller and disturbance rejection behavior before applying the controller to the physical system.

<img width="1749" height="465" alt="Ekran görüntüsü 2026-05-31 161524" src="https://github.com/user-attachments/assets/ddb516dc-6b05-45ee-82a8-181869b44851" />


### Model Structure

The Simulink model represents the following closed-loop path:

```text
Reference angle
      |
      v
Error calculation
      |
      v
PID controller
      |
      v
Saturation / actuator limit
      |
      v
PWM-to-voltage gain: 6 / 3500
      |
      v
DC motor/platform transfer function
      |
      v
Radian-to-degree gain: 180 / pi
      |
      v
Output angle feedback
```

The transfer function used in the model is:

```text
G(s) = 443.8 / (s^2 + 84.4s)
```

The model also includes an output disturbance input. A disturbance step with value `15` is applied at `t = 5 s` to observe how the closed-loop system returns to the reference angle.

### Step Response

The step response shows that the output reaches the 60° target with a small overshoot and then settles close to the reference.

<img width="1562" height="804" alt="image" src="https://github.com/user-attachments/assets/de94dbf4-73d6-44ef-91fb-911f530f5518" />


### Disturbance Rejection

When a disturbance is applied at 5 seconds, the output angle is temporarily disturbed, but the closed-loop controller brings the response back to the 60° reference.

<img width="1280" height="874" alt="image" src="https://github.com/user-attachments/assets/a875fadf-6367-45ee-9f1d-fe2547a74b35" />


### Step and Disturbance Response

The full simulation shows both the 0° to 60° reference tracking and the disturbance rejection behavior in the same run.

<img width="1540" height="796" alt="image" src="https://github.com/user-attachments/assets/29badd89-e4a9-41e5-8415-2e60a1f3f71a" />




## Demo Video
https://www.youtube.com/watch?v=SxqxC8VSgxw


