Team 12 ENPH 253 2026 Full Repository



# ENPH 253 Autonomous Robot — Team 12 (2026)

Autonomous competition robot for UBC's ENPH 253 (summer 2026), built by a team of four.
The robot follows tape, finds and classifies objects with an onboard camera, picks them
up with a 3-DOF arm, and runs a multi-phase mission on its own.

Video run: https://github.com/user-attachments/assets/4afef0d1-7076-4fb5-ac5a-32cf80194094




## Highlights

- **9-task FreeRTOS firmware** on an ESP32-S3, split across both cores: sensing and
  background work on core 0, control and actuation on core 1
- **Concurrency done carefully:** ISR-safe encoder and tape counters protected with
  spinlocks (`portENTER_CRITICAL_ISR`), and mutex-guarded getters/setters for all shared
  state (pose, IMU, camera detections, turret angle)
- **Cascaded control:** per-wheel PID velocity loops (feedforward table, anti-windup)
  under tape-following, drive-to-pose, and camera-alignment controllers
- **Sensor fusion:** Kalman filter fusing encoder heading with MPU6050 gyro data, gated on
  wheel motion to remove stall-induced yaw drift; pose re-anchored on ramp transitions
  using fused pitch
- **3-DOF arm** with forward/inverse kinematics and a rate-limited motion task, sequenced
  by a mission state machine
- **Live telemetry and tuning:** JSON telemetry over serial, with a browser dashboard for
  live plots and on-the-fly PID gain changes

## System Overview

| | |
|---|---|
| **MCU** | ESP32-S3 (PlatformIO, Arduino framework on ESP-IDF 5.5) |
| **Drive** | Differential drive, two geared DC motors with quadrature encoders |
| **Sensors** | 5-channel phototransistor tape array, MPU6050 IMU, Colpitts oscillator metal detector |
| **Vision** | CanMV K230 camera over UART (object class + bounding box), used for camera-guided PID |
| **Actuators** | 3-DOF arm (servos + stepper turret), claw servo |

### Firmware architecture

Firmware is layered so modules never call each other directly. `app.cpp` owns the flow.

1. **Atomic commands:** single-purpose building blocks (`follow_tape`, `drive_to_position`,
   `move_arm_to_position`, `open_claw`, …)
2. **Event protocols:** background tasks raise events (tape spotted, object detected,
   ramp detected) that trigger named sequences
3. **Mission sequencer:** a phase state machine; any phase can be booted directly
   for debugging

| Task | Core | Period | Role |
|---|---|---|---|
| `imuTask` | 0 | 10 ms | MPU6050 read + Kalman filtering |
| `poseTask` | 0 | 20 ms | Encoder + gyro fusion, pose (x, y, θ), ramp detection |
| `encoderTask` | 0 | 100 ms | Quadrature ticks → wheel RPM |
| `telemetryTask` | 0 | 150 ms | JSON telemetry out, tuning commands in |
| `driveTask` | 1 | 20 ms | Wheel PID + active drive mode |
| `armTask` | 1 | – | Rate-limited arm motion |
| `k230Task` | 1 | 10 ms | UART parsing of camera detections |
| `oscillatorTask` | 1 | 100 ms | Metal (foil) detection via pulse counter |
| `missionSequencer` | 1 | – | Phase state machine |

## Repository Layout

```
embedded-software/
  ENPH253-T12-MAIN/     # competition firmware (PlatformIO project)
  Labs+Test_Files/      # bring-up and validation sketches per subsystem
  DashBoard/            # browser telemetry dashboard (HTML + Web Serial)
  IMU_Plot/             # Python IMU plotting tool
electrical-hardware/    # Altium projects for the robot's boards
mechanical-hardware/
```

### Bring-up and validation

Each subsystem was validated on its own before integration, with a standalone sketch in
`Labs+Test_Files/`: H-bridge, rotary encoders, ADC/DMA, IMU, servo and stepper motors,
phototransistors, OLED, and K230 UART.

## Building

```bash
cd embedded-software/ENPH253-T12-MAIN
pio run -t upload       # set upload_port in platformio.ini for your machine
pio device monitor      # 115200 baud
```

## My Role

I designed the firmware architecture and wrote the robot's firmware: the FreeRTOS task
structure, drivetrain and wheel PID, tape following, encoder and IMU drivers, Kalman
pose estimation, arm kinematics and motion, the mission sequencer, and telemetry. I built
the browser dashboard with AI assistance. I also read the schematics for and soldered,
validated, tested, and harnessed the robot's five boards (H-bridge, vehicle controller,
stepper driver, sensor module, power distribution), which were designed by a teammate.

A teammate wrote the K230 camera alignment (adapted from my tape-following PID) and the
oscillator sampling code.
