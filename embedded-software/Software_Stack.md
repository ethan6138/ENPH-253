# Software Stack — ENPH 253 Team 12

Firmware for an autonomous competition robot on an ESP32-S3. This document describes the
system **as built**, plus the design decisions behind it and what changed from the
original plan.

## Hardware Context

| | |
|---|---|
| **MCU** | ESP32-S3 (dual core), PlatformIO, Arduino framework on ESP-IDF 5.5 (pioarduino) |
| **Drive** | Differential drive, two geared DC motors, quadrature encoders, caster |
| **Sensing** | 5-channel phototransistor tape array (comparator outputs), MPU6050 IMU, Colpitts oscillator metal detector |
| **Vision** | CanMV K230 over UART: object class, confidence, bounding box (JSON) |
| **Actuators** | 3-DOF arm (two servos + stepper turret), claw servo |

## File Structure

```
ENPH253-T12-MAIN/
├── platformio.ini
├── src/
│   ├── main.cpp            # setup() → app(); loop() unused
│   ├── app.cpp / app.h     # init + mission sequencer
│   └── test.cpp / test.h   # bench tests (drive, arm, dashboard)
├── include/
│   ├── pins.h              # every pin assignment, in one place
│   ├── types.h             # shared structs and enums
│   ├── helpers.h           # unit conversions, angle wrapping, servo PWM math
│   └── error_handler.h     # placeholder for thread-safe fault reporting
└── lib/
    ├── Drivetrain/         # drive task + modes: tape, pose, camera, blind, raw
    │   ├── DriveTrain.*        # mode switch + shared inner loop
    │   ├── WheelPID.*          # per-wheel PID with anti-windup
    │   ├── FeedForwardTable.h  # calibrated duty-vs-RPM tables
    │   ├── TapeFollowEncoder.* # tape PD, crossing detection (ISRs)
    │   ├── TapeFollowNoEncoder.*
    │   ├── DriveToPose.*       # position controller
    │   ├── K230Align.*         # camera-based alignment
    │   └── MotorDriver.*       # H-bridge PWM output
    ├── Encoder/            # quadrature ISRs + RPM task
    ├── Imu/                # MPU6050 task + Kalman filter
    ├── Pose/               # odometry + heading fusion + ramp re-anchoring
    ├── Arm/                # arm task, IK/FK, claw, turret camera-align
    ├── K230/               # camera UART task + JSON parsing
    ├── Oscillator/         # PCNT-based metal detection
    ├── Telemetry/          # serial telemetry out + tuning commands in
    └── Sonar/ Beacon/ Turret/ Light/   # stubs from the original plan (unused)
```

## Shared Types

All cross-module types live in `include/types.h`, namespaced by owner:

```cpp
struct Vector3D { float x, y, z; };                 // arm targets, cm

namespace ArmParameters {
  enum class ArmPreset { RockPickup, SolarPanel, HabitatPiece,
                         DropOff, Home, DriveHome };
  struct ArmAngles { float alpha{90}, beta, gamma; };   // degrees
}

namespace Imu {
  struct IMUAngles { float roll, pitch; };          // degrees
  struct IMUAccel  { float x, y, z; };
}

namespace Pose {
  enum class ResetPoint { Start, RampBottom, RampTop };
  struct PoseState { float x, y, yaw; };   // cm from top-left of field; yaw in rad from +y
}

namespace Wheels {
  enum class WheelSide { LEFT, RIGHT };
  struct RpmTargets { float rpmLeft, rpmRight; };   // what every drive mode outputs
}

namespace K230 {
  enum class AlignTargetClass { ROCK, TELETUBBY, LOOP };
}
```

`Wheels::RpmTargets` is the key interface: every drive mode returns one, which is what
lets all modes share a single wheel PID loop.

## Architecture

The firmware has three layers. Lower layers never know about the mission.

```
┌──────────────────────────────────────────────┐
│ Mission sequencer (app.cpp)                  │  scripted phases, blocking calls
├──────────────────────────────────────────────┤
│ Commands: driveToDeltaAndWait, driveFollow-  │  each starts a mode or target,
│ Tape, moveToPreset, closeClaw, ...           │  "AndWait" variants block until done
├──────────────────────────────────────────────┤
│ Background tasks: drive, arm, pose, imu,     │  fixed-period loops that own the
│ encoder, camera, oscillator, telemetry       │  hardware and the control math
└──────────────────────────────────────────────┘
```

**Rule:** modules don't call each other. `app.cpp` wires them together, so reading
`app.cpp` shows the whole flow. The exceptions are read-only state getters (for example
the drivetrain reading `getPose()`, the wheel PID checking ramp state), which go through
mutex-protected accessors.


## Constraints

**Design rules**
- Modules don't call each other; `app.cpp` wires them together. Cross-module reads go
  through mutex-protected getters only.
- Hardware is owned by exactly one task (drive task → motors, arm task → servos and
  stepper, IMU task → I²C). Everyone else sends targets, never writes hardware directly.
- ISRs only count or flag (encoder ticks, tape crossings, PCNT overflow). No math, no
  blocking, no `Serial` in an ISR; work happens in the task that reads the result.
- `encoder_init()` must run before `driveTrain_init()`, since the drive task reads encoder RPM.

**Hardware limits that shaped the code**
- **PWM:** 12-bit resolution (0–4095); servos at 100 Hz with 500–2500 µs pulses over 270°.
- **Motors:** each wheel has a minimum duty cycle below which it stalls under load, so the
  feedforward table starts there instead of at zero.
- **PCNT counter is 16-bit,** so oscillator counts are extended with an overflow
  accumulator to support longer sample windows.
- **Encoders:** 1 ms debounce window (one tick is ~8.3 ms at 100 RPM), so real edges are
  never rejected but noise spikes are.
- **Arm:** joints move at most 1° per 5 ms tick to limit current draw and keep motion smooth.
- **Claw:** closing intentionally stalls the servo for grip, so grip time is kept short to
  avoid overheating.

### Why a scripted sequencer instead of an event system

The original plan was event-driven: background checks raise events that interrupt
whatever is running. We switched to a linear, blocking sequencer because the course is
fixed and known in advance. A script is easier to reason about, to debug one step at a
time, and to re-tune between runs, and it avoids handling every event in every state.
Real-time work still happens in the background tasks, so blocking the sequencer never
stalls a control loop.

## Concurrency Model

Core 0 runs sensing and background work. Core 1 runs control and actuation, so a burst of
sensing work can't delay a motor update. All tasks are created in `app_init()` and never
deleted.

| Task | Core | Period | Stack | Responsibility |
|---|---|---|---|---|
| `imuTask` | 0 | 10 ms | 4 KB | Read MPU6050, Kalman-filter roll/pitch, publish yaw rate |
| `poseTask` | 0 | 20 ms | 4 KB | Odometry + heading fusion → pose (x, y, θ); ramp detection |
| `encoderTask` | 0 | 100 ms | 4 KB | Convert ISR tick counts to wheel RPM |
| `telemetryTask` | 0 | 150 ms | 4 KB | Stream state over serial; accept tuning commands |
| `driveTask` | 1 | 20 ms | 4 KB | Active drive mode → per-wheel PID → motor PWM |
| `armTask` | 1 | 5 ms | 8 KB | Rate-limited joint motion, waypoint queue, claw state |
| `k230Task` | 1 | 10 ms | 4 KB | Parse camera detections from UART |
| `oscillatorTask` | 1 | 100 ms | 4 KB | Metal detection via PCNT (only when armed) |
| `missionSequencer` | 1 | — | 64 KB | The scripted mission (large stack for deep call chains) |

All tasks run at the same priority (1), so tasks on a core time-slice round-robin. Loops
use `vTaskDelay`, except `armTask`, which uses `vTaskDelayUntil` for a fixed 5 ms period.

### Shared-state protection

- **ISR ↔ task:** encoder tick counts and the oscillator overflow accumulator are written
  in ISRs (`IRAM_ATTR`) and protected with spinlocks (`portENTER_CRITICAL_ISR` /
  `portENTER_CRITICAL`).
- **Task ↔ task:** pose, IMU angles, camera detections, and arm angles are each guarded by
  a FreeRTOS mutex and exposed only through getters and setters.
- **Simple flags** (drive mode, oscillator sampling enable, ramp detected) are single-word
  `volatile` values.
- **Blocking calls:** `...AndWait()` functions poll their condition every 10 ms with
  `vTaskDelay`, so the sequencer yields the core while it waits.

## Drivetrain Control

Every encoder-based drive mode produces **target wheel RPMs**, and one shared inner loop
turns those into PWM.

```
 mode (tape / position / camera)  →  [target RPM L, R]
                                         │
                       feedforward table + PID  ← encoder RPM
                                         │
                                     motor PWM
```

**Drive modes** (`DriveMode`): `TAPE`, `POSITION` (absolute pose or relative delta),
`CAMERA` (align to a detected object), `BLIND` (fixed RPM), `RAW` (direct duty cycle for
testing), and `TAPE_NO_ENCODER` (fallback that drives duty cycle straight from tape error).

**Per-wheel PID** (`WheelPID.cpp`):
- **Feedforward:** a per-wheel duty-cycle table calibrated on the robot under load gives
  most of the output; PID only corrects the remainder. A boosted table is used on the
  ramp.
- **Anti-windup:** the integral contribution is clamped, and the integral is reset when
  the target jumps by more than 10 RPM or goes to zero.
- Gains are tunable live over serial.

**Tape following:** sensor positions are weighted to compute a lateral error, a PD loop
turns it into differential RPM targets, and the last known direction is remembered to
recover when the line is lost. Outer sensors trigger ISRs used to detect tape crossings.

**Camera alignment:** PD on the bounding box's pixel offset from a target position, with
deadbands, producing the same RPM targets.

## Pose Estimation

`poseTask` integrates wheel odometry every 20 ms. Heading is fused with a 2-state
(angle, gyro bias) Kalman filter: the **gyro yaw rate drives the prediction** and the
**encoder-derived heading is the measurement**.

$$\Delta\theta_{enc} = \frac{d_R - d_L}{b}, \qquad
x \mathrel{+}= \bar{d}\,\sin\theta, \qquad y \mathrel{+}= \bar{d}\,\cos\theta$$

where $d_L, d_R$ are wheel travel distances, $\bar{d}$ their mean, and $b$ the track width
(23 cm). Heading is measured from +y, counter-clockwise positive.

- **Stall gating:** when both wheels are stationary the gyro rate can be zeroed to stop
  drift while stalled (controlled by `stationaryTickThreshold`).
- **Ramp re-anchoring:** pitch from the IMU detects the bottom and top of the ramp, and
  pose snaps to known field coordinates at each, clearing accumulated drift.

The Kalman filter implementation is adapted from TKJ Electronics' open-source filter
(Kristian Lauszus, GPL-2).

## Arm

3-DOF arm: shoulder (α) and elbow (β) servos plus a stepper turret (γ), 12.7 cm links.

**Inverse kinematics** (`InverseKinematics.cpp`) solves a target point $(x, y, z)$
geometrically:

$$r = \sqrt{x^2 + y^2}, \quad z' = z - h, \quad D = \sqrt{r^2 + z'^2}, \quad
\gamma = \operatorname{atan2}(-x, y)$$

α and β follow from the law of cosines on the two-link triangle. Targets out of reach are
rejected and the arm holds its current pose. The arm is constrained to one solution per
target.

**Motion:** `armTask` runs every 5 ms and moves each joint toward its target by at most
1° per tick, so motion is smooth and current draw stays bounded. A waypoint queue
sequences multi-step moves, and named presets (`Home`, `DriveHome`, `RockPickup`,
`DropOff`, …) cover common positions.

## Sensors and Peripherals

- **Encoders:** quadrature decoded in ISRs with a lookup table and a debounce window.
- **IMU:** MPU6050 over I²C; Kalman-filtered roll and pitch with mounting offsets.
- **Camera:** the K230 streams JSON detections over UART; `k230Task` parses them into a
  mutex-protected list (label, score, bounding box).
- **Metal detector:** the ESP32 PCNT peripheral counts oscillator edges over a 50 ms
  window; an ISR extends the 16-bit counter. A frequency shift past threshold flags foil.

## Telemetry and Tuning

`telemetryTask` sends one line of `Key:value` pairs every 150 ms (IMU, pose, arm, wheel
RPMs and duty, tape sensors, oscillator, camera). It also reads `KEY:value` commands to
change PID gains, drive targets, and arm position live. A browser dashboard (HTML + Web
Serial, built with AI assistance) plots the stream and sends commands.

## Mission (as run)

1. **Rocks 1–6.** For each rock: align to it with the camera, approach, watch for a
   teletubby (flash the light if confident), and check for foil with the metal detector.
   Foil rocks are picked up with the arm; others are skipped. The robot tape-follows up
   the ramp between rocks 4 and 5, re-anchoring pose at the bottom and top.
2. **Solar panel.** Camera-align to the panel's loop, align the turret, hook the loop with
   the claw, and pull the cover off.
3. **Habitat.** Implemented but disabled for competition: it didn't get enough testing time.

Each phase can be run on its own from `app.cpp` for testing.

## What Changed From the Original Plan

| Planned | Shipped | Why |
|---|---|---|
| Event-driven protocols | Scripted blocking sequencer | Known course; simpler to debug and tune |
| Sonar + IR beacon for phase 2 | Camera PID with the K230 | Vision was more reliable and reused one pipeline |
| Python + WebSocket dashboard | HTML + Web Serial | No backend to run; opens straight from the browser |

## Lessons

- **Test time beats features.** Phase 3 was written but never trusted on game day.
- **A shared inner loop pays off.** Every drive mode only outputs RPM targets, so tuning
  the wheel PID once improved everything.
- **Telemetry first.** Live plots and on-the-fly gain changes made tuning dramatically
  faster than re-flashing.