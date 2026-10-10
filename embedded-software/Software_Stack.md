# Full Software Stack

## Hardware Context 

- **MCU**: ESP32 S3 R8N8
- **Platform**: PlatformIO, pioarduino fork, pinned to 55.03.30 / core 3.3.0 / IDF 5.5 
- **Drive**: Differential drive, two geared DC motors with rotary encoders + caster 
- **Sensors**: 5 photo transistors, comparator, IMU, Colpitts Oscillator + comparator, 
  sonar sensor, IR beacon
- **Vision**: CanMV K230 over UART, sends events/classification + bounding box continously
- **Actuators**: 3-DOF arm on turret, claw servo, indicator light

## File Structure

```cpp
platformio.ini
├── src/
│   ├── main.cpp
│   ├── app.cpp
│   └── app.h
├── include/
│   ├── pins.h
│   ├── types.h
│   ├── shared_state.h
│   └── error_handler.h
└── lib/
    ├── Drivetrain/
    │   ├── Drivetrain.h
    │   └── Drivetrain.cpp
    ├── TapeFollow/
    │   ├── TapeFollow.h
    │   └── TapeFollow.cpp
    ├── Arm/
    │   ├── Armmove.h
    │   └── Arm.cpp
    ├── Turret/
    │   ├── Turret.h
    │   └── Turret.cpp
    ├── Beacon/
    │   ├── Beacon.h
    │   └── Beacon.cpp
    ├── Sonar/
    │   ├── Sonar.h
    │   └── Sonar.cpp
    ├── Camera/
    │   ├── Camera.h
    │   └── Camera.cpp
    ├── Oscillator/
    │   ├── Oscillator.h
    │   └── Oscillator.cpp
    ├── Imu/
    │   ├── Imu.h
    │   └── Imu.cpp
    ├── Pose/
    │   ├── Pose.h
    │   └── Pose.cpp
    ├── Light/
    │   ├── Light.h
    │   └── Light.cpp
    └── Telemetry/
        ├── Telemetry.h
        └── Telemetry.cpp
```

## Shared Types 

```cpp
enum class Phase {
  init, 
  phase_1,
  phase_2,
  phase_3, 
  done
};

enum class ObjectType {
  rock, 
  teletubby,
  solar, 
  habitat,
  spot_ring
};

struct SpotObject{ 
  ObjectType type{}; 
  float bbox_x{}; 
  float bbox_y{}; 
}; 

struct TapeState {
  bool sensors[5]; 
};

struct Pose{
  float x{}; 
  float y{}; 
  float theta{}; 
};

struct ArmPosition{
  float alpha{};
  float beta{}; 
  float gamma{}; 
};

// Named ArmPosition presets — pass these into move_arm_to_position()
// BEACON   : arm positioned for initial beacon approach in phase 2
// SWEEP    : arm positioned for turret sweep during beacon acquisition
// PICKUP   : arm extended down for rock/habitat piece pickup
// STORE    : arm retracted to carry position
// SOLAR    : arm positioned to hook solar panel fabric loop
// BASKET   : arm positioned over basket for cover drop
// HOME     : fully retracted safe position
```

## Module Communication Rule 

All communication routes exclusively through `app.cpp`. No lib modules inlcudes or calls another lib module directly. `app.cpp` is where the command flow occurs, it calls modules, modules never call eachother. This keeps dependencies explicit and tracable. Reading `app.cpp` should tell you what talks to what. 

### Execptions 

Two execptions to the rule above

- `shared_state.h` - `turret_angle` float + its mutex. Written by `Turret::task()` in phase 2 synchronous PID, read by `Drivetrain::task()`. Both access through `get_turrent_angle() / set_turret_angle()` wrappers that handle the mutex internally, the raw variable is never mutated directly. 
- `Arm.cpp` includes `<Turret.h>` and calls `Turret::set_angle(gamma)` directly for the gamma step of `move_arm_to_position` One directional and narrow: Arm calls one Turret function, Turret knows nothing about the Arm. Temporally safe too since arm positioning only happens in phases 1 and 3, turret PID task only runs in phase 2, meaning they never conlfict on the physical mtoor. 

everything else routes through `app.cpp` exclusively. 

## Layer 1- Atomic Commands 

These are building blocks that we use. Each does exactly one thing: They don't know about phases or events. 

`follow_tape` - TapeFollow + Drivetrain
Cascaded PID. outer loop reads up 5 GPIO comparator pins, computes discrete tape errors(which sensors are on/off tape), outputs differential speed corrections. Inner loop (per wheel) takes those targets and closes the loop against encoder feedback, output motor PWM. Has no natural end - runs until an event interrupts it. 

```
5x GPIO comparator reads
  -> tape error (discrete)
  -> tape PID (outer loop) 
  -> [left_target, right_target] RPM 
  -> per-wheel speed PID <- encoder feedback 
feedback
  -> motor PWM 
```

`drive_blind(x, theta)` - Drivetain 
Converts velocity + angular rate to per-wheel RPM target via differential drvie kinematics, feeds into the same per-wheel speed PID inner loop. Continous velocity command with no natural end - caller is responsible for termination (either a timed duration from `millis()` or watiing for a `spot_tape` event to fire) 

`drive_to_position(delta_x, delta_y, delta_theta)` - Drivetrain + Pose 
Closed-loop position PID against the pose estimate. Natural end when position error falls within tolerance threshold. Reference point is midpoint of rear axle (center of rotation for differential drive).

`drive_based_on_camera` - Drivetrain + Camera 
Continous PID directly against k230 bounding box x-offset from image center. PID outputs heading corrections, combined with forward velocity component. Runs util an external condition terminates it(sonar threshold, foil detection) Coordinate convention x is lateral, y is forward. aligns camera frame with robot frame directly, no transformation needed. 

`try_detect_foil` - Oscillator 
Reads Colpitts oscillator output (frequency shift detected via hardware comparators). Returns bool only. No arm movements nested inside since those live in the event sequence layer above.

`move_arm_to_position(ArmPosition)` - Arm + Turret for gamma 
Internal servo sequencer handles safe movement oder: beta first (forarm) then beta + gamma synchronized (bicep + turret). All angles will have presets positions for specific task types.gamma is almost always dynamically calcualted- execpt when storing rock. Calls `Turret::set_angle(gamma)` for the gamma step. 

`open_claw` / `close_claw` - Arm 
Sets claw servos PWM and holds it indefinitely until superseded. close_claw intentially stalls at fully closed for maximum grip torque. Short grip duration should be achieve for high stall current draw and overheating issues. 

`flash_light(bool)` - Light 
- If true, we flash indicator LED 3 times.
- If false no-op. Guard logic against false positives since one mishaps looses us all teletubby points.... 

`sample_beacon` - Beacon + Pose 
ADC sampling of IR beacon signal, cross-correlation signal processing, combined with current pose estimate. Returns signal strength (used by turret PID in phase 2) and/or beacon world position esitmate. 

## Layer 2 - Event-Based Sequences 

Events interrupt whatever command protocol is currently running and triggers a named protocol (we wil need to add error handlers into this. for example, in phase 3, it makes no sense to detect a rock). Events are monitored by always running background checks. 

### Events 

| Event | Source | Trigger Conditions
| --- | --- | --- |
| spot_tape | TapeFollow task | Any of 5 GPIO sensors goes high | 
| spot_double_tape | TapeFollow task | one side all-sensors-high pulses within time frame
| spot_object | Camera task | K230 sends classification over UART -> fires with SpotObject payloads 
| up_ramp_detected | IMU task | IMU pitch exceeds threshold - triggers pose recalibration
| top_of_ramp_detected | IMU task | IMU pitch returns to flat after ramp - also triggers pose recalibration 

### Protocols

See draw.io diagram for full sequence detail — each phase is a separate linked page.
Summary of what fires what:

| Protocol | Trigger | Phase active |
|---|---|---|
| Rock approach + foil check | `spot_object → ROCK` | Phase 1 |
| Teletubby flash | `spot_object → TELETUBBY` | Phase 1, fires anytime in parallel |
| Ramp recalibration | `up_ramp_detected` / `top_of_ramp_detected` | Phase 1, parallel with `follow_tape` |
| Solar panel approach | `spot_double_tape` | Phase 2 |
| Habitat placement | `spot_double_tape` | Phase 3 |

## Layer 3 — Mission Sequencer (`app.cpp`)

Three sequential phases. `current_phase` is a `volatile Phase` settable manually in 
`app_init()` for debugging — boot straight into any phase without running the ones before it.

Mission state variables, `static` at file scope in `app.cpp`:

```cpp
static bool rock_foil = false;           // true once foil rock collected
static int  rock_counter = 0;            // increments each rock visited (0-6)
static bool panel_cover_removed = false; // true once solar cover in basket
```

See draw.io diagram for full phase sequences. Transition conditions:

| From | To | Condition |
|---|---|---|
| Phase 1 | Phase 2 | `rock_counter == 6` |
| Phase 2 | Phase 3 | `panel_cover_removed == true` |
| Phase 3 | Done | match timeout |

## Concurrency Model

### Always-running tasks (created in `app_init()`, never deleted)

| Task | Core | Rate | Purpose |
|---|---|---|---|
| `Camera::task` | 0 | ~debate | Continuous UART poll from K230, parses `SpotObject`, sets fresh detection flag |
| `Pose::task` | 0 | ~debate | Encoder + IMU complementary filter fusion, updates `(x, y, θ)` continuously |
| `TapeFollow::task` | 0 | ~debate | GPIO reads, event detection, feeds outer tape PID |

### Phase-gated tasks (created in `app_init()`, activated by task notification)

| Task | Core | Active during | Purpose |
|---|---|---|---|
| `Turret::task` | 1 | Phase 2 only | Beacon PID → `set_angle()` → writes `shared_state` |
| `Drivetrain::heading_task` | 1 | Phase 2 only | Reads `shared_state` → heading PID → drive correction |
| `Sonar::task` | 0 | Phase 2 only | Distance threshold → fires event to kill phase 2 tasks |

### Synchronous blocking calls from `app.cpp`
Sequential commands that block until done — `move_arm_to_position`, `open_claw`, 
`close_claw`, `flash_light`, timed `drive_blind` segments. No separate FreeRTOS task needed.

### Shared resource protection
- `shared_state.turret_angle` — only via `get_turret_angle()` / `set_turret_angle()` wrappers
- `Camera` latest `SpotObject` — mutex protected
- `Pose (x, y, θ)` — mutex or double-buffer to avoid partial reads mid-update

## Dashboard / Telemetry 

No clue how to do this yet! probably a python backend oepning up usb serial to esp32, hosts local websocket server at `localhost: 8000`. Browser forntend shows live plots grouped by segments, so something like `DRVIE.ERR` and `DRIVE.OUT` or somth. configurable PID tuning sliders (hopefully) with editable min/max and descriptions, raw command text boxes, with live values panel, with a debug log. 

esp32 side: `Telemetry` lib module wraps up `dashboard_link.h`. Each module registers its turnable gians once in `app_init()` via a struct. then each task will print one telemetry line per tick via afew functions. then Dashboard sends key values back; we will poll those commands each tick live. 

Serial Protocol: 

```
ESP32 -> laptop: 

Laptop -> ESP32: Set 
ESP32 -> laptop: OK 
```

## Key Known Constraints 
- **ADC Ceiling**: ESP32-S3 continous adc max is at around 83,333Hz. we can match it up to internal clock speed but resolution drops and garbage values occur. Hence Colpitts' oscillator has been converted to hardware comparator 
- `adc_continuous_data_t` **buffers**: declare as `static`, not locals stack overflow otherwise 
- **Arduino ESP32 cire 3.x** `Wire.begin()` moved out of global initializer scope(must be in `setup()`), LEDC API changed (`ledcAttach`/`ledcWrite(pin, ....)`), ADC API moved to `esp_adc/adc_oneshot.h` 
- **Wifi + ADC2**: these two conflict. cannot run both at same time. telemtry is wired USB only 
- **Reverse tape following**: Physically not possible with front mounted sensors. we gotta rotate 
- `drive_blind` **termination**: Continous velocity command with no natural ends. Never pre-compute encoder ticks as a termination condition - `use micros()` duration or wait for a `spot_tape` event. 
- **PlateformIO LDF cross-lib includes**: `Arm.cpp` includes `<Turret.h>` with angle bracets (not quotes) for LDF to correctly resolve the cross-lib dependency. if "undefined reference" linker errors show up on cross-lib includes, check lin_ldf_mode in platformio.ini
- `volatile Phase current_phase`: settable manually in `app_init()` to boot directly into any phase for isolated testing. Gonna need to implement this or need an override