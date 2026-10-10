# Arm Module — Calling Reference

Everything below is reachable through a single `#include <Arm.h>` — no other file in `lib/Arm/` needs to be included directly.

(0,0,0) for the arm is the floor below the center of rotation of the arm. 

## 1. Initialize once, at boot
```cpp
void app_init(void) {
  armInit(); // spins up armTask internally; nothing else needed
}
```

## 2. Move to raw joint angles, wait until arrived
```cpp
ArmParameters::ArmAngles homeAngles{0.0f, 0.0f, 0.0f};
moveToPositionAndWait(homeAngles);
```

## 3. Move to a real-world (x, y, z) coordinates, wait until arrived
```cpp
Vector3D rockTarget{10.0f, getSonarDistance(), 5.0f}; // y filled from sonar
moveToPositionAndWait(rockTarget);
```
IK runs internally. If the target is unreachable, the arm silently holds its current position instead of moving — check `getArmPosition()` afterward if you need to confirm it actually got there.

## 4. Fire-and-forget — start a move without blocking
```cpp
ArmParameters::ArmAngles homeAngles{0.0f, 0.0f, 0.0f};
moveToPosition(homeAngles);   // returns instantly
driveToSolarPanel();          // runs concurrently with the arm stowing
```

## 5. Claw — one-shot, no wait needed
```cpp
closeClaw();
openClaw();
```

## 6. Read where the claw currently is
```cpp
ArmParameters::ArmAngles current = getCurrentArmAngles(); // joint angles
Vector3D currentPosition = getArmPosition();               // real-world (x, y, z)
```

## 7. Realistic sequencer snippet — Phase 1 rock pickup
```cpp
void runPhase1_RockScanning() {
    // ... drive to rock, detect foil ...
    if (isFoilRock) {
        Vector3D rockTarget{10.0f, getSonarDistance(), 5.0f};
        moveToPositionAndWait(rockTarget);  // wait — need to be positioned before grabbing
        closeClaw();
        vTaskDelay(pdMS_TO_TICKS(300));     // grip settle time

        ArmParameters::ArmAngles homeAngles{0.0f, 0.0f, 0.0f};
        moveToPosition(homeAngles);          // fire-and-forget — stow while driving
        driveToNextObjective();              // overlaps with the stow motion
        openClaw();                          // drop rock in storage once arrived
    }
}
```

## Rule of thumb
If the next action depends on the arm having physically arrived → use `moveToPositionAndWait`.
If it doesn't → use `moveToPosition` and let the arm finish while something else happens.

## Not yet available
Preset positions (`ArmPreset::Home`, `ArmPreset::RockPickup`, etc.) are planned but not built — the enum exists in `types.h` but there's no storage or getter for preset angles yet, and no way for Sonar/Camera to patch a sensor-dependent field into one. Until that lands, build targets by hand as shown above.