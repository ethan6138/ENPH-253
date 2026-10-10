#include "AccelStepper.h"
#include <Arduino.h>
#include <FastAccelStepper.h>

#define ENA_PIN 1000
#define DIR_PIN 6
#define STEP_PIN 5

// // manual stepping
//  void setup() {

//   Serial.begin(115200);

//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(STEP_PIN, OUTPUT);

//   digitalWrite(DIR_PIN, HIGH);
// }

// void loop() {
//   for (int i = 0; i < 201; i++) {
//     digitalWrite(STEP_PIN, HIGH);
//     delay(10);
//     digitalWrite(STEP_PIN, LOW);
//     delay(10);
//   }
// }

// accelstepper at max rated speed 1000 steps
// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// void setup() {
//   Serial.begin(115200);

//   stepper.setMaxSpeed(1000);
//   stepper.setAcceleration(200);
//   stepper.setSpeed(1000);
//   stepper.setCurrentPosition(0);
// }

// void loop() { stepper.runSpeed(); }

//---------------------------------------------------------------

// // fast accel stepper
// FastAccelStepperEngine engine{FastAccelStepperEngine()};
// FastAccelStepper *fastStepper = NULL;

// void setup() {
//   engine.init();
//   fastStepper = engine.stepperConnectToPin(STEP_PIN);
//   if (fastStepper) {
//     fastStepper->setDirectionPin(DIR_PIN);
//     fastStepper->setAutoEnable(true);

//     fastStepper->setSpeedInHz(200);
//     fastStepper->setAcceleration(50);
//   }
// }

// void loop() {
//   if (fastStepper->isRunning() == true)
//     fastStepper->moveTo(2000);
// }

// copied code for stepper
FastAccelStepperEngine engine{FastAccelStepperEngine()};
FastAccelStepper *fastStepper = NULL;

constexpr int motorStepsPerRev{200};   // QSH4218 defualt no m1-3
constexpr int gearRatioNumerator{1};   // turret gear
constexpr int gearRatioDenominator{1}; // motor gear
constexpr float stepsPerDegree =
    (motorStepsPerRev * static_cast<float>(gearRatioNumerator) /
     gearRatioDenominator) /
    360.0f;
constexpr bool gammaInverted{true};

void setup() {
  engine.init();
  fastStepper = engine.stepperConnectToPin(STEP_PIN);

  if (fastStepper) {
    fastStepper->setDirectionPin(DIR_PIN);
    fastStepper->setEnablePin(ENA_PIN);
    fastStepper->setAutoEnable(true);

    fastStepper->setSpeedInHz(400);
    fastStepper->setAcceleration(1000);
    fastStepper->setCurrentPosition(0);
    fastStepper->runForward(); 
  }

  // delay(5000);
  // int gamma{150};

  // if (fastStepper) {
  //   int32_t rawTargetSteps = static_cast<int32_t>(gamma * stepsPerDegree);
  //   int32_t targetSteps = gammaInverted ? -rawTargetSteps : rawTargetSteps;
  //   fastStepper->moveTo(targetSteps); // absolute angle to 0 that we set
  // }


}

void loop() {}