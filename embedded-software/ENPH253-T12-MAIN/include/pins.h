#pragma once

// pin definitions EVERY PIN
// -------------------- Oscillator --------------------------
// Oscillator
constexpr int PIN_GPIO_OSCILLATOR{1};

//  -------------------- ARM PINS -----------------
// claw servo pins
constexpr int PIN_PWM_CLAW{4};

// arm servo pins
constexpr int PIN_PWM_ALPHA{2};
constexpr int PIN_PWM_BETA{3};

// Lazy susan stepper/gamma pins
constexpr int PIN_GPIO_STEP{5};
constexpr int PIN_GPIO_DIR{6};
constexpr int PIN_GPIO_ENA{100}; // THIS DOES NOT EXIST WHAT FUCKKKKK

// Ultrasonic sensor Pins
constexpr int PIN_GPIO_ECHO{41};
constexpr int PIN_GPIO_TRIG{42};

//  -------------------- DRIVETRAIN  -----------------
// Rotary Encoders
constexpr int PIN_GPIO_LMD_A{8};
constexpr int PIN_GPIO_LMD_B{7};
constexpr int PIN_GPIO_RMD_A{9};
constexpr int PIN_GPIO_RMD_B{10};

// drive train PWMS
constexpr int PIN_PWM_LMD_0{12};
constexpr int PIN_PWM_LMD_1{11};
constexpr int PIN_PWM_RMD_0{14};
constexpr int PIN_PWM_RMD_1{13};
// if wheel inverted, turn to true
constexpr bool LEFT_WHEEL_INVERTED{true};
constexpr bool RIGHT_WHEEL_INVERTED{true};

// Tapee Sensors
constexpr int PIN_GPIO_TAPE_1{38};
constexpr int PIN_GPIO_TAPE_2{39};
constexpr int PIN_GPIO_TAPE_3{40};
constexpr int PIN_GPIO_TAPE_4{21};
constexpr int PIN_GPIO_TAPE_5{18};

// -------------------- IMU ---------------------------------
// IMU
constexpr int PIN_I2C_IMU_SDA{47};
constexpr int PIN_I2C_IMU_SCL{48};

// -------------------- BEACON ------------------------------
// Beacon
constexpr int PIN_ADC_BEACON{15};

// -------------------- K230 --------------------------------
// K230
constexpr int PIN_UART_K230_RX{16};
constexpr int PIN_UART_K230_TX{17};
