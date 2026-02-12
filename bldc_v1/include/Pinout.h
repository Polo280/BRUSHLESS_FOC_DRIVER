#ifndef PINOUT_H
#define PINOUT_H

// #define FOC_CONTROL
#define SERIAL_DEBUG
// #define SERIAL_DEBUG_CURRENTS
// #define CSV_FORMAT_CURRENTS

#define BACKWARDS

// #define MEASURE_TICKS_PER_REV

// #define TICKS_PER_REV (float)281  //< Motor Hub
// #define TICKS_PER_REV (float)134 //< Motor Directo
// #define TICKS_PER_REV (float) 94 //< Motor Directo relacion 3

#define TICKS_PER_REV (float)31 //< Motor Directo Raw

#define FILTER_SIZE 5  
//#define RPM

#include <cstdint>

// HALL Sensors
constexpr uint8_t kHallAPin{12}; // Pin for Hall sensor A
constexpr uint8_t kHallCPin{21}; // Pin for Hall sensor B
constexpr uint8_t kHallBPin{10}; // Pin for Hall sensor C

// Hall Parameters
constexpr uint8_t kHallOverSample{8};

// DRIVER GATES (PWM Outputs)
constexpr uint8_t kGateAH{4};  // High-side gate for phase A
constexpr uint8_t kGateAL{33}; // Low-side gate for phase A
constexpr uint8_t kGateBH{6};  // High-side gate for phase B
constexpr uint8_t kGateBL{9};  // Low-side gate for phase B
constexpr uint8_t kGateCH{36}; // High-side gate for phase C
constexpr uint8_t kGateCL{37}; // Low-side gate for phase C

// DRIVERS GATES PARAMS
constexpr uint32_t kPwmFrequency{30000}; // 30 kHz PWM frequency

// Other Pins
constexpr uint8_t kBuiltInLedPin{13}; // Built-in LED pin
constexpr uint8_t kIrqFlagPin{30};    // Irq flag pin

// ADC Inputs
constexpr uint8_t kThrottlePin{A10};   // Throttle input pin  ADC1 IN1
constexpr uint8_t kVBatSensePin{A17};  // Battery voltage bus ADC1 IN10
constexpr uint8_t kCurrentSenseA{A12}; // Current sense for phase A
constexpr uint8_t kCurrentSenseB{A13}; // Current sense for phase B
constexpr uint8_t kCurrentSenseC{A14}; // Current sense for phase C

// ADC Resolution & Sampling
constexpr uint8_t kAnalogResolution{12}; // 12-bit ADC
constexpr uint8_t kAnalogAveraging{8};   // 8x oversampling

// Throttle Configuration
constexpr uint16_t kThrottleLow{550};     // Minimum throttle value
constexpr uint16_t kThrottleHigh{2900};  // Maximum throttle value
constexpr uint16_t kThrottleResolution{4095}; // Throttle resolution (12-bit)

// COMMUNICATION 
constexpr uint8_t kUART_RX{0};
constexpr uint8_t kUART_TX{1};

// Battery voltage divider for sensing
constexpr uint32_t kResistorVal1{82000};
constexpr uint32_t kResistorVal2{4700};

// Motor profiles 
// #define HUB_MOTOR
#define KOFORD_MOTOR

#if defined(HUB_MOTOR) && defined(KOFORD_MOTOR)
    #error "Define only one motor"
#elif defined(HUB_MOTOR)
    constexpr uint8_t kBatteryVoltage = 48;
#elif defined(KOFORD_MOTOR)
    constexpr uint8_t kBatteryVoltage = 24;
#else
    #error "No motor type defined"
#endif


//////////////////////////////////////////////////
//////////////// DEBUG MACROS ///////////////////
//////////////////////////////////////////////////

#ifdef SERIAL_DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#endif // PINOUT_H













