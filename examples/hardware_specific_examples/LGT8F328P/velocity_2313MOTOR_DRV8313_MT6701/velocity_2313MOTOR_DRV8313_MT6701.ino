/**
 *
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 *
 *
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
 */
#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"


#define SENSOR1_CS 5 // some digital pin that you're using as the nCS pin
#define NFT_PIN 2     // driver nFT, active low
#define NRT_PIN 3     // driver nRT, active low reset
#define STARTUP_TONE_VOLTAGE 0.35f
MagneticSensorMT6701SSI sensor(SENSOR1_CS);

// magnetic sensor instance - SPI
// magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// velocity set point variable
float target_velocity = 100;

void resetDriver() {
  digitalWrite(NRT_PIN, LOW);
  delayMicroseconds(20);
  digitalWrite(NRT_PIN, HIGH);
}

void monitorDriverFault() {
  static bool fault_latched = false;
  const bool fault = digitalRead(NFT_PIN) == LOW;

  if (fault && !fault_latched) {
    fault_latched = true;
    resetDriver();
  } else if (!fault) {
    fault_latched = false;
  }
}

void playMotorTone(const uint16_t frequency, const uint16_t duration_ms) {
  const uint16_t half_period_us = 500000UL / frequency;
  const uint32_t cycles = (uint32_t)frequency * duration_ms / 1000UL;
  const float electrical_angle = motor.electricalAngle();

  for (uint32_t i = 0; i < cycles; i++) {
    motor.setPhaseVoltage(STARTUP_TONE_VOLTAGE, 0, electrical_angle);
    delayMicroseconds(half_period_us);
    motor.setPhaseVoltage(-STARTUP_TONE_VOLTAGE, 0, electrical_angle);
    delayMicroseconds(half_period_us);
    monitorDriverFault();
  }

  motor.setPhaseVoltage(0, 0, electrical_angle);
  delay(40);
}

void playStartupMelody() {
  playMotorTone(262, 220); // Do, C4
  playMotorTone(294, 220); // Re, D4
  playMotorTone(330, 280); // Mi, E4
}

// Lightweight serial target parser. This avoids linking Commander and its
// generic command/monitoring code, saving program flash on ATmega328-class MCUs.
void readTargetVelocity() {
  static float value = 0.0f;
  static float fraction = 0.1f;
  static bool negative = false;
  static bool afterDecimal = false;
  static bool receiving = false;

  while (Serial.available()) {
    const char c = Serial.read();

    if (c == 'T' || c == 't') {
      value = 0.0f;
      fraction = 0.1f;
      negative = false;
      afterDecimal = false;
      receiving = true;
    } else if (c == '-' && receiving) {
      negative = true;
    } else if (c == '.' && receiving) {
      afterDecimal = true;
    } else if (c >= '0' && c <= '9') {
      if (!receiving) {
        value = 0.0f;
        fraction = 0.1f;
        negative = false;
        afterDecimal = false;
        receiving = true;
      }
      if (afterDecimal) {
        value += (c - '0') * fraction;
        fraction *= 0.1f;
      } else {
        value = value * 10.0f + (c - '0');
      }
    } else if ((c == '\n' || c == '\r' || c == ' ') && receiving) {
      target_velocity = negative ? -value : value;
      receiving = false;
    }
  }
}

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  // nFT and nT are both active low. Keep the driver out of reset normally.
  pinMode(NFT_PIN, INPUT_PULLUP);
  digitalWrite(NRT_PIN, HIGH);
  pinMode(NRT_PIN, OUTPUT);
  resetDriver();

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 1.5;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println(F("foc init fail."));
  } else {
    playStartupMelody();
  }

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  monitorDriverFault();
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  readTargetVelocity();
}