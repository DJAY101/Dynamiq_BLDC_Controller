//////// Written by David J ////////
/////// ESP-32-S3 Wroom 1 Based BLDC Controller /////

#include <stdlib.h>

#include "math.h"

#include "esp32-hal-ledc.h"
#include "esp32-hal-cpu.h"
#include "esp_attr.h"

#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "../include/MotorController.h"
#include "../include/SerialManager.h"

// Include the library for the AS5047P sensor.
#include <AS5047P.h>

// Other Constants
#define CPU_FREQ_MHZ 160

#define PWM_FREQ 16384
#define PWM_RES 11

// define the spi bus speed for mag encoder
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000 // 1Mhz

#define LIPO_DISABLE_VOLTAGE 14.0


// initialize a new AS5047P sensor object.
AS5047P magEncoder(ENCODER_CSN, AS5047P_CUSTOM_SPI_BUS_SPEED);

// init my motor drivers and serial manager
MotorController* m_motorController = new MotorController();
SerialManager* m_serialManager = new SerialManager();

bool disableSerialInput = false;

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Map double values
double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Returns the current voltage of the lipo battery
double getLipoVoltage() {
  // Voltage offset of lipo due to voltage divider tolerance
  const double LIPO_OFFSET = 0.7;
  return mapf(analogRead(LIPO_V_PIN), 0, 4095, 0, 36.3) + LIPO_OFFSET;
}


// Update stat LED's depending on the direction the motor is turning
double previousTheta = 0;
void updateLED(double currentTheta) {
  if (currentTheta > previousTheta) {
    // if theta is increasing then turn on stat 1
    digitalWrite(STAT_1_LED_PIN, HIGH);
    digitalWrite(STAT_2_LED_PIN, LOW);
  } else if (currentTheta < previousTheta) {
    // if theta is decreasing then turn on stat 2
    digitalWrite(STAT_1_LED_PIN, LOW);
    digitalWrite(STAT_2_LED_PIN, HIGH);
  } else {
    // if theta is stationary then turn on both
    digitalWrite(STAT_1_LED_PIN, HIGH);
    digitalWrite(STAT_2_LED_PIN, HIGH);
  }
  previousTheta = currentTheta;
}

void printArray(char* arr, int size) {
  for (int i = 0; i < size; i++) {
    Serial.println(arr[i]);
  }
}

double getPotValueMapped(double a, double b) {
  return mapf(analogRead(POT_PIN), 0, 4095.0, a, b);
}

double getSign(double num) {
  if (num == 0) return 0;
  if (num > 0) { return 1.0; } else { return -1.0; }
}

double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}


double torque = 0.0;
double theta = 0;
double priorTheta = 0;
double deltaTheta = 0; // keep delta theta between -0.4 and 0.4 for now
double targetDeltaTheta = 0;
double targetTheta = 0;
double k_P = 0.02;
double k_F = 0.16;
double torqueDeltaThetaRatio = 0.10;

double ENCODER_AND_MOTOR_OFFSET = 6.2;



void executeSerialCommand(SerialManager* serialManager) {
   // Print out command recieved
  SerialCommand userCommand = serialManager->readSerialCommand();
  if (userCommand.cmd == ' ') return;

  Serial.print("Command: ");
  Serial.print(userCommand.cmd);
  Serial.print(" Set to: ");
  Serial.println(userCommand.value, 5);

  // Process the command here
  switch (userCommand.cmd) {
    case 'b':
        Serial.print("Battery at ");
        Serial.println(getLipoVoltage());
        break;
    case 'e':
        Serial.println("Driver Enabled");
        m_motorController->setDriverEnable(true);
        break;
    case 'd':
        Serial.println("Driver Disabled");
        m_motorController->setIdle();
        break;
    case 'p':
        Serial.print("Position set to: ");
        Serial.println(userCommand.value, 5);
        m_motorController->setOpenLoopPosition(userCommand.value, true);
        break;
    case 't':
         *m_motorController->getTestValue() = userCommand.value;
        break;
    case 'o':
        Serial.println("Percentage Output Mode Enabled");
        m_motorController->openLoopPercentageOutput(userCommand.value);
        break;
    case 'z':
        Serial.println("Serial Input Disabled...");
        disableSerialInput = true;
        break;
    default: // optional
        Serial.println("No matching cmd");
  }
}


// void mainLoop() {

//   // Update status led depending on rotation direction
//   updateLED(theta);

//   double output = (targetTheta - magEncoder.readAngleDegree()) * k_P + k_F * getSign(targetTheta - magEncoder.readAngleDegree());
//   // double output = (targetTheta - theta) * k_P;

//   // Serial.print("Enc Degree: ");
//   // Serial.print(magEncoder.readAngleDegree(), 5);

//   targetDeltaTheta = clamp(output, -1.57, 1.57);

//   // ramping deltaTheta (ramping how fast the velocity changes)
//   deltaTheta += getSign(targetDeltaTheta - deltaTheta) * 0.03;

//   if (fabs(targetDeltaTheta - deltaTheta) < 0.6) { deltaTheta = targetDeltaTheta; }

//   // First update theta to the actual theta depending on the mag encoder rotation

//   theta = (magEncoder.readAngleDegree() + ENCODER_AND_MOTOR_OFFSET) / (360.0 / 40.0) * M_PI;
//   // Serial.print(" Elec Rad: ");
//   // Serial.print(theta, 5);

//   // Serial.print(" DelTheta: ");
//   // Serial.println(deltaTheta);
//   // Add a delta theta so it rotates
//   // theta += deltaTheta;
//   theta += deltaTheta;


//   // Find the trig ratio for the output between 0 - 100%
//   double U_duty = sin(theta) * 50.0 + 50.0;
//   double V_duty = sin(theta + 120.0 * M_PI / 180.0) * 50.0 + 50.0;
//   double W_duty = sin(theta + 240.0 * M_PI / 180.0) * 50.0 + 50.0;

//   // estimating a linear torque curve
//   torque = clamp(torqueDeltaThetaRatio * fabs(deltaTheta) + 0.09, 0, 0.2);
//   // torque = 0.08;

//   // set Phase U
//   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (float)U_duty * torque);
//   // set phase V
//   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (float)V_duty * torque);
//   // set phase W
//   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, (float)W_duty * torque);

// }

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  // Set CPU Freq
  setCpuFrequencyMhz(CPU_FREQ_MHZ);
  
  // Setup Driver Pins for the motor controller
  m_motorController->init();

  // Start serial communication
  m_serialManager->init();

  // Get the ESP32 to set the correct pins for encoder SPI (using its built in MUX)
  SPI.begin(ENCODER_CLK, ENCODER_MISO, ENCODER_MOSI, ENCODER_CSN);

  // Initialise the magnetic encoder
  while (!magEncoder.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }

}



bool lipoThresholdHit = false;
void loop() {
  m_motorController->update();
  if (!disableSerialInput) {
    m_serialManager->updateSerialInput();
  }
  executeSerialCommand(m_serialManager);

  // Lipo protection if statement
  if (getLipoVoltage() < LIPO_DISABLE_VOLTAGE || lipoThresholdHit) {
    lipoThresholdHit = true;
    digitalWrite(LIPO_LED_PIN, HIGH);
    m_motorController->setDriverEnable(false);
    m_motorController->setIdle();
    Serial.println("Lipo Voltage Threshold hit");
  }
}
