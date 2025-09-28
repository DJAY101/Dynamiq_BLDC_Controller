//////// Written by David J ////////
/////// ESP-32-S3 Wroom 1 Based BLDC Controller /////

/*
    The following program runs a simple brushless motor using sinusoidal control
    Copyright (C) 2025 Dawei Jin
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#include <stdlib.h>

#include "math.h"

#include "esp32-hal-ledc.h"
#include "esp32-hal-cpu.h"
#include "esp_attr.h"

// include custom classes
#include "../include/MotorController.h"
#include "../include/SerialManager.h"

// Include the library for the AS5047P sensor.
#include <AS5047P.h>

// ESP32 Freq
constexpr u_int32_t CPU_FREQ_MHZ {240};

// Mag Encoder SPI Bus Speed
constexpr u_int32_t AS5047P_CUSTOM_SPI_BUS_SPEED {9000000}; // 9Mhz

// The voltage at which the controller disables itself
constexpr double LIPO_DISABLE_VOLTAGE {14.0};

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
  constexpr double LIPO_OFFSET = 0.7;
  return mapf(analogRead(LIPO_V_PIN), 0, 4095, 0, 36.3) + LIPO_OFFSET;
}

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
        m_motorController->setClosedLoopPosition(userCommand.value);
        break;
    case 't':
         *m_motorController->getTestValue() = userCommand.value;
        break;
    case 'o':
        Serial.println("Percentage Output Mode Enabled");
        m_motorController->openLoopPercentageOutput(userCommand.value);
        break;
    case 'l':
        Serial.println("Percentage Output Mode Enabled");
        m_motorController->closedLoopPercentageOutput(userCommand.value);
        break;
    case 'z':
        Serial.println("Serial Input Disabled...");
        disableSerialInput = true;
        break;
    default: // optional
        Serial.println("No matching cmd");
  }
}

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
  delay(300);
}


bool lipoThresholdHit = false;
void loop() {


  double encoderAngle = magEncoder.readAngleDegree();
  
  m_motorController->update(encoderAngle, magEncoder.readMagnitude());
  

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



