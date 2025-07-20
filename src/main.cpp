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

// Include the library for the AS5047P sensor.
#include <AS5047P.h>

// Pin definition
#define DRV_EN_PIN 38
// Low side control pins
#define W_LS_PIN 10
#define V_LS_PIN 12
#define U_LS_PIN 14

// High side control pins
#define U_HS_PIN 21
#define V_HS_PIN 13
#define W_HS_PIN 11

// Current sensing input
#define U_CUR_PIN 7
#define V_CUR_PIN 8
#define W_CUR_PIN 9

// Encoder SPI pins
#define ENCODER_CLK 39
#define ENCODER_MISO 40
#define ENCODER_MOSI 41
#define ENCODER_CSN 42

#define POT_PIN 18

// Lipo voltage
#define LIPO_V_PIN 15
// Lipo Error LED
#define LIPO_LED_PIN 4
// Status LED's
#define STAT_1_LED_PIN 5
#define STAT_2_LED_PIN 6

// Other Constants
#define CPU_FREQ_MHZ 80

#define PWM_FREQ 16384
#define PWM_RES 11

// define the spi bus speed for mag encoder
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000 // 1Mhz

#define LIPO_DISABLE_VOLTAGE 14.0

// Math Constants
#define M_PI 3.14159265359

// initialize a new AS5047P sensor object.
AS5047P magEncoder(ENCODER_CSN, AS5047P_CUSTOM_SPI_BUS_SPEED);

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Map double values
double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Sets up the board pins to the correct INPUT / OUTPUT
void setupDriverPins() {
  // Driver enable pin
  pinMode(DRV_EN_PIN, OUTPUT);

  // LED Pins
  pinMode(LIPO_LED_PIN, OUTPUT);
  pinMode(STAT_1_LED_PIN, OUTPUT);
  pinMode(STAT_2_LED_PIN, OUTPUT);

  // Phase enable pins
  pinMode(U_LS_PIN, OUTPUT);
  pinMode(V_LS_PIN, OUTPUT);
  pinMode(W_LS_PIN, OUTPUT);

  // Phase current sensing pins
  pinMode(U_CUR_PIN, INPUT);
  pinMode(V_CUR_PIN, INPUT);
  pinMode(W_CUR_PIN, INPUT);

  // Lipo voltage sensing pin
  pinMode(LIPO_V_PIN, INPUT);

  // Potentiometer pin
  pinMode(POT_PIN, INPUT);
}

// enables or disables gate driver
void setDriverEnable(bool enable) {
  if (enable) {
    // Enable Driver
    digitalWrite(DRV_EN_PIN, HIGH);
    // Enable Phases
    digitalWrite(U_LS_PIN, HIGH);
    digitalWrite(V_LS_PIN, HIGH);
    digitalWrite(W_LS_PIN, HIGH);
  } else {
    // disable Driver
    digitalWrite(DRV_EN_PIN, LOW);
    // disable Phases
    digitalWrite(U_LS_PIN, LOW);
    digitalWrite(V_LS_PIN, LOW);
    digitalWrite(W_LS_PIN, LOW);
  }
}

// double relativeAngle = 0.0;
// double previousRelativeAngle = 0.0;
// void updateRelativeAngle() {
//   double currentAngle = magEncoder.readAngleDegree();
//   double deltaAngle = (magEncoder.readAngleDegree() - previousRelativeAngle);
//   if (fabs(deltaAngle) < 3.0 && fabs(deltaAngle) > 0.1) {
//     relativeAngle += deltaAngle;

//   }
//   previousRelativeAngle = currentAngle;
// }

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


double testVal = 0;

char cmd = ' ';
char numbers[] = "aaaaaaaaaa";
int numCounter = 0;



void serialReadUpdate() {
  if (Serial.available() > 0) {
    // read the incoming byte:

    if (cmd == ' ') {
      // Read the command letter
      cmd = Serial.read();
    } else {
      // Read the number assocciated to it
      char readData = Serial.read();

      if (readData != '\n') {
        // If its not the end character then save it to the numbers array
        Serial.println(numCounter);
        numbers[numCounter++] = readData;
      } else if (numCounter + 1 < (sizeof(numbers) / sizeof(numbers[0]))) {
        
        // Otherwise if the end character is detected fill the rest of the array with .0
        bool firstLoop = true;
        for (int i = numCounter; i < (sizeof(numbers) / sizeof(numbers[0])); i++) {
          if (firstLoop) {
            firstLoop = false;
            numbers[i] = '.';
          } else {
            numbers[i] = '0';
          }
        }

        double userNumber = atof(numbers);

        // Print out command recieved
        Serial.print("Command: ");
        Serial.print(cmd);
        Serial.print(" Set to: ");
        Serial.println(userNumber, 5);
              auto errorInfo = AS5047P_Types::ERROR_t(); 
              auto settings = magEncoder.read_SETTINGS1();
              auto encSettings1 = magEncoder.read_SETTINGS1();
              encSettings1.data.values.DIR = 1;
        // Process the command here
        switch (cmd) {
          case 'b':
              Serial.print("Battery at ");
              Serial.println(getLipoVoltage());
              break;
          case 'e':
              Serial.println("Driver Enabled");
              setDriverEnable(true);
              break;
          case 'd':
              Serial.println("Driver Disabled");
              setDriverEnable(false);
              break;
          case 't':
              Serial.print("Set torque to: ");
              Serial.println(userNumber);
              torque = userNumber;
              break;
          case 'p':
              Serial.print("Position set to: ");
              Serial.println(userNumber, 5);
              // targetTheta = userNumber * 40 / 360 * M_PI;
              targetTheta = userNumber;
              break;
          case 'a':
              
              k_P = userNumber;
              break;
          default: // optional
              Serial.println("No matching cmd");
        }
        
        // Clear command
        cmd = ' ';
      }
    }

  } else {
    // reset the number array to be a default value
    memset(numbers, 'a', sizeof(numbers));
    numCounter = 0;
  }
}



void mainLoop() {

  // Update status led depending on rotation direction
  updateLED(theta);

  // Get user commands via serial
  serialReadUpdate();

  double output = (targetTheta - magEncoder.readAngleDegree()) * k_P + k_F * getSign(targetTheta - magEncoder.readAngleDegree());
  // double output = (targetTheta - theta) * k_P;

  Serial.print("Enc Degree: ");
  Serial.print(magEncoder.readAngleDegree(), 5);

  targetDeltaTheta = clamp(output, -1.57, 1.57);

  // ramping deltaTheta (ramping how fast the velocity changes)
  deltaTheta += getSign(targetDeltaTheta - deltaTheta) * 0.03;

  if (fabs(targetDeltaTheta - deltaTheta) < 0.6) { deltaTheta = targetDeltaTheta; }

  // First update theta to the actual theta depending on the mag encoder rotation

  theta = (magEncoder.readAngleDegree() + ENCODER_AND_MOTOR_OFFSET) / (360.0 / 40.0) * M_PI;
  Serial.print(" Elec Rad: ");
  Serial.print(theta, 5);

  Serial.print(" DelTheta: ");
  Serial.println(deltaTheta);
  // Add a delta theta so it rotates
  // theta += deltaTheta;
  theta += deltaTheta;


  // Find the trig ratio for the output between 0 - 100%
  double U_duty = sin(theta) * 50.0 + 50.0;
  double V_duty = sin(theta + 120.0 * M_PI / 180.0) * 50.0 + 50.0;
  double W_duty = sin(theta + 240.0 * M_PI / 180.0) * 50.0 + 50.0;

  // estimating a linear torque curve
  torque = clamp(torqueDeltaThetaRatio * fabs(deltaTheta) + 0.09, 0, 0.2);
  // torque = 0.08;

  // set Phase U
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (float)U_duty * torque);
  // set phase V
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (float)V_duty * torque);
  // set phase W
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, (float)W_duty * torque);
  


}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Set CPU Freq
  setCpuFrequencyMhz(CPU_FREQ_MHZ);
  
  // Setup Driver Pins
  setupDriverPins();

  // Start serial communication
  Serial.begin(115200);

  // Get the ESP32 to set the correct pins for encoder SPI (using its built in MUX)
  SPI.begin(ENCODER_CLK, ENCODER_MISO, ENCODER_MOSI, ENCODER_CSN);

  // Initialise the magnetic encoder
  while (!magEncoder.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }

  // setup GPIO output for mcpwm
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, U_HS_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, V_HS_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, W_HS_PIN);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 19000;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
  pwm_config.cmpr_a = 50;    //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 50;    //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
  
  mcpwm_sync_config_t sync_config;
  sync_config.sync_sig = MCPWM_SELECT_TIMER0_SYNC; // Gets the sync signal from timer 0
  sync_config.timer_val = 999; // The phase offset between the timers (999 makes them most in sync)
  sync_config.count_direction = MCPWM_TIMER_DIRECTION_UP; // The counter should count up
  
  mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &sync_config);
  mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ); // Setup an sync signal from timer 0
}



bool lipoThresholdHit = false;
void loop() {
  // Lipo protection if statement
  if (getLipoVoltage() < LIPO_DISABLE_VOLTAGE || lipoThresholdHit) {
    lipoThresholdHit = true;
    digitalWrite(LIPO_LED_PIN, HIGH);
    setDriverEnable(false);
    Serial.println("Lipo Voltage Threshold hit");
  } else {
    mainLoop();
  }
}
