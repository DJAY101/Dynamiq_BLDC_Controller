#pragma once

#include <Arduino.h>

#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "StatusLight.h"

// Pin definition
constexpr u_int8_t DRV_EN_PIN {38};

// Low side control pins
constexpr u_int8_t W_LS_PIN {10};
constexpr u_int8_t V_LS_PIN {12};
constexpr u_int8_t U_LS_PIN {14};

// High side control pins
constexpr u_int8_t U_HS_PIN {21};
constexpr u_int8_t V_HS_PIN {13};
constexpr u_int8_t W_HS_PIN {11};

// Current sensing input
constexpr u_int8_t U_CUR_PIN {7};
constexpr u_int8_t V_CUR_PIN {8};
constexpr u_int8_t W_CUR_PIN {9};

// Current sensing constants
constexpr double SHUNT_RESISTANCE {0.1};
constexpr double CURRENT_AMP {5.0};

// Lipo voltage
constexpr u_int8_t LIPO_V_PIN {15};
// Lipo Error LED
constexpr u_int8_t LIPO_LED_PIN {4};

// Motor Magnet Pole Count
constexpr u_int32_t MAGNETIC_POLE_COUNTS {40};



  enum ControlMode {
    IDLE,
    OPEN_LOOP_PERCENT_OUTPUT,
    OPEN_LOOP_ELECTRICAL_POSITION,
    CLOSED_LOOP_PERCENT_OUTPUT,
    CLOSED_LOOP_POSITION
  };


class MotorController {
  public:

    MotorController();
    void init(); // Sets up the board pins to the correct INPUT / OUTPUT
    void update(double encAngle, double encMag); // this needs to be in the main loop for the motor to update and spin
    void setDriverEnable(bool enable);
    void setIdle(); // puts the driver into idle mode
    void openLoopPercentageOutput(double percentOutput); // uses SVPWM with no feedback loop from mag encoder
    void setOpenLoopPosition(double position, bool physicalShaftPos = false);
    void closedLoopPercentageOutput(double percentOutput); // Using encoder feedback to update the electrical theta
    void setClosedLoopPosition(double position);

    double* getTestValue() { return &testValue; }

    // returns the current values of the three phases in a array where [u, v, w] data is stored
    std::array<double, 3> getPhaseCurrents();

  private: 
    void setPhasePercentOutput(double uDutyPercent, double vDutyPercent, double wDutyPercent);
    void svpwmCommutation();
    void svpwmEncoderCommutation();
    RotationDir getOutputDirection() {return (m_percentageOutput >=0) ? RotationDir::CW : RotationDir::CCW; }

    double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };
    double clamp(double d, double min, double max) {
      const double t = d < min ? min : d;
      return t > max ? max : t;
    }
    double getSign(double num) {
      if (num == 0) return 0;
      if (num > 0) { return 1.0; } else { return -1.0; }
    }

    
    ControlMode m_currentControlMode = ControlMode::IDLE;

    double m_electricalTheta = 0.0; // degrees for electrical theta state not physical
    double m_electricalThetaStep = 0.0; // This depicts how much the theta steps to rotate the motor

    double const deltaElectricalThetaStep_KP = 0.001; // KP values used for the acceleration of the motor

    double m_percentageOutput = 0.0; 

    // Used by OPEN_LOOP_ELECTRICAL_POSITION control mode
    double m_targetElectricalPosition = 0.0;

    // Used by CLOSED_LOOP_POSITION control mode
    double m_targetPosition = 0.0;

    double m_encoderAngle = 0.0;

    double m_torque = 0.0;

    // test value that can be adjusted in real time by serial
    double testValue = -90;
    double m_previousEnc{0};
    unsigned long m_previousMillis{0};
};