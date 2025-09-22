#include "../include/MotorController.h"

MotorController::MotorController() {

}

void MotorController::init() {
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

    // setup GPIO output for mcpwm
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, U_HS_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, V_HS_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, W_HS_PIN);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 19000;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
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


void MotorController::setDriverEnable(bool enable) {
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

void MotorController::setPhasePercentOutput(double uDutyPercent, double vDutyPercent, double wDutyPercent) {
  const double SAFETY_MAX_DUTY = 31;
  // Driver protection, if it exceeds this duty cycle the driver doesnt like it and faults :(
  if (uDutyPercent >= SAFETY_MAX_DUTY || vDutyPercent >= SAFETY_MAX_DUTY || wDutyPercent >= SAFETY_MAX_DUTY) {
    Serial.println("MAX PHASE DUTY EXCEEDED");
    return;
  }

  // set Phase U
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, uDutyPercent);
  // set phase V
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, vDutyPercent);
  // set phase W
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, wDutyPercent);
}

void MotorController::openLoopPercentageOutput(double percentOutput) {
  if (percentOutput == 0) {
    setIdle();
    return;
  }
  m_currentControlMode = ControlMode::OPEN_LOOP_PERCENT_OUTPUT;
  m_percentageOutput = percentOutput;
}

void MotorController::closedLoopPercentageOutput(double percentOutput) {
  if (percentOutput == 0) {
    setIdle();
    return;
  }
  m_currentControlMode = ControlMode::CLOSED_LOOP_PERCENT_OUTPUT;
  m_percentageOutput = percentOutput;
}

void MotorController::setClosedLoopPosition(double position) {
  m_currentControlMode = ControlMode::CLOSED_LOOP_POSITION;
  m_targetPosition = position;
}


void MotorController::setIdle() {
  setDriverEnable(false);
  setPhasePercentOutput(0, 0, 0);
  m_currentControlMode = ControlMode::IDLE;
  m_percentageOutput = 0.0;
  m_electricalThetaStep = 0.0;
}

void MotorController::setOpenLoopPosition(double position, bool physicalShaftPos) {
  m_currentControlMode = ControlMode::OPEN_LOOP_ELECTRICAL_POSITION;
  if (physicalShaftPos) {
    m_targetElectricalPosition = position * (M_PI / 180.0) * (MAGNETIC_POLE_COUNTS / 2.0);
    return;
  }
  m_targetElectricalPosition = position;
}


// open loop svpwmCommutation
void MotorController::svpwmCommutation() {

  m_percentageOutput = clamp(m_percentageOutput, -100.0, 100.0);

  // Ramping so that the velocity doesnt change instantly causing high current spikes
  double targetElectricalThetaStep = mapf(m_percentageOutput, -100, 100, -0.1, 0.1);
  m_electricalThetaStep += (targetElectricalThetaStep - m_electricalThetaStep) * deltaElectricalThetaStep_KP;

  if (fabs(targetElectricalThetaStep - m_electricalThetaStep) < 0.01) {
    m_electricalThetaStep = targetElectricalThetaStep;
  }

  // sets the target electrical theta with a step added
  double targetElectricalTheta = m_electricalTheta + m_electricalThetaStep;

  // Find the trig ratio for the output between 0 - 100%
  double U_duty = sin(targetElectricalTheta) * 50.0 + 50.0;
  double V_duty = sin(targetElectricalTheta + 120.0 * M_PI / 180.0) * 50.0 + 50.0;
  double W_duty = sin(targetElectricalTheta + 240.0 * M_PI / 180.0) * 50.0 + 50.0;

  double MAX_TORQUE = 0.07 + 1.5 * fabs(m_electricalThetaStep); // limit the maximum duty percentage of the PWM (assumes linear relationship to keep the motor turning at high speed)

  // Set the phase with the output above
  setPhasePercentOutput(U_duty * MAX_TORQUE, V_duty * MAX_TORQUE, W_duty * MAX_TORQUE);

  // assumes the electrical theta is now at the target (hence open loop control)
  m_electricalTheta = targetElectricalTheta;
  
  Serial.print(">CA:");
  Serial.println(fmod((m_encoderAngle * MAGNETIC_POLE_COUNTS / 2.0) - 90, 360), 4);

  Serial.print(">EA:");
  Serial.println(fmod(m_electricalTheta * 180.0 / M_PI, 360.0), 4);
}


// Closed loop encoder based electrical theta commutation
void MotorController::svpwmEncoderCommutation() {

  // clamp the percentage output between -100 and 100
  m_percentageOutput = clamp(m_percentageOutput, -100.0, 100.0);
  

  // Ramping so that the torque doesnt change instantly causing high current spikes
  double targetTorque = mapf(m_percentageOutput, -100, 100, -0.20, 0.20);
  
  m_torque += (targetTorque - m_torque) * 0.003;
  if (fabs(targetTorque - m_torque) < 0.01) {
    m_torque = targetTorque;
  }

  // Set the electrical theta to the actual position taken from the encoder (closed loopiness comes in)
  // const double angleOffset = 130.0; // for motor with 130kv*
  const double angleOffset = -90.0; // for motor with 90kV*
  m_electricalTheta = fmod((m_encoderAngle * MAGNETIC_POLE_COUNTS / 2.0) + angleOffset, 360) * M_PI / 180.0; 

  // The theta step should be 90 degrees from the magnet to ensure maximum torque and the direction depends on the torque sign
  m_electricalThetaStep = (m_torque >= 0) ? M_PI/2 : -M_PI/2;

  // sets the target electrical theta with the 90 degrees offset
  double targetElectricalTheta = m_electricalTheta + m_electricalThetaStep;

  // Find the trig ratio for the output between 0 - 100%
  double U_duty = sin(targetElectricalTheta) * 50.0 + 50.0;
  double V_duty = sin(targetElectricalTheta + 120.0 * M_PI / 180.0) * 50.0 + 50.0;
  double W_duty = sin(targetElectricalTheta + 240.0 * M_PI / 180.0) * 50.0 + 50.0;

  double absoluteTorque = fabs(m_torque) + 0.02;

  // Set the phase with the output above
  setPhasePercentOutput(U_duty * absoluteTorque, V_duty * absoluteTorque, W_duty * absoluteTorque);
}



void MotorController::update(double encAngle) {
  m_encoderAngle = encAngle;

  // Serial.print(">Encoder Angle:");
  // Serial.println(m_encoderAngle, 4);

  if (m_currentControlMode == ControlMode::IDLE) return; // Do nothing if its in the idle state

  // Utilise svpwmCommutation if its open loop control
  if (m_currentControlMode == ControlMode::OPEN_LOOP_PERCENT_OUTPUT) {
    svpwmCommutation();
  }

  if (m_currentControlMode == ControlMode::CLOSED_LOOP_PERCENT_OUTPUT) {
    svpwmEncoderCommutation();
  }

  if (m_currentControlMode == ControlMode::OPEN_LOOP_ELECTRICAL_POSITION) {
    m_percentageOutput = (m_targetElectricalPosition - m_electricalTheta) * 0.9;
    svpwmCommutation();
  }

  if (m_currentControlMode == ControlMode::CLOSED_LOOP_POSITION) {
    double error = (m_targetPosition - m_encoderAngle);
    m_percentageOutput = getSign(error) + error * 0.6;
    if (fabs(error) > 2) {
      Serial.print(">Angle:");
      Serial.println(m_encoderAngle);
    }

    svpwmEncoderCommutation();
  }

}