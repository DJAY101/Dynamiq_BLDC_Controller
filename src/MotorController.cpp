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
  const double SAFETY_MAX_DUTY = 0.4;

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