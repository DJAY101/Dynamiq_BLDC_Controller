#include <Arduino.h>

#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

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



class MotorController {
  public:

    MotorController();
    void init(); // Sets up the board pins to the correct INPUT / OUTPUT
    void setDriverEnable(bool enable);

  private: 
    void setPhasePercentOutput(double uDutyPercent, double vDutyPercent, double wDutyPercent);
  

};