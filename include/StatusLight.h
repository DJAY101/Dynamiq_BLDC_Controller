#pragma once

#include <Arduino.h>
#include "MotorEnums.h"

// Status LED's
constexpr u_int8_t STAT_1_LED_PIN {5};
constexpr u_int8_t STAT_2_LED_PIN {6};

class StatusLight {
  public:
    static StatusLight& getInstance() {
        static StatusLight instance;  // created on first call, destroyed at program end
        return instance;
    }
    void init();
    void updateStatus(RotationDir dir, double percentOutput);
    
    
    // Delete copy/move so no duplicates can exist
    StatusLight(const StatusLight&) = delete;
    StatusLight& operator=(const StatusLight&) = delete;
    StatusLight(StatusLight&&) = delete;
    StatusLight& operator=(StatusLight&&) = delete;

    double getPercentOutput() const {return m_percentOutput; }
    RotationDir getRotationDir() const {return m_dir; }

    void toggleStat1();
    void toggleStat2();

  private:
    double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };

    StatusLight() {}   // private constructor
    ~StatusLight() {}  // private destructor

    hw_timer_t *m_timer{nullptr};
    static void IRAM_ATTR onTimer();

    double m_percentOutput{0};
    RotationDir m_dir {RotationDir::IDLE};

    bool m_stat1{false}; // Current status of light 1
    bool m_stat2{false}; // Current status of light 2

};