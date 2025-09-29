#include "../include/StatusLight.h"

void StatusLight::init() {
    // set the pinmode for onboard LED lights
    pinMode(STAT_1_LED_PIN, OUTPUT);
    pinMode(STAT_2_LED_PIN, OUTPUT);

    // Start timer 1 on esp32
    // prescaler is set to the same as the cpu freq so the new freq should be 1Mhz
    m_timer = timerBegin(0, getCpuFrequencyMhz(), true);
    timerAttachInterrupt(m_timer, &StatusLight::onTimer, true);
    timerAlarmWrite(m_timer, 250000, true);
    timerAlarmEnable(m_timer);

    analogWrite(STAT_1_LED_PIN, LOW);
    analogWrite(STAT_2_LED_PIN, HIGH);
    m_stat1 = false;
    m_stat2 = true;

};

void StatusLight::toggleStat1() {
    if (m_stat1) {
        analogWrite(STAT_1_LED_PIN, 0);
        m_stat1 = false;
    } else {
        analogWrite(STAT_1_LED_PIN, 50);
        m_stat1 = true;
    }
}
void StatusLight::toggleStat2() {
    if (m_stat2) {
        analogWrite(STAT_2_LED_PIN, 0);
        m_stat2 = false;
    } else {
        analogWrite(STAT_2_LED_PIN, 50);
        m_stat2 = true;
    }
}

void IRAM_ATTR StatusLight::onTimer() {
    RotationDir dir = StatusLight::getInstance().getRotationDir();
    if (dir == RotationDir::CW) {
        // digitalWrite(STAT_1_LED_PIN, !digitalRead(STAT_1_LED_PIN));
        StatusLight::getInstance().toggleStat1();
    } else if (dir == RotationDir::CCW) {
        // digitalWrite(STAT_2_LED_PIN, !digitalRead(STAT_2_LED_PIN));
        StatusLight::getInstance().toggleStat2();
    } else {
        // IDLE
        // digitalWrite(STAT_1_LED_PIN, !digitalRead(STAT_1_LED_PIN));
        // digitalWrite(STAT_2_LED_PIN, !digitalRead(STAT_2_LED_PIN));
        StatusLight::getInstance().toggleStat1();
        StatusLight::getInstance().toggleStat2();
    }
}

void StatusLight::updateStatus(RotationDir dir, double percentOutput) {
    m_percentOutput = percentOutput;
    m_dir = dir;

    if (m_dir == RotationDir::IDLE) {
        timerAlarmWrite(m_timer, 250000, true);
        analogWrite(STAT_1_LED_PIN, LOW);
        analogWrite(STAT_2_LED_PIN, HIGH);
        m_stat1 = false;
        m_stat2 = true;
    } else {
        timerAlarmWrite(m_timer, mapf(fabs(percentOutput), 0, 100, 150000, 25000), true);
        analogWrite(STAT_1_LED_PIN, LOW);
        analogWrite(STAT_2_LED_PIN, LOW);
        m_stat1 = false;
        m_stat2 = false;
    }
};
