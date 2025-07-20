// This class manages the input from serial connection
#include <Arduino.h>

#define SERIAL_BAUD_RATE 115200

struct SerialCommand {
    char cmd;
    double value;
};

class SerialManager {
  public:
    SerialManager();
    void init();
    void updateSerialInput(); // Reads a character from the serial port
    SerialCommand readSerialCommand(); // Returns a command if there is an available command that has been read
  private:

    char cmd = ' ';
    char numbers[11] = "aaaaaaaaaa";
    int numCounter = 0;

    double inputNumber = 0;
    bool commandRead = true;
};