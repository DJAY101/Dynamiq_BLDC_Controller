#include "../include/SerialManager.h"

SerialManager::SerialManager() {}

void SerialManager::init() {
      Serial.begin(115200);
}

void SerialManager::updateSerialInput() {
    if (!readyToRead) return;

    // For serial you have to read character by character
    // If there is no characters to read from Serial
    if (Serial.available() <= 0) {
        // reset the number array to be a default value
        return;
    }

    // If there is a character to read from the serial...

    if (cmd == ' ') {
      // Read the command letter
      cmd = Serial.read();
    } else {
      // Start processing the digits into the char array
      char readData = Serial.read();

      if (readData != '\n') {
        // If its not the end character then save it to the numbers array
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

        inputNumber = atof(numbers);

        readyToRead = false; // prevents the updater from updating until the command is read
      }
    }
}

SerialCommand SerialManager::readSerialCommand() {
    // if the command has already been read then skip and return default vals
    if (readyToRead) return SerialCommand{.cmd = ' ', .value = 0.0 };

    SerialCommand command;
    command.cmd = cmd;
    command.value = inputNumber;
    
    // Clear command, so that the serial updater will register the next command
    cmd = ' ';
    readyToRead = true;

    memset(numbers, 'a', sizeof(numbers));
    numCounter = 0;

    return command;
}