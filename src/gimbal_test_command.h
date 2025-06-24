#ifndef GIMBAL_TEST_COMMAND_H
#define GIMBAL_TEST_COMMAND_H

#include <Arduino.h>

// Test command timing variables
extern unsigned long lastTestCommand;
extern const unsigned long TEST_COMMAND_INTERVAL;
extern bool enable_test_command;

// Function declarations
void sendDJITestCommand();
void enableTestMode(bool enable = true);

#endif
