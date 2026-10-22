// Serial-Monitor commands (115200 baud, line ending "Newline"): bench tests, calibration and settings.
// Type "help" for the list.
#pragma once

void consolePoll();                // call from loop(): runs a command when a whole line has arrived
void startMission(int deliveries); // defined in cargo_robot.ino
void stopMission();
const char *stateName();
