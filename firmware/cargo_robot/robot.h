// The robot's parts and live readings, shared by the main program and the serial console.
#pragma once
#include "control.h"
#include "hw.h"

extern Motor motorL, motorR;
extern Encoder encL, encR;
extern BedServo servoFront, servoRear;
extern bool manualMotors;          // true while a bench test drives the motors directly (no wheel control)

// Updated by robotUpdate().
struct Readings {
  float busV = 0;                  // motor 12 V bus; below BUS_ON_V = E-stop pressed
  Line line;
  float sonarCm = 0;               // ultrasonic, straight ahead
  float wheelL = 0, wheelR = 0;    // rad/s
  Odometry odom;
  float bedDeg = 0;                // where the servos have been told to hold the bed
};
extern Readings rd;

enum class Light { Off, On, Slow, Fast };

void robotBegin();                 // start every part (called once from setup)
void robotUpdate();                // call as often as possible: sensors, wheel control, bed, lights, buzzer
void driveTarget(float v, float w);   // wanted speed: m/s forward, rad/s turning left (ramped by "accel")
void driveStop();                  // stop now
void motorsEnable(bool on);
void bedMove(float deg);           // move the bed smoothly (tip_speed) to deg; 0 = flat
bool bedMoving();
void bedRelax();                   // servos off: the bed rests flat on its two posts
void lights(Light green, Light red);
void beep(int ms);
void applyInversions();            // after changing inv_* settings
