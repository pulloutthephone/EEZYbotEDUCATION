/*
  EEZYbotCONTROL.h - Library for controling the EEZYbotARM mk2.
  Created by João Coelho, 20 February, 2024.
  Released into the public domain.
*/

#include <ESP32Servo.h>

// public functions
void Start();
void MoveL(const float*, const float);
void MoveJ(const float*, const float);
void ToggleGripper(const bool);
void WaitTime(const int);
void UpdateRobotPosition(const float*);
void SetLastPosition(const float*);
float *GetLastPosition();

// private functions
static void FK(const float*, float*);
static void IK(const float*, float*);
static float **LinearInterpolation(const float*, const float*, float, int*, float*);
static float **JointInterpolation(const float*, const float*, float, int*, float*);
static void WriteToPhysicalMotors(const float*);