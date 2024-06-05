/*
  EEZYbotCONTROL.h - Library for controling the EEZYbotARM mk2.
  Created by João Coelho, 20 February, 2024.
  Released into the public domain.
*/

#include <ESP32Servo.h>

void Start();

void FK(const float*, float*);

void IK(const float*, float*);

float** LinearInterpolation(const float*, const float*, float, int*, float*);

float** JointInterpolation(const float*, const float*, float, int*, float*);

void MoveL(const float*, const float);

void MoveJ(const float*, const float);

void WriteToPhysicalMotors(const float*);

void ToggleGripper(const bool);

void WaitTime(const int);

void UpdateRobotPosition(const float*);

void SetLastPosition(const float*);

float* GetLastPosition();