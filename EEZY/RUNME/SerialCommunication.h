#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include "EEZYbotCONTROL.h"

class SerialCommunication {
public:
  SerialCommunication();
  void HandleSerialCommunication();
  String GetOperatingMode();
  float* GetRequestedPosition();
  void SetRobotExecutingState(bool);

private:
  void ReadSerialPort();
  void WriteToSerialPort();

  String mode;
  float robot_requested_position[3];
  long int last_serial_write_time;
  bool executing_state;
};

#endif