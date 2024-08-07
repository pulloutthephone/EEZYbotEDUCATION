#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include "EEZYbotCONTROL.h"

class SerialCommunication
{
public:
  SerialCommunication();
  void HandleSerialCommunication();
  String GetOperatingMode();
  float *GetRequestedPosition();

private:
  void ReadSerialPort();
  void WriteToSerialPort();

  String mode, last_mode;
  float robot_requested_position[3];
  long int last_serial_write_time;
};

#endif