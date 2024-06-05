#include "SerialCommunication.h"

SerialCommunication::SerialCommunication()
{
  // initialize the mode
  mode = "automatic";

  // start the writing timer for serial communication
  last_serial_write_time = millis();
}

String SerialCommunication::GetOperatingMode()
{
  return mode;
}

float *SerialCommunication::GetRequestedPosition()
{
  return robot_requested_position;
}

void SerialCommunication::SetRobotExecutingState(const bool new_state)
{
  executing_state = new_state;
}

void SerialCommunication::HandleSerialCommunication()
{
  ReadSerialPort();

  // send packets every 30 ms
  if (millis() - last_serial_write_time >= 30)
  {
    last_serial_write_time = millis();
    WriteToSerialPort();
  }
}

void SerialCommunication::ReadSerialPort()
{
  if (Serial.available() > 0)
  {
    // read the incoming packet
    String packet = Serial.readStringUntil('\n');

    // extract mode from the packet
    int comma_index = packet.indexOf(',');
    String new_mode = packet.substring(0, comma_index);
    packet.remove(0, comma_index + 1);

    // temporary array to hold the requested position
    float temp_requested_position[3];

    // save requested position
    for (int i = 0; i < 3; ++i)
    {
      comma_index = packet.indexOf(',');
      temp_requested_position[i] = packet.substring(0, comma_index).toFloat();
      packet.remove(0, comma_index + 1);
    }

    // acquire mutex to update requested position
    if (xSemaphoreTake(requested_pos_mutex, portMAX_DELAY) == pdTRUE)
    {
      for (int i = 0; i < 3; ++i)
      {
        robot_requested_position[i] = temp_requested_position[i];
      }
      xSemaphoreGive(requested_pos_mutex);
    }

    if (xSemaphoreTake(operating_mode_mutex, portMAX_DELAY) == pdTRUE)
    {
      mode = new_mode;
      xSemaphoreGive(operating_mode_mutex);
    }
  }
}

void SerialCommunication::WriteToSerialPort()
{
  String mode_state;
  float *robot_last_position;
  bool current_state;

  if (xSemaphoreTake(last_pos_mutex, portMAX_DELAY) == pdTRUE)
  {
    robot_last_position = GetLastPosition();
    xSemaphoreGive(last_pos_mutex);
  }

  if (xSemaphoreTake(operating_mode_mutex, portMAX_DELAY) == pdTRUE)
  {
    mode_state = mode;
    xSemaphoreGive(operating_mode_mutex);
  }

  if (xSemaphoreTake(executing_state_mutex, portMAX_DELAY) == pdTRUE)
  {
    current_state = executing_state;
    xSemaphoreGive(executing_state_mutex);
  }

  if (mode_state.equals("automatic") || (mode_state.equals("manual") && current_state == true))
  {
    mode_state = "automatic";
    if (xSemaphoreTake(requested_pos_mutex, portMAX_DELAY) == pdTRUE)
    {
      robot_requested_position[0] = robot_last_position[0];
      robot_requested_position[1] = robot_last_position[1];
      robot_requested_position[2] = robot_last_position[2];
      xSemaphoreGive(requested_pos_mutex);
    }
  }

  String buffer = mode_state + "," + String(robot_last_position[0]) + "," + String(robot_last_position[1]) + "," + String(robot_last_position[2]) + ",";
  Serial.println(buffer);
}
