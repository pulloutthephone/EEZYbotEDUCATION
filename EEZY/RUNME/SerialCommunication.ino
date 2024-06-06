#include "SerialCommunication.h"

SerialCommunication::SerialCommunication() {
    mode                   = "automatic"; // initialize robot working mode
    last_serial_write_time = millis();    // start the writing timer for serial communication
}

String SerialCommunication::GetOperatingMode() {
    return mode;
}

float *SerialCommunication::GetRequestedPosition() {
    return robot_requested_position;
}

void SerialCommunication::HandleSerialCommunication() {
    ReadSerialPort();

    // send packets every 30 ms
    if (millis() - last_serial_write_time >= 30) {
        last_serial_write_time = millis();
        WriteToSerialPort();
    }
}

void SerialCommunication::ReadSerialPort() {
    if (Serial.available() <= 0) return;

    // read the incoming packet
    String packet = Serial.readStringUntil('\n');

    // extract mode from the packet
    int comma_index      = packet.indexOf(',');
    String incoming_mode = packet.substring(0, comma_index);
    packet.remove(0, comma_index + 1);

    // temporary array to hold the requested position
    float temp_requested_position[3];

    // save requested position
    for (int i = 0; i < 3; ++i) {
        comma_index                = packet.indexOf(',');
        temp_requested_position[i] = packet.substring(0, comma_index).toFloat();
        packet.remove(0, comma_index + 1);
    }

    // acquire mutex to update requested position
    if (xSemaphoreTake(requested_pos_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 3; ++i) {
            robot_requested_position[i] = temp_requested_position[i];
        }
        xSemaphoreGive(requested_pos_mutex);
    }

    // acquire mutex to update operating mode
    if (xSemaphoreTake(operating_mode_mutex, portMAX_DELAY) == pdTRUE) {
        mode = incoming_mode;

        // reset robot task if manual mode has been requested
        if (!mode.equals(last_mode) && mode.equals("manual")) {
            vTaskDelete(RobotControlTaskHandle);

            xTaskCreatePinnedToCore(
                RobotTask,                 // task function
                "RobotControlTaskHandle",  // name of task
                100000,                    // stack size of task
                NULL,                      // parameter of the task
                1,                         // priority of the task
                &RobotControlTaskHandle,   // task handle to keep track of created task
                1);                        // pin task to core 1
        }

        last_mode = mode;

        xSemaphoreGive(operating_mode_mutex);
    }
}

void SerialCommunication::WriteToSerialPort() {
    String current_mode;
    float *robot_last_position;
    bool current_state;

    if (xSemaphoreTake(last_pos_mutex, portMAX_DELAY) == pdTRUE) {
        robot_last_position = GetLastPosition();
        xSemaphoreGive(last_pos_mutex);
    }

    if (xSemaphoreTake(operating_mode_mutex, portMAX_DELAY) == pdTRUE) {
        current_mode = mode;
        xSemaphoreGive(operating_mode_mutex);
    }

    String buffer = current_mode + "," + String(robot_last_position[0]) + "," + String(robot_last_position[1]) + "," + String(robot_last_position[2]) + ",";
    Serial.println(buffer);
}