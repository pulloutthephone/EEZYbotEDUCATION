#include "Prog.h"
#include "SerialCommunication.h"
#include <esp_task_wdt.h>

TaskHandle_t      RobotControlTaskHandle;
TaskHandle_t      SerialCommunicationTaskHandle;
SemaphoreHandle_t last_pos_mutex;
SemaphoreHandle_t requested_pos_mutex;
SemaphoreHandle_t operating_mode_mutex;

SerialCommunication communication;

void setup()
{
  Serial.begin(115200);

  // disable watchdog
  esp_task_wdt_deinit();

  // create semaphores
  last_pos_mutex       = xSemaphoreCreateMutex();
  requested_pos_mutex  = xSemaphoreCreateMutex();
  operating_mode_mutex = xSemaphoreCreateMutex();

  // create the task for the communication
  xTaskCreatePinnedToCore(
    CommunicationTask,                // task function
    "SerialCommunicationTaskHandle",  // name of task
    10000,                            // stack size of task
    NULL,                             // parameter of the task
    1,                                // priority of the task
    &SerialCommunicationTaskHandle,   // task handle to keep track of created task
    1);                               // core pinned task

  // create the task for the robot control
  xTaskCreatePinnedToCore(
    RobotTask,                        // task function
    "RobotControlTaskHandle",         // name of task
    100000,                           // stack size of task
    NULL,                             // parameter of the task
    2,                                // priority of the task
    &RobotControlTaskHandle,          // task handle to keep track of created task
    1);                               // core pinned task
}

void loop() {}

void RobotTask(void *pvParameters)
{
  Start();
  for (;;)
  {
    String operating_mode;
    if (xSemaphoreTake(operating_mode_mutex, portMAX_DELAY) == pdTRUE)
    {
      operating_mode = communication.GetOperatingMode();
      xSemaphoreGive(operating_mode_mutex);
    }

    if      (operating_mode.equals("automatic")) MAIN();
    else if (operating_mode.equals("manual"))
    {
      float *pos = nullptr;
      if (xSemaphoreTake(requested_pos_mutex, portMAX_DELAY) == pdTRUE)
      {
        pos = communication.GetRequestedPosition();
        xSemaphoreGive(requested_pos_mutex);
      }
      UpdateRobotPosition(pos);
    }
  }
}

void CommunicationTask(void *pvParameters)
{
  for (;;) communication.HandleSerialCommunication();
}
