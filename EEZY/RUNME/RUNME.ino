#include "Prog.h"
#include "SerialCommunication.h"
#include "soc/rtc_wdt.h"

TaskHandle_t RobotControlTaskHandle, SerialCommunicationTaskHandle;
SemaphoreHandle_t last_pos_mutex, requested_pos_mutex, operating_mode_mutex, executing_state_mutex;

SerialCommunication communication;

void setup()
{
  Serial.begin(115200);

  // disable watchdog
  rtc_wdt_protect_off();
  rtc_wdt_disable();

  // create semaphores
  last_pos_mutex = xSemaphoreCreateMutex();
  requested_pos_mutex = xSemaphoreCreateMutex();
  operating_mode_mutex = xSemaphoreCreateMutex();
  executing_state_mutex = xSemaphoreCreateMutex();

  // create the task for the communication
  xTaskCreatePinnedToCore(
      CommunicationTask,               // task function
      "SerialCommunicationTaskHandle", // name of task
      10000,                           // stack size of task
      NULL,                            // parameter of the task
      1,                               // priority of the task
      &SerialCommunicationTaskHandle,  // task handle to keep track of created task
      1);                              // pin task to core 1

  // create the task for the robot control
  xTaskCreatePinnedToCore(
      RobotTask,                // task function
      "RobotControlTaskHandle", // name of task
      100000,                   // stack size of task
      NULL,                     // parameter of the task
      1,                        // priority of the task
      &RobotControlTaskHandle,  // task handle to keep track of created task
      1);                       // pin task to core 1
}

void loop()
{
}

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
    if (operating_mode.equals("automatic"))
    {
      if (xSemaphoreTake(executing_state_mutex, portMAX_DELAY) == pdTRUE)
      {
        communication.SetRobotExecutingState(true);
        xSemaphoreGive(executing_state_mutex);
      }
      MAIN();
      if (xSemaphoreTake(executing_state_mutex, portMAX_DELAY) == pdTRUE)
      {
        communication.SetRobotExecutingState(false);
        xSemaphoreGive(executing_state_mutex);
      }
    }
    else if (operating_mode.equals("manual"))
    {
      float *pos;
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
  for (;;)
  {
    communication.HandleSerialCommunication();
  }
}
