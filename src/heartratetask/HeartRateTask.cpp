#include "heartratetask/HeartRateTask.h"
#include <drivers/Hrs3300.h>
#include <components/heartrate/HeartRateController.h>
#include <nrf_log.h>

using namespace Pinetime::Applications;

<<<<<<< HEAD
TickType_t CurrentTaskDelay(HeartRateTask::States state, TickType_t ppgDeltaTms) {
  switch (state) {
    case HeartRateTask::States::ScreenOnAndMeasuring:
    case HeartRateTask::States::ScreenOffAndMeasuring:
      return ppgDeltaTms;
    case HeartRateTask::States::ScreenOffAndWaiting:
      return pdMS_TO_TICKS(1000);
    default:
      return portMAX_DELAY;
  }
}


=======
>>>>>>> main
HeartRateTask::HeartRateTask(Drivers::Hrs3300& heartRateSensor,
                             Controllers::HeartRateController& controller,
                             Controllers::Settings& settings)
  : heartRateSensor {heartRateSensor}, controller {controller}, settings {settings} {
}

void HeartRateTask::Start() {
  messageQueue = xQueueCreate(10, 1);
  controller.SetHeartRateTask(this);

  if (pdPASS != xTaskCreate(HeartRateTask::Process, "Heartrate", 500, this, 0, &taskHandle)) {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

void HeartRateTask::Process(void* instance) {
  auto* app = static_cast<HeartRateTask*>(instance);
  app->Work();
}

void HeartRateTask::Work() {
  int lastBpm = 0;

  while (true) {
<<<<<<< HEAD
    TickType_t delay = CurrentTaskDelay(state, ppg.deltaTms);
=======
    TickType_t delay = CurrentTaskDelay();
>>>>>>> main
    Messages msg;

    if (xQueueReceive(messageQueue, &msg, delay) == pdTRUE) {
      switch (msg) {
        case Messages::GoToSleep:
<<<<<<< HEAD
          HandleGoToSleep();
          break;
        case Messages::WakeUp:
          HandleWakeUp();
          break;
        case Messages::StartMeasurement:
          HandleStartMeasurement(&lastBpm);
          break;
        case Messages::StopMeasurement:
          HandleStopMeasurement();
=======
          if (state == States::Running) {
            state = States::Idle;
          } else if (state == States::Measuring) {
            state = States::BackgroundWaiting;
            StartWaiting();
          }
          break;
        case Messages::WakeUp:
          if (state == States::Idle) {
            state = States::Running;
          } else if (state == States::BackgroundMeasuring) {
            state = States::Measuring;
          } else if (state == States::BackgroundWaiting) {
            state = States::Measuring;
            StartMeasurement();
          }
          break;
        case Messages::StartMeasurement:
          if (state == States::Measuring || state == States::BackgroundMeasuring) {
            break;
          }
          state = States::Measuring;
          lastBpm = 0;
          StartMeasurement();
          break;
        case Messages::StopMeasurement:
          if (state == States::Running || state == States::Idle) {
            break;
          }
          if (state == States::Measuring) {
            state = States::Running;
          } else if (state == States::BackgroundMeasuring) {
            state = States::Idle;
          }
          StopMeasurement();
>>>>>>> main
          break;
      }
    }

<<<<<<< HEAD
<<<<<<< HEAD
    if (measurementStarted) {
      auto sensorData = heartRateSensor.ReadHrsAls();
      int8_t ambient = ppg.Preprocess(sensorData.hrs, sensorData.als);
      int bpm = ppg.HeartRate();

      // If ambient light detected or a reset requested (bpm < 0)
      if (ambient > 0) {
        // Reset all DAQ buffers
        ppg.Reset(true);
        // Force state to NotEnoughData (below)
        lastBpm = 0;
        bpm = 0;
      } else if (bpm < 0) {
        // Reset all DAQ buffers except HRS buffer
        ppg.Reset(false);
        // Set HR to zero and update
        bpm = 0;
        controller.Update(Controllers::HeartRateController::States::Running, bpm);
      }

      if (lastBpm == 0 && bpm == 0) {
        controller.Update(Controllers::HeartRateController::States::NotEnoughData, bpm);
      }

      if (bpm != 0) {
        lastBpm = bpm;
        controller.Update(Controllers::HeartRateController::States::Running, lastBpm);
      }
=======
    switch (state) {
      case States::ScreenOffAndWaiting:
        HandleBackgroundWaiting();
        break;
      case States::ScreenOffAndMeasuring:
      case States::ScreenOnAndMeasuring:
        HandleSensorData(&lastBpm);
        break;
      case States::ScreenOffAndStopped:
      case States::ScreenOnAndStopped:
        // nothing to do -> ignore
        break;
>>>>>>> remotes/heartrate-measurements-in-background/heartrate-measurements-in-background
=======
    if (state == States::BackgroundWaiting) {
      HandleBackgroundWaiting();
    } else if (state == States::BackgroundMeasuring || state == States::Measuring) {
      HandleSensorData(&lastBpm);
>>>>>>> main
    }
  }
}

void HeartRateTask::PushMessage(HeartRateTask::Messages msg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(messageQueue, &msg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HeartRateTask::StartMeasurement() {
  heartRateSensor.Enable();
  ppg.Reset(true);
  vTaskDelay(100);
  measurementStart = xTaskGetTickCount();
}

void HeartRateTask::StopMeasurement() {
  heartRateSensor.Disable();
  ppg.Reset(true);
  vTaskDelay(100);
}

<<<<<<< HEAD
void HeartRateTask::HandleGoToSleep() {
  switch (state) {
    case States::ScreenOnAndStopped:
      state = States::ScreenOffAndStopped;
      break;
    case States::ScreenOnAndMeasuring:
      state = States::ScreenOffAndMeasuring;
      break;
    case States::ScreenOffAndStopped:
    case States::ScreenOffAndWaiting:
    case States::ScreenOffAndMeasuring:
      // shouldn't happen -> ignore
      break;
  }
}

void HeartRateTask::HandleWakeUp() {
  switch (state) {
    case States::ScreenOffAndStopped:
      state = States::ScreenOnAndStopped;
      break;
    case States::ScreenOffAndMeasuring:
      state = States::ScreenOnAndMeasuring;
      break;
    case States::ScreenOffAndWaiting:
      state = States::ScreenOnAndMeasuring;
      StartMeasurement();
      break;
    case States::ScreenOnAndStopped:
    case States::ScreenOnAndMeasuring:
      // shouldn't happen -> ignore
      break;
  }
}

void HeartRateTask::HandleStartMeasurement(int* lastBpm) {
  switch (state) {
    case States::ScreenOffAndStopped:
    case States::ScreenOnAndStopped:
      state = States::ScreenOnAndMeasuring;
      *lastBpm = 0;
      StartMeasurement();
      break;
    case States::ScreenOnAndMeasuring:
    case States::ScreenOffAndMeasuring:
    case States::ScreenOffAndWaiting:
      // shouldn't happen -> ignore
      break;
  }
}

void HeartRateTask::HandleStopMeasurement() {
  switch (state) {
    case States::ScreenOnAndMeasuring:
      state = States::ScreenOnAndStopped;
      StopMeasurement();
      break;
    case States::ScreenOffAndMeasuring:
    case States::ScreenOffAndWaiting:
      state = States::ScreenOffAndStopped;
      StopMeasurement();
      break;
    case States::ScreenOnAndStopped:
    case States::ScreenOffAndStopped:
      // shouldn't happen -> ignore
      break;
  }
=======
void HeartRateTask::StartWaiting() {
  StopMeasurement();
  backgroundWaitingStart = xTaskGetTickCount();
>>>>>>> main
}

void HeartRateTask::HandleBackgroundWaiting() {
  if (!IsBackgroundMeasurementActivated()) {
    return;
  }

<<<<<<< HEAD
  if (ShouldStartBackgroundMeasuring()) {
    state = States::ScreenOffAndMeasuring;
=======
  TickType_t ticksSinceWaitingStart = xTaskGetTickCount() - backgroundWaitingStart;
  if (ticksSinceWaitingStart >= GetHeartRateBackgroundMeasurementIntervalInTicks()) {
    state = States::BackgroundMeasuring;
>>>>>>> main
    StartMeasurement();
  }
}

void HeartRateTask::HandleSensorData(int* lastBpm) {
  int8_t ambient = ppg.Preprocess(heartRateSensor.ReadHrs(), heartRateSensor.ReadAls());
  int bpm = ppg.HeartRate();

  // If ambient light detected or a reset requested (bpm < 0)
  if (ambient > 0) {
    // Reset all DAQ buffers
    ppg.Reset(true);
  } else if (bpm < 0) {
    // Reset all DAQ buffers except HRS buffer
    ppg.Reset(false);
    // Set HR to zero and update
    bpm = 0;
  }

<<<<<<< HEAD
  bool notEnoughData = *lastBpm == 0 && bpm == 0;
  if (notEnoughData) {
=======
  if (*lastBpm == 0 && bpm == 0) {
>>>>>>> main
    controller.Update(Controllers::HeartRateController::States::NotEnoughData, bpm);
  }

  if (bpm != 0) {
    *lastBpm = bpm;
    controller.Update(Controllers::HeartRateController::States::Running, bpm);
<<<<<<< HEAD
  }

  if (state == States::ScreenOnAndMeasuring || IsContinuousModeActivated()) {
    return;
  } 

  // state == States::ScreenOffAndMeasuring 
  //    (because state != ScreenOnAndMeasuring and the only state that enables measuring is ScreenOffAndMeasuring)
  // !IsContinuousModeActivated()

  if (ShouldStartBackgroundMeasuring()) {
    // This doesn't change the state but resets the measurment timer, which basically starts the next measurment without resetting the sensor.
    // This is basically a fall back to continuous mode, when measurments take too long.
    measurementStart = xTaskGetTickCount();
    return;
  }

  bool noDataWithinTimeLimit = bpm == 0 && ShoudStopTryingToGetData();
  bool dataWithinTimeLimit = bpm != 0;
  if (dataWithinTimeLimit || noDataWithinTimeLimit) {
    state = States::ScreenOffAndWaiting;
    StopMeasurement();
  }

}

TickType_t HeartRateTask::GetHeartRateBackgroundMeasurementIntervalInTicks() {
  int ms;
  switch (settings.GetHeartRateBackgroundMeasurementInterval()) {
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::FifteenSeconds:
      ms = 15 * 1000;
      break;
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::ThirtySeconds:
      ms = 30 * 1000;
      break;
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::OneMinute:
      ms = 60 * 1000;
      break;
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::FiveMinutes:
      ms = 5 * 60 * 1000;
      break;
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::TenMinutes:
      ms = 10 * 60 * 1000;
      break;
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::ThirtyMinutes:
      ms = 30 * 60 * 1000;
      break;
    default:
      ms = 0;
      break;
  }
  return pdMS_TO_TICKS(ms);
}

bool HeartRateTask::IsContinuousModeActivated() {
=======
    if (state == States::Measuring || IsContinuosModeActivated()) {
      return;
    }
    if (state == States::BackgroundMeasuring) {
      state = States::BackgroundWaiting;
      StartWaiting();
    }
  }
  TickType_t ticksSinceMeasurementStart = xTaskGetTickCount() - measurementStart;
  if (bpm == 0 && state == States::BackgroundMeasuring && !IsContinuosModeActivated() &&
      ticksSinceMeasurementStart >= DURATION_UNTIL_BACKGROUND_MEASURMENT_IS_STOPPED) {
    state = States::BackgroundWaiting;
    StartWaiting();
  }
  if (bpm == 0 && state == States::BackgroundMeasuring &&
      xTaskGetTickCount() - measurementStart >= DURATION_UNTIL_BACKGROUND_MEASURMENT_IS_STOPPED) {
    state = States::BackgroundWaiting;
    StartWaiting();
  }
}

TickType_t HeartRateTask::CurrentTaskDelay() {
  switch (state) {
    case States::Measuring:
    case States::BackgroundMeasuring:
      return ppg.deltaTms;
    case States::Running:
      return pdMS_TO_TICKS(100);
    case States::BackgroundWaiting:
      return pdMS_TO_TICKS(10000);
    default:
      return portMAX_DELAY;
  }
}

TickType_t HeartRateTask::GetHeartRateBackgroundMeasurementIntervalInTicks() {
  switch (settings.GetHeartRateBackgroundMeasurementInterval()) {
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::TenSeconds:
      return pdMS_TO_TICKS(10 * 1000);
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::ThirtySeconds:
      return pdMS_TO_TICKS(30 * 1000);
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::OneMinute:
      return pdMS_TO_TICKS(60 * 1000);
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::FiveMinutes:
      return pdMS_TO_TICKS(5 * 60 * 1000);
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::TenMinutes:
      return pdMS_TO_TICKS(10 * 60 * 1000);
    case Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::ThirtyMinutes:
      return pdMS_TO_TICKS(30 * 60 * 1000);
    default:
      return 0;
  }
}

bool HeartRateTask::IsContinuosModeActivated() {
>>>>>>> main
  return settings.GetHeartRateBackgroundMeasurementInterval() ==
         Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::Continuous;
}

bool HeartRateTask::IsBackgroundMeasurementActivated() {
  return settings.GetHeartRateBackgroundMeasurementInterval() !=
         Pinetime::Controllers::Settings::HeartRateBackgroundMeasurementInterval::Off;
}
<<<<<<< HEAD

TickType_t HeartRateTask::GetTicksSinceLastMeasurementStarted() {
  return xTaskGetTickCount() - measurementStart;
}

bool HeartRateTask::ShoudStopTryingToGetData() {
  return GetTicksSinceLastMeasurementStarted() >= DURATION_UNTIL_BACKGROUND_MEASUREMENT_IS_STOPPED;
}

bool HeartRateTask::ShouldStartBackgroundMeasuring() {
  return GetTicksSinceLastMeasurementStarted() >= GetHeartRateBackgroundMeasurementIntervalInTicks();
}
=======
>>>>>>> main
