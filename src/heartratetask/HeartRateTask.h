#pragma once
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <components/heartrate/Ppg.h>
#include "components/settings/Settings.h"

<<<<<<< HEAD
#define DURATION_UNTIL_BACKGROUND_MEASUREMENT_IS_STOPPED pdMS_TO_TICKS(30 * 1000)
=======
#define DURATION_UNTIL_BACKGROUND_MEASURMENT_IS_STOPPED pdMS_TO_TICKS(30 * 1000)
>>>>>>> main

namespace Pinetime {
  namespace Drivers {
    class Hrs3300;
  }

  namespace Controllers {
    class HeartRateController;
  }

  namespace Applications {
    class HeartRateTask {
    public:
<<<<<<< HEAD
      enum class Messages : uint8_t {
        GoToSleep,
        WakeUp,
        StartMeasurement,
        StopMeasurement
      };

      enum class States {
        ScreenOnAndStopped,
        ScreenOnAndMeasuring,
        ScreenOffAndStopped,
        ScreenOffAndWaiting,
        ScreenOffAndMeasuring
      };
=======
      enum class Messages : uint8_t { GoToSleep, WakeUp, StartMeasurement, StopMeasurement };
      enum class States { Idle, Running, Measuring, BackgroundWaiting, BackgroundMeasuring };
>>>>>>> main

      explicit HeartRateTask(Drivers::Hrs3300& heartRateSensor,
                             Controllers::HeartRateController& controller,
                             Controllers::Settings& settings);
      void Start();
      void Work();
      void PushMessage(Messages msg);

    private:
      static void Process(void* instance);
      void StartMeasurement();
      void StopMeasurement();
      void StartWaiting();

      void HandleBackgroundWaiting();
      void HandleSensorData(int* lastBpm);
      TickType_t CurrentTaskDelay();

      TickType_t GetHeartRateBackgroundMeasurementIntervalInTicks();
      bool IsContinuosModeActivated();
      bool IsBackgroundMeasurementActivated();

      void HandleGoToSleep();
      void HandleWakeUp();
      void HandleStartMeasurement(int* lastBpm);
      void HandleStopMeasurement();

      void HandleBackgroundWaiting();
      void HandleSensorData(int* lastBpm);

      TickType_t GetHeartRateBackgroundMeasurementIntervalInTicks();
      bool IsContinuousModeActivated();
      bool IsBackgroundMeasurementActivated();

      TickType_t GetTicksSinceLastMeasurementStarted();
      bool ShoudStopTryingToGetData();
      bool ShouldStartBackgroundMeasuring();

      TaskHandle_t taskHandle;
      QueueHandle_t messageQueue;
      States state = States::ScreenOnAndStopped;
      Drivers::Hrs3300& heartRateSensor;
      Controllers::HeartRateController& controller;
      Controllers::Settings& settings;
      Controllers::Ppg ppg;
<<<<<<< HEAD
=======
      TickType_t backgroundWaitingStart = 0;
>>>>>>> main
      TickType_t measurementStart = 0;
    };

  }
}
