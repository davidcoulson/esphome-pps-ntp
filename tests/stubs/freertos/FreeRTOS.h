#pragma once
#include <cstdint>
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void) (x)
#define portEXIT_CRITICAL(x) (void) (x)
#define pdPASS 1
#define tskNO_AFFINITY 0x7FFFFFFF
#define pdMS_TO_TICKS(x) (x)
inline int xPortGetCoreID() { return 0; }
inline void vTaskDelay(int) {}
inline unsigned uxTaskGetStackHighWaterMark(TaskHandle_t) { return 0; }
inline BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *, int, void *, int, TaskHandle_t *, BaseType_t) { return pdPASS; }
