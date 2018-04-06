#include "FreeRTOS.h"
#include "task.h"

int main(void) {
    const uint32_t STACK_SIZE = 1024;

    vTaskStartScheduler();

    return 0;
}
