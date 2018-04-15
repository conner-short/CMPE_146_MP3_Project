#include "FreeRTOS.h"
#include "sys_config.h"
#include "task.h"
#include "tasks.hpp"
#include "uart0_min.h"
#include "VS1053.hpp"

void vDebugTask(void* p)
{
    VS1053* dec = (VS1053*)p;

    FIL file;

    if(f_open(&file, "1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\02_-_Moar_Ghosts_\'n\'_Stuff.mp3", FA_READ) == FR_OK)
    {
        dec->play(&file);
    }
    else
    {
        uart0_puts("fs error");
    }

    vTaskSuspend(NULL);
}

int main(void) {
    const uint32_t STACK_SIZE = 1024;

    static VS1053 mp3Decoder;
    VS1053::pin_t data_cs = {2, 7};
    VS1053::pin_t control_cs = {2, 6};
    VS1053::pin_t dreq = {2, 5};

    if(!spi.init(LabSPI::SSP1, 8, LabSPI::IDLE_LOW_CAPTURE_RISING, 8))
    {
        uart0_puts("ssp1 init failed");

        while(1)
        {
            ;
        }
    }

    if(!mp3Decoder.init(LabSPI::SSP0, data_cs, control_cs, dreq))
    {
        uart0_puts("decoder init failed.");

        while(1)
        {
            ;
        }
    }

    scheduler_add_task(new terminalTask(1));

    xTaskCreate(vDebugTask, "debug", STACK_SIZE, &mp3Decoder, 1, NULL);

    scheduler_start();

    return 0;
}
