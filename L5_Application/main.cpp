#include "FreeRTOS.h"
#include "source/cmd_handlers/mp3Handler.hpp"
#include "sys_config.h"
#include "task.h"
#include "tasks.hpp"
#include "uart0_min.h"
#include "VS1053.hpp"

int main(void) {
    static VS1053 mp3Decoder;
    VS1053::pin_t reset = {2, 4};
    VS1053::pin_t dreq = {2, 5};
    VS1053::pin_t control_cs = {2, 6};
    VS1053::pin_t data_cs = {2, 7};

    if(!mp3Decoder.init(LabSPI::SSP0, reset, data_cs, control_cs, dreq))
    {
        uart0_puts("Failed to initialize decoder");

        while(1)
        {
            ;
        }
    }

    mp3CmdDec = &mp3Decoder;

    scheduler_add_task(new terminalTask(1));

    /*xTaskCreate(vDebugTask, "Debug", STACK_SIZE, &mp3Decoder, 1, NULL);*/

    scheduler_start();

    return 0;
}
