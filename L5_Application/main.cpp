#include "FreeRTOS.h"
#include "LPC17xx.h"

#include "pin_t.hpp"
#include "source/cmd_handlers/mp3Handler.hpp"
#include "sys_config.h"
#include "task.h"
#include "tasks.hpp"
#include "uart0_min.h"

#include "SPIController.hpp"
#include "VS1053.hpp"

int main(void) {
    static VS1053 mp3Decoder;
    pin_t reset = {2, 4};
    pin_t dreq = {2, 5};
    pin_t control_cs = {2, 6};
    pin_t data_cs = {2, 7};

    /* Configure SPI pin functions */
    LPC_PINCON->PINSEL0 &= ~(3 << 30);
    LPC_PINCON->PINSEL0 |=  (2 << 30);
    LPC_PINCON->PINSEL1 &= ~((3 << 4) | (3 << 2));
    LPC_PINCON->PINSEL1 |=  ((2 << 4) | (2 << 2));

    if(!mp3Decoder.init(SPIController::SSP0, reset, data_cs, control_cs, dreq))
    {
        uart0_puts("Failed to initialize decoder");

        while(1)
        {
            ;
        }
    }

    mp3CmdDec = &mp3Decoder;

    scheduler_add_task(new terminalTask(2));

    scheduler_start();

    return 0;
}
