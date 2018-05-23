#include "FreeRTOS.h"
#include "LPC17xx.h"

#include "pin_t.hpp"
#include "source/cmd_handlers/mp3Handler.hpp"
#include "sys_config.h"
#include "task.h"
#include "tasks.hpp"
#include "uart0_min.h"

#include "Scroll_Nav.hpp"
#include "SPIController.hpp"
#include "VS1053.hpp"

void scrollInfoTask(void* p)
{
    Scroll_Nav nav;

    pin_t s4;
    s4.port = 2; s4.pin = 3;

    pin_t s2;
    s2.port = 2; s2.pin = 1;

    pin_t s3;
    s3.port = 2; s3.pin = 2;

    pin_t s5;
    s5.port = 0; s5.pin = 0;

    pin_t wheel_a;
    wheel_a.port = 0; wheel_a.pin = 29;

    pin_t wheel_b;
    wheel_b.port = 0; wheel_b.pin = 30;

    if(nav.init(NULL, &s2, &s3, &s4, &s5, &wheel_a, &wheel_b))
    {
        while(1)
        {
            switch(nav.waitForNextEvent(portMAX_DELAY))
            {
                case Scroll_Nav::NONE: uart0_puts("ev: NONE"); break;
                case Scroll_Nav::WHEEL_CW: uart0_puts("ev: WHEEL_CW"); break;
                case Scroll_Nav::WHEEL_CCW: uart0_puts("ev: WHEEL_CCW"); break;
                case Scroll_Nav::S1_DOWN: uart0_puts("ev: S1_DOWN"); break;
                case Scroll_Nav::S1_UP: uart0_puts("ev: S1_UP"); break;
                case Scroll_Nav::S2_DOWN: uart0_puts("ev: S2_DOWN"); break;
                case Scroll_Nav::S2_UP: uart0_puts("ev: S2_UP"); break;
                case Scroll_Nav::S3_DOWN: uart0_puts("ev: S3_DOWN"); break;
                case Scroll_Nav::S3_UP: uart0_puts("ev: S3_UP"); break;
                case Scroll_Nav::S4_DOWN: uart0_puts("ev: S4_DOWN"); break;
                case Scroll_Nav::S4_UP: uart0_puts("ev: S4_UP"); break;
                case Scroll_Nav::S5_DOWN: uart0_puts("ev: S5_DOWN"); break;
                case Scroll_Nav::S5_UP: uart0_puts("ev: S5_UP"); break;
                default: uart0_puts("ev: Error"); break;
            }
        }
    }
    else
    {
        uart0_puts("Error: could not init scroll wheel");

        vTaskSuspend(NULL);
    }
}

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

    xTaskCreate(scrollInfoTask, "Scroll", 1024, NULL, 1, NULL);

    scheduler_start();

    return 0;
}
