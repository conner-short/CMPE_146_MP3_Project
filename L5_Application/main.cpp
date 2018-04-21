#include "FreeRTOS.h"
#include "sys_config.h"
#include "task.h"
#include "tasks.hpp"
#include "uart0_min.h"
#include "VS1053.hpp"

void vDebugTask(void* p)
{
    VS1053* dec = (VS1053*)p;

    while(1)
    {
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\01_-_FML.mp3");
        vTaskDelay(20000);
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\02_-_Moar_Ghosts_\'n\'_Stuff.mp3");
        vTaskDelay(20000);
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\03_-_Ghosts_n_Stuff_(feat._Rob_Swire).mp3");
        vTaskDelay(10000);
        dec->setPlayType(VS1053::FF);
        vTaskDelay(10000);
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\04_-_Hi_Friend!.mp3");
        vTaskDelay(20000);
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\05_-_Bot.mp3");
        vTaskDelay(20000);
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\06_-_Word_Problems.mp3");
        vTaskDelay(20000);
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\07_-_Soma.mp3");
        vTaskDelay(20000);
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\08_-_Lack_of_A_Better_Name.mp3");
        vTaskDelay(20000);
        dec->play("1:\\Music\\deadmau5\\For_Lack_of_a_Better_Name\\09_-_The_16th_Hour.mp3");
        vTaskDelay(20000);
    }
}

int main(void) {
    const uint32_t STACK_SIZE = 1024;

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

    scheduler_add_task(new terminalTask(1));

    xTaskCreate(vDebugTask, "Debug", STACK_SIZE, &mp3Decoder, 1, NULL);

    scheduler_start();

    return 0;
}
