#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"

#include "LabGPIO.hpp"
#include "LabGPIOInterrupts.hpp"
#include "semphr.h"
#include "sys_config.h"
#include "task.h"
#include "VS1053.hpp"

#include "uart0_min.h"

#define EVENT_NEW_FILE_SELECTED  (1 << 0)
#define EVENT_DREQ_HIGH          (1 << 1)
#define EVENT_STOP_REQUESTED     (1 << 2)
#define EVENT_INIT               (1 << 3)

VS1053::VS1053() {}
VS1053::~VS1053() {}

bool VS1053::init(LabSPI::Peripheral spiChannel, pin_t& data_cs, pin_t& control_cs, pin_t& dreq)
{
    spiDev = spiChannel;

    /* SPI init with low startup speed */
    if(!spi.init(spiDev, 8, LabSPI::IDLE_LOW_CAPTURE_RISING, getSpiDivider(false)))
    {
        return false;
    }

    /* Configure GPIO pins */

    if(!dataCs.init(data_cs.port, data_cs.pin))
    {
        return false;
    }

    dataCs.setAsOutput();
    dataCs.setHigh();

    if(!controlCs.init(control_cs.port, control_cs.pin))
    {
        return false;
    }

    controlCs.setAsOutput();
    controlCs.setHigh();

    if(!dataReq.init(dreq.port, dreq.pin))
    {
        return false;
    }

    dataReq.setAsInput();

    /* Create SPI command queue */
    spiQueue = xQueueCreate(128, sizeof(spi_cmd_t));

    if(spiQueue == NULL)
    {
        return false;
    }

    /* Create event flags */
    eventFlags = xEventGroupCreate();

    if(eventFlags == NULL)
    {
        return false;
    }

    /* Create state machine wait semaphore */
    stateMachineWaitSem = xSemaphoreCreateBinary();

    if(stateMachineWaitSem == NULL)
    {
        return false;
    }

    /* Create tasks */

    if(xTaskCreate(stateMachineTaskFunc, "VS1053-SM", STACK_SIZE, this, 3, &stateMachineTask) != pdPASS)
    {
        return false;
    }

    if(xTaskCreate(dataTaskFunc, "VS1053-DR", STACK_SIZE, this, 2, &dataTask) != pdPASS)
    {
        return false;
    }

    /* Configure data request interrupt */

    LabGPIOInterrupts& gi = LabGPIOInterrupts::getInstance();
    gi.init();

    if(!gi.attachInterruptHandler(dreq.port, dreq.pin, handleDataReqIsr, this, BOTH))
    {
        return false;
    }

    xEventGroupSetBits(eventFlags, EVENT_INIT);

    return true;
}

void VS1053::handleDataReqIsr(void* p)
{
    VS1053* dec = (VS1053*)p;
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    if(dec->dataReq.getLevel())
    {
        if(xEventGroupSetBitsFromISR(dec->eventFlags, EVENT_DREQ_HIGH, &higherPriorityTaskWoken) == pdTRUE)
        {
            /* Yield if necessary */
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        }
    }
    else
    {
        xEventGroupClearBitsFromISR(dec->eventFlags, EVENT_DREQ_HIGH);
    }
}

void VS1053::stateMachineTaskFunc(void* p)
{
    VS1053* dec = (VS1053*)p;

    spi_cmd_t cmd;

    while(1)
    {
        switch(dec->state)
        {
            case INIT:
                if(xEventGroupWaitBits(dec->eventFlags, EVENT_INIT, pdTRUE, pdTRUE, portMAX_DELAY))
                {
                    /* Do a soft reset */
                    if(!controlRegSet(dec, MODE, 0x0002))
                    {
                        break;
                    }

                    waitForDReq(dec);

                    /* Set mode register */
                    if(!controlRegWrite(dec, MODE, 0x0800))
                    {
                        break;
                    }

                    /* Set initial volume to -12dB */
                    if(!setVolumeInternal(dec, 0x00))
                    {
                        break;
                    }

                    /* Set clock control register */
                    if(!controlRegWrite(dec, CLOCKF, 0xc000))
                    {
                        break;
                    }

                    waitForDReq(dec); /* Wait for clock to settle */

                    /* Bump SPI speed up after clock has settled */
                    if(!spi.init(dec->spiDev, 8, LabSPI::IDLE_LOW_CAPTURE_RISING, getSpiDivider(true)))
                    {
                        break;
                    }

                    dec->state = IDLE;
                }
                break;

            case IDLE:
                if(xEventGroupWaitBits(dec->eventFlags, EVENT_NEW_FILE_SELECTED, pdTRUE, pdTRUE, portMAX_DELAY))
                {
                    /* New file was selected by the user */
                    dec->state = PLAYING;
                }
                break;

            case PLAYING:
                if(dec->bufferIndex >= dec->fileBufferLen)
                {
                    dec->bufferIndex = 0;

                    /* Read more file data */
                    if(f_read(&(dec->currentFile), dec->fileBuffer, BUFFER_SIZE, (UINT*)(&(dec->fileBufferLen))) != FR_OK)
                    {
                        f_close(&(dec->currentFile));
                        dec->state = INIT;
                        break;
                    }
                    else if(dec->fileBufferLen == 0)
                    {
                        f_close(&(dec->currentFile));
                        dec->state = ENDING;
                        break;
                    }
                }

                cmd.type = DATA;
                cmd.buffer = &(dec->fileBuffer[dec->bufferIndex]);
                cmd.len = ((dec->fileBufferLen - dec->bufferIndex) > MAX_TRANSCEIVE_SIZE)
                        ? MAX_TRANSCEIVE_SIZE
                                : (dec->fileBufferLen - dec->bufferIndex);
                cmd.notify = dec->stateMachineWaitSem;

                /* Enqueue the data packet */
                xQueueSendToBack(dec->spiQueue, &cmd, portMAX_DELAY);

                dec->bufferIndex += cmd.len;
                break;

            case STOPPING:
                break;

            case ENDING:
                break;

            default:
                dec->state = INIT;
                break;
        }
    }
}

void VS1053::dataTaskFunc(void* p)
{
    VS1053* dec = (VS1053*)p;

    while(1)
    {
        spi_cmd_t cmd;

        if(xQueueReceive(dec->spiQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            waitForDReq(dec);

            /* Command is ready and DREQ is high */

            /* Select chip */
            switch(cmd.type)
            {
                case DATA:
                    dec->dataCs.setLow();
                    break;

                case CMD:
                    dec->controlCs.setLow();
                    break;
            }

            /* Transfer bytes */
            for(uint32_t i = 0; (i < cmd.len) && (i < MAX_TRANSCEIVE_SIZE); i++)
            {
                cmd.buffer[i] = spi.transfer(cmd.buffer[i]);
            }

            /* Deselect chip */
            switch(cmd.type)
            {
                case DATA:
                    dec->dataCs.setHigh();
                    break;

                case CMD:
                    dec->controlCs.setHigh();
                    break;
            }

            /* Notify command requester on completion */
            if(cmd.notify != NULL)
            {
                xSemaphoreGive(cmd.notify);
            }
        }
    }
}

bool VS1053::play(FIL* f)
{
    if(f == NULL)
    {
        return false;
    }

    stop();
    //setPlayType(PLAY); /* Disable FF / Rew, if enabled */

    /* Save the file */
    memcpy(&currentFile, f, sizeof(FIL));

    /* Notify the state machine that the new file is ready */
    xEventGroupSetBits(eventFlags, EVENT_NEW_FILE_SELECTED);

    return true;
}

void VS1053::stop()
{
    if(state == PLAYING)
    {
        xEventGroupSetBits(eventFlags, EVENT_STOP_REQUESTED);
    }
}

bool VS1053::setVolumeInternal(VS1053* dec, uint8_t vol)
{
    return controlRegWrite(dec, VOL, ((uint16_t)(vol << 8)) | ((uint16_t)(vol)));
}

bool VS1053::controlRegRead(VS1053* dec, control_reg_t reg, uint16_t* val)
{
    uint8_t cmd_buffer[4];

    cmd_buffer[0] = 0x03;
    cmd_buffer[1] = (uint8_t)reg;
    cmd_buffer[2] = 0x00;
    cmd_buffer[3] = 0x00;

    spi_cmd_t cmd;

    cmd.type = CMD;
    cmd.buffer = cmd_buffer;
    cmd.len = 4;
    cmd.notify = xSemaphoreCreateBinary();

    if(cmd.notify == NULL)
    {
        return false;
    }

    /* Enqueue command */
    xQueueSendToBack(dec->spiQueue, &cmd, portMAX_DELAY);

    /* Wait for completion */
    xSemaphoreTake(cmd.notify, portMAX_DELAY);

    vSemaphoreDelete(cmd.notify);

    *val = ((((uint16_t)cmd_buffer[2]) << 8) | ((uint16_t)cmd_buffer[3]));

    return true;
}

bool VS1053::controlRegWrite(VS1053* dec, control_reg_t reg, uint16_t val)
{
    uint8_t cmd_buffer[4];

    cmd_buffer[0] = 0x02;
    cmd_buffer[1] = (uint8_t)reg;
    cmd_buffer[2] = (uint8_t)(val >> 8);
    cmd_buffer[3] = (uint8_t)(val);

    spi_cmd_t cmd;

    cmd.type = CMD;
    cmd.buffer = cmd_buffer;
    cmd.len = 4;
    cmd.notify = xSemaphoreCreateBinary();

    if(cmd.notify == NULL)
    {
        return false;
    }

    xQueueSendToBack(dec->spiQueue, &cmd, portMAX_DELAY);

    xSemaphoreTake(cmd.notify, portMAX_DELAY);

    vSemaphoreDelete(cmd.notify);

    return true;
}

bool VS1053::controlRegSet(VS1053* dec, control_reg_t reg, uint16_t bits)
{
    uint16_t val;

    if(!controlRegRead(dec, reg, &val))
    {
        return false;
    }

    val |= bits;

    if(!controlRegWrite(dec, reg, val))
    {
        return false;
    }

    return true;
}

bool VS1053::controlRegClear(VS1053* dec, control_reg_t reg, uint16_t bits)
{
    uint16_t val;

    if(!controlRegRead(dec, reg, &val))
    {
        return false;
    }

    val &= ~bits;

    if(!controlRegWrite(dec, reg, val))
    {
        return false;
    }

    return true;
}

void VS1053::waitForDReq(VS1053* dec) {
    if(!(dec->dataReq.getLevel()))
    {
        xEventGroupWaitBits(dec->eventFlags, EVENT_DREQ_HIGH, pdFALSE, pdTRUE, portMAX_DELAY);
    }
}

uint8_t VS1053::getSpiDivider(bool speed)
{
    uint8_t res = (uint8_t)(sys_get_cpu_clock() / (speed ? SPI_FREQ_HZ : SPI_START_FREQ_HZ));
    if(res & 1)
    {
        res += 1;
    }

    return res;
}
