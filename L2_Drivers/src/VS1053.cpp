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

#define EVENT_NEW_FILE_SELECTED  (1 << 0)
#define EVENT_DREQ_HIGH          (1 << 1)
#define EVENT_STOP_REQUESTED     (1 << 2)

#define SM_CANCEL                (1 << 3)

VS1053::VS1053() {}
VS1053::~VS1053() {}

bool VS1053::init(LabSPI::Peripheral spi_channel, pin_t& reset, pin_t& data_cs, pin_t& control_cs, pin_t& dreq)
{
    spiDev = spi_channel;

    /* SPI init with low startup speed */
    if(!spi.init(spiDev, 8, LabSPI::IDLE_LOW_CAPTURE_RISING, getSpiDivider(false)))
    {
        return false;
    }

    /* Configure GPIO pins */

    if(!resetPin.init(reset.port, reset.pin))
    {
        return false;
    }

    resetPin.setAsOutput();
    resetPin.setLow();

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

    /* Create event flags */
    eventFlags = xEventGroupCreate();

    if(eventFlags == NULL)
    {
        return false;
    }

    /* Create tasks */
    if(xTaskCreate(workerTaskFunc, "VS1053", STACK_SIZE, this, 2, &workerTask) != pdPASS)
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

    state = HW_RESET;

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

void VS1053::workerTaskFunc(void* p)
{
    VS1053* dec = (VS1053*)p;

    bool eof;
    uint8_t end_fill_byte;
    uint32_t i, j; /* Loop counters */

    spi_cmd_t cmd;

    while(1)
    {
        switch(dec->state)
        {
            case HW_RESET:
                dec->resetPin.setLow();
                vTaskDelay(5);
                dec->resetPin.setHigh();

                dec->state = INIT;
                break;

            case SW_RESET:
                controlRegSet(dec, MODE, 0x0002); /* Do a soft reset */
                break;

            case INIT:
                controlRegWrite(dec, MODE, 0x0800, true); /* Set mode register */
                setVolumeInternal(dec, 0x18); /* Set initial volume to -12dB */

                controlRegWrite(dec, CLOCKF, 0xc000, true); /* Set clock control register */
                waitForDReq(dec); /* Wait for clock to settle */

                /* Bump SPI speed up after clock has settled */
                if(!spi.init(dec->spiDev, 8, LabSPI::IDLE_LOW_CAPTURE_RISING, getSpiDivider(true)))
                {
                    break;
                }

                dec->state = IDLE;
                break;

            case IDLE:
                if(xEventGroupWaitBits(dec->eventFlags, EVENT_NEW_FILE_SELECTED, pdTRUE, pdTRUE, portMAX_DELAY))
                {
                    /* New file was selected by the user */
                    dec->bufferIndex  = 0;
                    dec->fileBufferLen = 0;
                    dec->state = PLAYING;
                }
                break;

            case PLAYING:
                if(xEventGroupGetBits(dec->eventFlags) & EVENT_STOP_REQUESTED)
                {
                    xEventGroupClearBits(dec->eventFlags, EVENT_STOP_REQUESTED);

                    /* Set SM_CANCEL bit in MODE register */
                    controlRegSet(dec, MODE, SM_CANCEL);

                    end_fill_byte = getEndFillByte(dec);

                    dec->state = STOPPING;
                    break;
                }

                if(!sendNextDataPacket(dec, &eof))
                {
                    dec->state = SW_RESET;
                }
                else if(eof)
                {
                    controlRegSet(dec, MODE, SM_CANCEL);
                    end_fill_byte = getEndFillByte(dec);
                    dec->state = ENDING;
                }
                break;

            case STOPPING:
                /* Send 2048 more bytes of file and wait for SM_CANCEL to clear */
                for(i = 0; (i < 64) && (controlRegRead(dec, MODE, true) & SM_CANCEL); i++)
                {
                    if(!sendNextDataPacket(dec, &eof))
                    {
                        dec->state = SW_RESET;
                    }
                    else if(eof)
                    {
                        dec->state = ENDING;
                    }
                }

                if(i == 64)
                {
                    dec->state = SW_RESET; /* SM_CANCEL stuck, do reset */
                }
                else if(dec->state == STOPPING) /* Ensure that we're still stopping */
                {
                    cmd.type = DATA;
                    cmd.buffer = &end_fill_byte;
                    cmd.len = 1;

                    xSemaphoreTake(LabSPI::bus_lock, portMAX_DELAY);

                    /* Send 2052 of endFillByte */
                    for(i = 0; i < 2052; i++)
                    {
                        transceive(dec, &cmd);
                    }

                    xSemaphoreGive(LabSPI::bus_lock);

                    dec->state = IDLE; /* Stopped, return to idle */
                }
                break;

            case ENDING:
                cmd.type = DATA;
                cmd.buffer = &end_fill_byte;
                cmd.len = 1;

                /* Send 2048 of endFillByte (32 at a time) and wait for SM_CANCEL to clear */
                for(i = 0; (i < 64) && (controlRegRead(dec, MODE, true) & SM_CANCEL); i++)
                {
                    xSemaphoreTake(LabSPI::bus_lock, portMAX_DELAY);

                    for(j = 0; j < 32; j++)
                    {
                        transceive(dec, &cmd);
                    }

                    xSemaphoreGive(LabSPI::bus_lock);
                }

                if(i == 64)
                {
                    dec->state = SW_RESET; /* SM_CANCEL stuck, do reset */
                }
                else
                {
                    dec->state = IDLE; /* Done, return to idle */
                }
                break;

            default:
                dec->state = SW_RESET;
                break;
        }
    }
}

bool VS1053::play(const char* path)
{
    if(path == NULL)
    {
        return false;
    }

    stop();
    //setPlayType(PLAY); /* Disable FF / Rew, if enabled */

    /* Save the file */
    if(f_open(&currentFile, path, FA_READ) != FR_OK)
    {
        return false;
    }

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

void VS1053::setVolumeInternal(VS1053* dec, uint8_t vol)
{
    controlRegWrite(dec, VOL, ((uint16_t)(vol << 8)) | ((uint16_t)(vol)), true);
}

uint16_t VS1053::controlRegRead(VS1053* dec, control_reg_t reg, bool acquireBus)
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

    if(acquireBus)
    {
        xSemaphoreTake(LabSPI::bus_lock, portMAX_DELAY);
    }

    transceive(dec, &cmd);

    if(acquireBus)
    {
        xSemaphoreGive(LabSPI::bus_lock);
    }

    return ((((uint16_t)cmd_buffer[2]) << 8) | ((uint16_t)cmd_buffer[3]));
}

void VS1053::controlRegWrite(VS1053* dec, control_reg_t reg, uint16_t val, bool acquireBus)
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

    if(acquireBus)
    {
        xSemaphoreTake(LabSPI::bus_lock, portMAX_DELAY);
    }

    transceive(dec, &cmd);

    if(acquireBus)
    {
        xSemaphoreGive(LabSPI::bus_lock);
    }
}

void VS1053::controlRegSet(VS1053* dec, control_reg_t reg, uint16_t bits)
{
    uint16_t val;

    xSemaphoreTake(LabSPI::bus_lock, portMAX_DELAY);

    val = controlRegRead(dec, reg, false);

    val |= bits;

    controlRegWrite(dec, reg, val, false);

    xSemaphoreGive(LabSPI::bus_lock);
}

void VS1053::controlRegClear(VS1053* dec, control_reg_t reg, uint16_t bits)
{
    uint16_t val;

    xSemaphoreTake(LabSPI::bus_lock, portMAX_DELAY);

    val = controlRegRead(dec, reg, false);

    val &= ~bits;

    controlRegWrite(dec, reg, val, false);

    xSemaphoreGive(LabSPI::bus_lock);
}

/* SPI bus lock must be acquired before entering this function */
void VS1053::transceive(VS1053* dec, spi_cmd_t* cmd)
{
    waitForDReq(dec);

    /* Select chip */
    switch(cmd->type)
    {
        case DATA:
            dec->dataCs.setLow();
            break;

        case CMD:
            dec->controlCs.setLow();
            break;
    }

    /* Transfer bytes */
    for(uint32_t i = 0; i < cmd->len; i++)
    {
        cmd->buffer[i] = spi.transfer(cmd->buffer[i]);
    }

    /* Deselect chip */
    switch(cmd->type)
    {
        case DATA:
            dec->dataCs.setHigh();
            break;

        case CMD:
            dec->controlCs.setHigh();
            break;
    }
}

void VS1053::waitForDReq(VS1053* dec) {
    if(!(dec->dataReq.getLevel()))
    {
        xEventGroupWaitBits(dec->eventFlags, EVENT_DREQ_HIGH, pdTRUE, pdTRUE, portMAX_DELAY);
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

bool VS1053::sendNextDataPacket(VS1053* dec, bool* eof)
{
    *eof = false;

    if(dec->bufferIndex >= dec->fileBufferLen)
    {
        dec->bufferIndex = 0;

        /* Read more file data */
        if(f_read(&(dec->currentFile), dec->fileBuffer, BUFFER_SIZE, (UINT*)(&(dec->fileBufferLen))) != FR_OK)
        {
            f_close(&(dec->currentFile));
            return false;
        }
        else if(dec->fileBufferLen == 0)
        {
            f_close(&(dec->currentFile));
            *eof = true;
            return true;
        }
    }

    spi_cmd_t cmd;

    cmd.type = DATA;
    cmd.buffer = &(dec->fileBuffer[dec->bufferIndex]);
    cmd.len = ((dec->fileBufferLen - dec->bufferIndex) > MAX_TRANSCEIVE_SIZE)
            ? MAX_TRANSCEIVE_SIZE
                    : (dec->fileBufferLen - dec->bufferIndex);

    xSemaphoreTake(LabSPI::bus_lock, portMAX_DELAY);

    transceive(dec, &cmd);

    xSemaphoreGive(LabSPI::bus_lock);

    dec->bufferIndex += cmd.len;

    return true;
}

uint8_t VS1053::getEndFillByte(VS1053* dec)
{
    xSemaphoreTake(LabSPI::bus_lock, portMAX_DELAY);

    /* Write address of endFillByte to WRAMADDR */
    controlRegWrite(dec, WRAMADDR, 0x1e06, false);

    /* Read value at address */
    uint16_t val = controlRegRead(dec, WRAM, false);

    xSemaphoreGive(LabSPI::bus_lock);

    return (uint8_t)val;
}
