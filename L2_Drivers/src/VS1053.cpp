#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"

#include "semphr.h"
#include "sys_config.h"
#include "task.h"

#include "LabGPIO.hpp"
#include "LabGPIOInterrupts.hpp"
#include "VS1053.hpp"

#define EVENT_NEW_FILE_SELECTED  (1 << 0)
#define EVENT_DREQ_HIGH          (1 << 1)
#define EVENT_STOP               (1 << 2)
#define EVENT_PAUSE              (1 << 4)
#define EVENT_RESUME             (1 << 5)

#define SM_CANCEL                (1 << 3)
#define SS_DO_NOT_JUMP           (1 << 15)

VS1053::VS1053() {}
VS1053::~VS1053() {}

bool VS1053::init(SPIController::ssp_t ssp, pin_t& reset, pin_t& data_cs, pin_t& control_cs, pin_t& dreq)
{
    mSSP = ssp;

    /* Init SPI to slow startup speed */
    if(!SPIController::getInstance().init(mSSP, SPIController::IDLE_LOW_CAPTURE_RISING, SPI_START_FREQ_HZ))
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

    if(!dataReq.init(dreq.port, dreq.pin))
    {
        return false;
    }

    dataReq.setAsInput();

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
    uint32_t i; /* Loop counter */

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

                dec->state = INIT;
                break;

            case INIT:
                controlRegWrite(dec, MODE, 0x0800, true); /* Set mode register */
                setVolumeInternal(dec, 0x18); /* Set initial volume to -12dB */

                controlRegWrite(dec, CLOCKF, 0xc000, true); /* Set clock control register */
                waitForDReq(dec); /* Wait for clock to settle */

                SPIController::getInstance().setSckFreq(dec->mSSP, SPI_FREQ_HZ);

                dec->state = IDLE;
                break;

            case IDLE:
                if(xEventGroupWaitBits(dec->eventFlags, EVENT_NEW_FILE_SELECTED, pdTRUE, pdTRUE, portMAX_DELAY))
                {
                    /* New file was selected, reset variables */

                    dec->bufferIndex  = 0;
                    dec->bufferLen = 0;
                    dec->fileReadBase = 0;
                    dec->fileReadOffset = 0;
                    dec->bytesToSend = 0;
                    dec->paused = false;

                    xEventGroupClearBits(dec->eventFlags, EVENT_PAUSE | EVENT_RESUME);

                    dec->state = PLAYING;
                }
                break;

            case PLAYING:
                if(xEventGroupGetBits(dec->eventFlags) & EVENT_STOP)
                {
                    xEventGroupClearBits(dec->eventFlags, EVENT_STOP);

                    /* Set SM_CANCEL bit in MODE register */
                    controlRegSet(dec, MODE, SM_CANCEL);

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
                    sendEndFillByte(dec, 2052); /* Send 2052 of endFillByte */
                    dec->state = IDLE; /* Stopped, return to idle */
                }
                break;

            case ENDING:
                /* Send 2048 of endFillByte (32 at a time) and wait for SM_CANCEL to clear */
                for(i = 0; (i < 64) && (controlRegRead(dec, MODE, true) & SM_CANCEL); i++)
                {
                    sendEndFillByte(dec, 32);
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
    setPlayType(PLAY); /* Disable FF / Rew, if enabled */

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
        xEventGroupSetBits(eventFlags, EVENT_STOP);
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

    SPIController& spi = SPIController::getInstance();

    if(acquireBus)
    {
        spi.acquire(dec->mSSP);
    }

    waitForDReq(dec);

    dec->controlCs.setLow();
    spi.transceive(dec->mSSP, cmd_buffer, 4);
    dec->controlCs.setHigh();

    if(acquireBus)
    {
        spi.release(dec->mSSP);
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

    SPIController& spi = SPIController::getInstance();

    if(acquireBus)
    {
        spi.acquire(dec->mSSP);
    }

    waitForDReq(dec);

    dec->controlCs.setLow();
    spi.transceive(dec->mSSP, cmd_buffer, 4);
    dec->controlCs.setHigh();

    if(acquireBus)
    {
        spi.release(dec->mSSP);
    }
}

void VS1053::controlRegSet(VS1053* dec, control_reg_t reg, uint16_t bits)
{
    uint16_t val;
    SPIController& spi = SPIController::getInstance();

    spi.acquire(dec->mSSP);

    val = controlRegRead(dec, reg, false);

    val |= bits;

    controlRegWrite(dec, reg, val, false);

    spi.release(dec->mSSP);
}

void VS1053::controlRegClear(VS1053* dec, control_reg_t reg, uint16_t bits)
{
    uint16_t val;
    SPIController& spi = SPIController::getInstance();

    spi.acquire(dec->mSSP);

    val = controlRegRead(dec, reg, false);

    val &= ~bits;

    controlRegWrite(dec, reg, val, false);

    spi.release(dec->mSSP);
}

void VS1053::waitForDReq(VS1053* dec) {
    if(!(dec->dataReq.getLevel()))
    {
        xEventGroupWaitBits(dec->eventFlags, EVENT_DREQ_HIGH, pdTRUE, pdTRUE, portMAX_DELAY);
    }
}

bool VS1053::sendNextDataPacket(VS1053* dec, bool* eof)
{
    if(xEventGroupGetBits(dec->eventFlags) & EVENT_PAUSE)
    {
        xEventGroupClearBits(dec->eventFlags, EVENT_PAUSE);

        dec->paused = true;

        EventBits_t bits = xEventGroupWaitBits(dec->eventFlags,
                EVENT_RESUME | EVENT_STOP, pdTRUE, pdFALSE, portMAX_DELAY);

        dec->paused = false;

        if(bits & EVENT_STOP)
        {
            /* Don't play anything else; just return so the stop handling can begin */
            return true;
        }
    }

    *eof = false;

    if(dec->currentPlayType != dec->requestedPlayType)
    {
        if((dec->requestedPlayType == FF) || (dec->requestedPlayType == REW))
        {
            /* Check SS_DO_NOT_JUMP */
            uint16_t status = controlRegRead(dec, STATUS, true);

            if(!(status & SS_DO_NOT_JUMP))
            {
                dec->currentPlayType = dec->requestedPlayType;
            }
        }
        else
        {
            dec->currentPlayType = dec->requestedPlayType;
        }
    }

    /* Do mode-specific operations */
    switch(dec->currentPlayType)
    {
        case PLAY:
            dec->fileReadBase += dec->fileReadOffset;
            dec->fileReadOffset = 0;
            break;

        case FF:
            if(dec->bytesToSend == 0)
            {
                /* Update byteRate */
                dec->byteRate = getByteRate(dec);

                /* Adjust read base */
                dec->fileReadBase += dec->byteRate;
                dec->fileReadOffset = 0;

                /* Send 2048 bytes of endFillByte to let the decoder prepare to skip
                 * around */
                sendEndFillByte(dec, 2048);

                /* Send 1/SEEK_SPEED seconds of audio to the decoder */
                dec->bytesToSend = (uint32_t)(dec->byteRate) / SEEK_SPEED;
            }
            break;

        case REW:
            if(dec->bytesToSend == 0)
            {
                /* Pretty much the same as fast-forwarding, just in reverse */

                dec->byteRate = getByteRate(dec);

                /* Mind the potential subtraction underflow */
                dec->fileReadBase -= (dec->fileReadBase > dec->byteRate) ? dec->byteRate : 0;
                dec->fileReadOffset = 0;

                sendEndFillByte(dec, 2048);

                dec->bytesToSend = (uint32_t)(dec->byteRate) / SEEK_SPEED;
            }
            break;

        default:
            return false;
    }

    if(dec->bufferIndex >= dec->bufferLen)
    {
        dec->bufferIndex = 0;

        /* Reposition read head */
        if(f_lseek(&(dec->currentFile), dec->fileReadBase + dec->fileReadOffset) != FR_OK)
        {
            return false;
        }

        /* Refill buffer */
        if(f_read(&(dec->currentFile), dec->fileBuffer, BUFFER_SIZE, (UINT*)(&(dec->bufferLen))) != FR_OK)
        {
            return false;
        }
        else if(dec->bufferLen == 0)
        {
            *eof = true;

            return true;
        }
    }

    uint8_t* buf = &(dec->fileBuffer[dec->bufferIndex]);
    uint32_t len = ((dec->bufferLen - dec->bufferIndex) > MAX_TRANSCEIVE_SIZE) ?
            MAX_TRANSCEIVE_SIZE : dec->bufferLen - dec->bufferIndex;

    SPIController& spi = SPIController::getInstance();

    spi.acquire(dec->mSSP);

    waitForDReq(dec);

    dec->dataCs.setLow();
    spi.transceive(dec->mSSP, buf, len);
    dec->dataCs.setHigh();

    spi.release(dec->mSSP);

    /* Update indexes */
    dec->bufferIndex += len;
    dec->fileReadOffset += len;
    dec->bytesToSend -= (dec->bytesToSend > len) ? len : dec->bytesToSend;

    return true;
}

uint8_t VS1053::getEndFillByte(VS1053* dec)
{
    SPIController& spi = SPIController::getInstance();

    spi.acquire(dec->mSSP);

    /* Write address of endFillByte to WRAMADDR */
    controlRegWrite(dec, WRAMADDR, 0x1e06, false);

    /* Read value at address */
    uint16_t val = controlRegRead(dec, WRAM, false);

    spi.release(dec->mSSP);

    return (uint8_t)val;
}

void VS1053::setPlayType(PLAY_TYPE t)
{
    requestedPlayType = t;
}

uint16_t VS1053::getByteRate(VS1053* dec)
{
    SPIController& spi = SPIController::getInstance();

    spi.acquire(dec->mSSP);

    /* Write address of byteRate to WRAMADDR */
    controlRegWrite(dec, WRAMADDR, 0x1e05, false);

    /* Read value at address */
    uint16_t val = controlRegRead(dec, WRAM, false);

    spi.release(dec->mSSP);

    return val;
}

void VS1053::pause(void)
{
    if(!paused)
    {
        xEventGroupSetBits(eventFlags, EVENT_PAUSE);
    }
}

void VS1053::resume(void)
{
    if(paused)
    {
        xEventGroupSetBits(eventFlags, EVENT_RESUME);
    }
}

void VS1053::sendEndFillByte(VS1053* dec, uint32_t count)
{
    uint8_t endFillByte = getEndFillByte(dec);
    uint8_t efbBuffer[4];
    efbBuffer[0] = endFillByte;
    efbBuffer[1] = endFillByte;
    efbBuffer[2] = endFillByte;
    efbBuffer[3] = endFillByte;

    SPIController& spi = SPIController::getInstance();

    spi.acquire(dec->mSSP);

    /* Send 4 at a time until there's less than 4 remaining */
    for(; count >= 4; count -= 4)
    {
        waitForDReq(dec);

        dec->dataCs.setLow();
        spi.transceive(dec->mSSP, efbBuffer, 4);
        dec->dataCs.setHigh();
    }

    /* Send the remaining bytes */
    for(; count > 0; count--)
    {
        waitForDReq(dec);

        dec->dataCs.setLow();
        spi.transceive(dec->mSSP, &endFillByte, 1);
        dec->dataCs.setHigh();
    }

    spi.release(dec->mSSP);
}
