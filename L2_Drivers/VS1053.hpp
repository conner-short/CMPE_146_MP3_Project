#ifndef __VS1053_HPP
#define __VS1053_HPP

#include <stdint.h>

#include "FreeRTOS.h"

#include "event_groups.h"
#include "ff.h"
#include "LabGPIO.hpp"
#include "LabSPI.hpp"
#include "semphr.h"

class VS1053
{
public:
    static const uint32_t MAX_TRANSCEIVE_SIZE = 32; /* From datasheet */

    VS1053();
    ~VS1053();

    typedef enum {PLAY, FF, REW} PLAY_TYPE;

    typedef struct
    {
        uint8_t port, pin;
    } pin_t;

    bool init(LabSPI::Peripheral spiChannel, pin_t& data_cs, pin_t& control_cs, pin_t& dreq);
    bool play(FIL* f);
    void pause(void);
    void stop(void);
    void setPlayType(PLAY_TYPE t);
    bool setVolume(uint8_t vol);

private:
    static const uint32_t SPI_FREQ_HZ = 7899000;
    static const uint32_t SPI_START_FREQ_HZ = 1500000;
    static const uint32_t STACK_SIZE = 1024;
    static const uint32_t BUFFER_SIZE = 16384;

    /* From datasheet */
    typedef enum
    {
        MODE        = 0x00,
        STATUS      = 0x01,
        BASS        = 0x02,
        CLOCKF      = 0x03,
        DECODE_TIME = 0x04,
        AUDATA      = 0x05,
        WRAM        = 0x06,
        WRAMADDR    = 0x07,
        HDAT0       = 0x08,
        HDAT1       = 0x09,
        AIADDR      = 0x0a,
        VOL         = 0x0b,
        AICTRL0     = 0x0c,
        AICTRL1     = 0x0d,
        AICTRL2     = 0x0e,
        AICTRL3     = 0x0f
    } control_reg_t;

    typedef enum
    {
        INIT,
        IDLE,
        PLAYING,
        STOPPING,
        ENDING
    } state_t;

    typedef enum
    {
        DATA,
        CMD
    } spi_cmd_type_t;

    typedef struct
    {
        spi_cmd_type_t type;
        uint8_t* buffer;
		uint32_t len;
		SemaphoreHandle_t notify;
    } spi_cmd_t;

    state_t state = INIT;

    LabSPI::Peripheral spiDev = LabSPI::SSP0;

    /* Pins */
    LabGPIO dataCs;
    LabGPIO controlCs;
    LabGPIO dataReq;

    PLAY_TYPE playType = PLAY; /* Play, FF, or rewind */

    TaskHandle_t stateMachineTask = NULL;
    TaskHandle_t dataTask = NULL;

    FIL currentFile;
    uint8_t fileBuffer[BUFFER_SIZE];
    uint32_t fileBufferLen = 0;
    uint32_t bufferIndex = 0;

    QueueHandle_t spiQueue = NULL;
    EventGroupHandle_t eventFlags = NULL;
    SemaphoreHandle_t stateMachineWaitSem = NULL;

    static void handleDataReqIsr(void* p);

    static void stateMachineTaskFunc(void* p);
    static void dataTaskFunc(void* p);

    static bool controlRegRead(VS1053* dec, control_reg_t reg, uint16_t* val);
    static bool controlRegWrite(VS1053* dec, control_reg_t reg, uint16_t val);
    static bool controlRegSet(VS1053* dec, control_reg_t reg, uint16_t bits);
    static bool controlRegClear(VS1053* dec, control_reg_t reg, uint16_t bits);

    static bool setVolumeInternal(VS1053* dec, uint8_t vol);

    static void waitForDReq(VS1053* dec);

    static uint8_t getSpiDivider(bool speed);
};

#endif
