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

    typedef enum {PLAY, PAUSE, FF, REW} PLAY_TYPE;

    typedef struct
    {
        uint8_t port, pin;
    } pin_t;

    /**
     * Initializes the VS1053 decoder
     *
     * @param spi_channel  The SPI device the decoder is connected to
     * @param reset        Port and pin numbers of the reset pin (XRESET)
     * @param data_cs      Port and pin numbers of the data chip select (XDCS)
     * @param control_cs   Port and pin numbers of the control chip select (XCS)
     * @param dreq         Port and pin numbers of the data request pin (DREQ)
     *
     * @return Retuns true on success, false otherwise
     */
    bool init(LabSPI::Peripheral spi_channel, pin_t& reset, pin_t& data_cs, pin_t& control_cs, pin_t& dreq);

    /**
     * Loads a file and begins playing it
     *
     * @param path  Path to the file
     *
     * @return Returns true on success, false otherwise
     */
    bool play(const char* path);

    /**
     * Stops the current file
     */
    void stop(void);

    /**
     * NOTE: NOT IMPLEMENTED YET
     * Selects between normal playback, fast-forward, and rewind
     *
     * @param t  Playback type
     */
    void setPlayType(PLAY_TYPE t);

    /**
     * Sets the playback volume
     *
     * @param vol  Offset from full volume in -0.5dB increments; 0x00 is full volume,
     *             0xfe is silence, and 0xff powers off the amplifier
     *
     * @return Returns true on success, false otherwise
     */
    bool setVolume(uint8_t vol);

private:
    static const uint32_t SPI_FREQ_HZ = 7899000;
    static const uint32_t SPI_START_FREQ_HZ = 1500000;
    static const uint32_t STACK_SIZE = 1024;
    static const uint32_t BUFFER_SIZE = 8192;

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
        HW_RESET,
        SW_RESET,
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
    } spi_cmd_t;

    state_t state = INIT;

    LabSPI::Peripheral spiDev = LabSPI::SSP0;

    /* Pins */
    LabGPIO resetPin;
    LabGPIO dataCs;
    LabGPIO controlCs;
    LabGPIO dataReq;

    PLAY_TYPE playType = PLAY; /* Play, FF, or rewind */

    TaskHandle_t workerTask = NULL;

    FIL currentFile;
    uint8_t fileBuffer[BUFFER_SIZE];
    uint32_t fileBufferLen = 0;
    uint32_t bufferIndex = 0;

    EventGroupHandle_t eventFlags = NULL;

    static void handleDataReqIsr(void* p);

    static void workerTaskFunc(void* p);
    static void wdtTaskFunc(void* p);

    static uint16_t controlRegRead(VS1053* dec, control_reg_t reg, bool acquireBus);
    static void controlRegWrite(VS1053* dec, control_reg_t reg, uint16_t val, bool acquireBus);
    static void controlRegSet(VS1053* dec, control_reg_t reg, uint16_t bits);
    static void controlRegClear(VS1053* dec, control_reg_t reg, uint16_t bits);

    static void setVolumeInternal(VS1053* dec, uint8_t vol);

    static void waitForDReq(VS1053* dec);
    static void transceive(VS1053* dec, spi_cmd_t* cmd);

    static uint8_t getSpiDivider(bool speed);

    static bool sendNextDataPacket(VS1053* dec, bool* eof);

    static uint8_t getEndFillByte(VS1053* dec);
};

#endif
