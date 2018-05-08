#ifndef __SPI_DEV_HPP
#define __SPI_DEV_HPP

#include <stddef.h>

#include "LabGPIO.hpp"
#include "pin_t.hpp"
#include "SPIController.hpp"

class SPIDev
{
public:
    SPIDev(void);

    bool init(SPIController::ssp_t ssp, pin_t cs_pin, SPIController::frame_format_t ff, uint32_t sck_freq_hz);
    bool setSckFreq(uint32_t sck_freq_hz);
    void acquire(void);
    void release(void);
    void transceive(uint8_t* buf, uint32_t len);

private:
    SPIController::ssp_t m_ssp = SPIController::SSP0;
    SemaphoreHandle_t mutex = NULL, spi_sync = NULL;
    LabGPIO cs;
};

#endif
