#ifndef __SPI_DEV_HPP
#define __SPI_DEV_HPP

#include <stddef.h>

#include "LabGPIO.hpp"
#include "SPIController.hpp"

class SPIDev
{
public:
    SPIDev(void);

    bool init(SPIController::ssp_t ssp, uint8_t cs_gpio_port, uint8_t cs_gpio_pin,
            SPIController::frame_format_t ff, uint32_t sck_freq_hz);
    void acquire(void);
    void release(void);
    void transceive(uint8_t* buf, uint32_t len, SemaphoreHandle_t sem);

private:
    SPIController::ssp_t m_ssp = SPIController::SSP0;
    SemaphoreHandle_t mutex = NULL;
    LabGPIO cs;
};

#endif
