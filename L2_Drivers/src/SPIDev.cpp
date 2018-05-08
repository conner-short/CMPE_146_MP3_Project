#include <stdio.h>

#include "SPIDev.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

SPIDev::SPIDev(void) {}

bool SPIDev::init(SPIController::ssp_t ssp, pin_t cs_pin, SPIController::frame_format_t ff, uint32_t sck_freq_hz)
{
    m_ssp = ssp;

    if(!cs.init(cs_pin.port, cs_pin.pin))
    {
        return false;
    }

    cs.setAsOutput();
    cs.setHigh();

    if(!SPIController::getInstance().init(ssp, ff, sck_freq_hz))
    {
        return false;
    }

    mutex = xSemaphoreCreateMutex();
    spi_sync = xSemaphoreCreateBinary();

    return ((mutex != NULL) && (spi_sync != NULL)) ? true : false;
}

void SPIDev::acquire(void) {xSemaphoreTake(mutex, portMAX_DELAY);}
void SPIDev::release(void) {xSemaphoreGive(mutex);}

void SPIDev::transceive(uint8_t* buf, uint32_t len)
{
    if((buf != NULL) && (xSemaphoreGetMutexHolder(mutex) == xTaskGetCurrentTaskHandle()))
    {
        SPIController::getInstance().acquire(m_ssp);
        cs.setLow();
        SPIController::getInstance().transceive(m_ssp, buf, len, spi_sync);
        xSemaphoreTake(spi_sync, portMAX_DELAY);
        cs.setHigh();
        SPIController::getInstance().release(m_ssp);
    }
}

bool SPIDev::setSckFreq(uint32_t sck_freq_hz)
{
    return SPIController::getInstance().setSckFreq(m_ssp, sck_freq_hz);
}
