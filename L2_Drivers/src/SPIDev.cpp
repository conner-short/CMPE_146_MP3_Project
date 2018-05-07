#include "SPIDev.hpp"

#include "FreeRTOS.h"

SPIDev::SPIDev(void) {}

bool SPIDev::init(SPIController::ssp_t ssp, uint8_t cs_gpio_port, uint8_t cs_gpio_pin,
        SPIController::frame_format_t ff, uint32_t sck_freq_hz)
{
    m_ssp = ssp;

    if(!cs.init(cs_gpio_port, cs_gpio_pin))
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

    return (mutex == NULL) ? false : true;
}

void SPIDev::acquire(void) {xSemaphoreTake(mutex, portMAX_DELAY);}
void SPIDev::release(void) {xSemaphoreGive(mutex);}

void SPIDev::transceive(uint8_t* buf, uint32_t len, SemaphoreHandle_t sem)
{
    if((buf != NULL) && (xSemaphoreGetMutexHolder(mutex) == xTaskGetCurrentTaskHandle()))
    {
        SPIController::getInstance().acquire(m_ssp);
        cs.setLow();
        SPIController::getInstance().transceive(m_ssp, buf, len, sem);
        cs.setHigh();
        SPIController::getInstance().release(m_ssp);
    }
}
