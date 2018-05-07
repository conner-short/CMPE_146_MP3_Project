#ifndef __SPI_CONTROLLER_HPP
#define __SPI_CONTROLLER_HPP

#include <stddef.h>
#include <stdint.h>

#include "LPC17xx.h"

#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

class SPIController
{
public:
    typedef enum {SSP0, SSP1} ssp_t;
    typedef enum
    {
        IDLE_LOW_CAPTURE_RISING,
        IDLE_LOW_CAPTURE_FALLING,
        IDLE_HIGH_CAPTURE_FALLING,
        IDLE_HIGH_CAPTURE_RISING
    } frame_format_t;

    static SPIController& getInstance(void);

    bool init(ssp_t ssp, frame_format_t ff, uint32_t sck_freq_hz);
    void acquire(ssp_t ssp);
    void release(ssp_t ssp);
    void transceive(ssp_t ssp, uint8_t* buf, uint32_t len, SemaphoreHandle_t sem);

private:
    SPIController(void);
    SPIController(SPIController const&) {};

    typedef struct
    {
        uint8_t* buf = NULL;
        uint32_t len = 0;
        SemaphoreHandle_t sem = NULL;
    } spi_msg_t;

    typedef struct
    {
        TaskHandle_t task = NULL;
        QueueHandle_t queue = NULL;
        EventGroupHandle_t ev = NULL;
        SemaphoreHandle_t mutex = NULL;
        volatile LPC_SSP_TypeDef* ssp;
        spi_msg_t msg;
        uint32_t tx_i = 0;
        uint32_t rx_i = 0;
    } task_state_t;

    static const unsigned short STACK_DEPTH = 1024;

    static const EventBits_t EVENT_HW_READY = 0x1;

    static task_state_t ssp0_state, ssp1_state;

    bool calcDividerSettings(uint32_t pclk_hz, uint32_t sck_freq_hz, uint8_t* cpsdvsr, uint8_t* scr);

    static void controllerTask(task_state_t* s);
    static void SSP0ControllerTask(void* p);
    static void SSP1ControllerTask(void* p);

    static void SSP0ISR(void);
    static void SSP1ISR(void);
    static BaseType_t ISR(task_state_t* state);

    static void disableTxInterrupts(volatile LPC_SSP_TypeDef* ssp);
    static void disableRxInterrupts(volatile LPC_SSP_TypeDef* ssp);
    static void enableTxInterrupts(volatile LPC_SSP_TypeDef* ssp);
    static void enableRxInterrupts(volatile LPC_SSP_TypeDef* ssp);
};

#endif
