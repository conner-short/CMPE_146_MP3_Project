#include "core_cm3.h"
#include "lpc_isr.h"
#include "SPIController.hpp"
#include "sys_config.h"

SPIController::SPIController(void)
{
    ssp0_state.ssp = LPC_SSP0;
    ssp1_state.ssp = LPC_SSP1;
}

SPIController& SPIController::getInstance(void)
{
    static SPIController instance;
    return instance;
}

bool SPIController::calcDividerSettings(uint32_t pclk_hz, uint32_t sck_freq_hz, uint8_t* cpsdvsr, uint8_t* scr)
{
    /* Validate parameters */

    if((pclk_hz == 0) || (sck_freq_hz == 0))
    {
        return false;
    }

    uint32_t ratio = pclk_hz / sck_freq_hz;

    if((ratio > (254 * 255)) || (ratio < (2 * 1)))
    {
        return false;
    }

    uint32_t min_err = 0xFFFFFFFF;
    uint32_t _cpsdvsr = 2, _scr_plus_1 = 1;

    /* Search for divider parameters which minimize error w.r.t. desired
     * frequency */
    for(uint32_t i = 2; i < 256; i += 2)
    {
        for(uint32_t j = 1; j < 256; j++)
        {
            uint32_t sck_gen_hz = pclk_hz / (i * j);

            /* Get absolute value of error */
            uint32_t err = (sck_gen_hz > sck_freq_hz) ? sck_gen_hz - sck_freq_hz :
                    sck_freq_hz - sck_gen_hz;

            if(err < min_err)
            {
               min_err = err;
               _cpsdvsr = i;
               _scr_plus_1 = j;
            }
            if(err == 0)
            {
                /* If an exact match was found, return it immediately */
                *cpsdvsr = (uint8_t)_cpsdvsr;
                *scr = ((uint8_t)_scr_plus_1) - 1;
                return true;
            }
        }
    }

    /* Otherwise, return the closest match possible */
    *cpsdvsr = _cpsdvsr;
    *scr = _scr_plus_1 - 1;

    return true;
}

bool SPIController::init(ssp_t ssp, frame_format_t ff, uint32_t sck_freq_hz)
{
    /* Configure CR0 */

    uint32_t cr0 = 0x0007; /* 8-bit frame size */

    switch(ff)
    {
        case IDLE_LOW_CAPTURE_RISING: /*cr0 |= (0 << 6);*/break;
        case IDLE_LOW_CAPTURE_FALLING:  cr0 |= (2 << 6);  break;
        case IDLE_HIGH_CAPTURE_FALLING: cr0 |= (1 << 6);  break;
        case IDLE_HIGH_CAPTURE_RISING:  cr0 |= (3 << 6);  break;
        default: return false;
    }

    uint8_t cpsdvsr, scr;
    uint32_t pclk = sys_get_cpu_clock();

    if(!calcDividerSettings(pclk, sck_freq_hz, &cpsdvsr, &scr))
    {
        return false;
    }

    cr0 |= (scr << 8);

    switch(ssp)
    {
        case SSP0:
            LPC_SC->PCONP |= (1 << 21); /* Power */

            LPC_SC->PCLKSEL1 &= ~(3 << 10); /* PCLK = CCLK */
            LPC_SC->PCLKSEL1 |=  (1 << 10);

            if(ssp0_state.task == NULL)
            {
                /* Create the queue */
                ssp0_state.queue = xQueueCreate(8, sizeof(spi_msg_t));

                if(ssp0_state.queue == NULL)
                {
                    LPC_SC->PCONP &= ~(1 << 21);
                    return false;
                }

                /* Start the controller task */
                if(xTaskCreate(SSP0ControllerTask, "SSP0", STACK_DEPTH, NULL, 3, &(ssp0_state.task)) != pdPASS)
                {
                    LPC_SC->PCONP &= ~(1 << 21);
                    return false;
                }
            }
            else
            {
                /* Reset the existing task (clear its queue) */
                xQueueReset(ssp0_state.queue);
            }

            /* Disable all interrupts */
            ssp0_state.ssp->IMSC = 0x0;

            /* Register interrupt handler */
            isr_register(SSP0_IRQn, SSP0ISR);
            NVIC_EnableIRQ(SSP0_IRQn);

            ssp0_state.ssp->CR0  = cr0;
            ssp0_state.ssp->CPSR = (uint32_t)cpsdvsr;

            ssp0_state.ssp->CR1  = 0x2; /* Enable the SSP */

            xEventGroupSetBits(ssp0_state.ev, EVENT_HW_READY);
            break;

        case SSP1:
            LPC_SC->PCONP |= (1 << 10);

            LPC_SC->PCLKSEL0 &= ~(3 << 20);
            LPC_SC->PCLKSEL0 |=  (1 << 20);

            if(ssp1_state.task == NULL)
            {
                /* Create the queue */
                ssp1_state.queue = xQueueCreate(8, sizeof(spi_msg_t));

                if(ssp1_state.queue == NULL)
                {
                    LPC_SC->PCONP &= ~(1 << 10);
                    return false;
                }

                /* Start the controller task */
                if(xTaskCreate(SSP1ControllerTask, "SSP1", STACK_DEPTH, NULL, 3, &(ssp1_state.task)) != pdPASS)
                {
                    LPC_SC->PCONP &= ~(1 << 10);
                    return false;
                }
            }
            else
            {
                /* Reset the existing task (clear its queue) */
                xQueueReset(ssp1_state.queue);
            }

            /* Disable all interrupts */
            ssp1_state.ssp->IMSC = 0x0;

            /* Register interrupt handler */
            isr_register(SSP1_IRQn, SSP0ISR);
            NVIC_EnableIRQ(SSP1_IRQn);

            ssp1_state.ssp->CR0  = cr0;
            ssp1_state.ssp->CPSR = (uint32_t)cpsdvsr;

            ssp1_state.ssp->CR1  = 0x2; /* Enable the SSP */

            xEventGroupSetBits(ssp1_state.ev, EVENT_HW_READY);
            break;

        default: return false;
    }

    return true;
}

void SPIController::SSP0ControllerTask(void* p)
{
    controllerTask(&ssp0_state);
    vTaskSuspend(NULL); /* Should never reach here */
}

void SPIController::SSP1ControllerTask(void* p)
{
    controllerTask(&ssp1_state);
    vTaskSuspend(NULL); /* Should never reach here */
}

void SPIController::controllerTask(task_state_t* state)
{
    xEventGroupWaitBits(state->ev, EVENT_HW_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    while(1)
    {
        xQueueReceive(state->queue, &(state->msg), portMAX_DELAY);

        state->rx_i = 0;
        state->tx_i = 0;

        enableTxInterrupts(state->ssp);
        enableRxInterrupts(state->ssp);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(state->msg.sem != NULL)
        {
            xSemaphoreGive(state->msg.sem);
        }
    }
}

void SPIController::SSP0ISR(void)
{
    portYIELD_FROM_ISR(ISR(&ssp0_state));
}

void SPIController::SSP1ISR(void)
{
    portYIELD_FROM_ISR(ISR(&ssp1_state));
}

BaseType_t SPIController::ISR(task_state_t* s)
{
    BaseType_t woken = pdFALSE;

    if(s->ssp->MIS & ((1 << 1) | (1 << 2)))
    {
        /* Rx FIFO not empty */

        /* Empty the Rx FIFO to the buffer */
        while((s->ssp->SR & (1 << 2)) && (s->rx_i < s->tx_i) && (s->rx_i < s->msg.len))
        {
            s->msg.buf[s->rx_i] = s->ssp->DR;
            s->rx_i++;
        }

        if(s->rx_i >= s->msg.len)
        {
            disableRxInterrupts(s->ssp);
        }
    }

    if(s->ssp->MIS & (1 << 3))
    {
        /* Tx FIFO at least half empty */

        /* Can send up to 4 bytes */
        for(uint32_t i = 0; (i < 4) && (s->tx_i < s->msg.len); i++, s->tx_i++)
        {
            s->ssp->DR = s->msg.buf[s->tx_i];
        }

        if(s->tx_i >= s->msg.len)
        {
            disableTxInterrupts(s->ssp);
        }
    }

    if((s->tx_i >= s->msg.len) && (s->rx_i >= s->msg.len))
    {
        /* Notify task that transmit and receive are done */
        vTaskNotifyGiveFromISR(s->task, &woken);
    }

    return woken;
}

void SPIController::transceive(ssp_t ssp, uint8_t* buf, uint32_t len, SemaphoreHandle_t sem)
{
    if(buf != NULL)
    {
        spi_msg_t new_msg;
        new_msg.buf = buf;
        new_msg.len = len;
        new_msg.sem = sem;

        switch(ssp)
        {
            case SSP0:
                if(xSemaphoreGetMutexHolder(ssp0_state.mutex) == xTaskGetCurrentTaskHandle())
                {
                    xQueueSend(ssp0_state.queue, &new_msg, portMAX_DELAY);
                }
                break;

            case SSP1:
                if(xSemaphoreGetMutexHolder(ssp1_state.mutex) == xTaskGetCurrentTaskHandle())
                {
                    xQueueSend(ssp1_state.queue, &new_msg, portMAX_DELAY);
                }
                break;

            default: break;
        }
    }
}

void SPIController::acquire(ssp_t ssp)
{
    switch(ssp)
    {
        case SSP0: xSemaphoreTake(ssp0_state.mutex, portMAX_DELAY); break;
        case SSP1: xSemaphoreTake(ssp1_state.mutex, portMAX_DELAY); break;
        default:   break;
    }
}

void SPIController::release(ssp_t ssp)
{
    switch(ssp)
    {
        case SSP0: xSemaphoreGive(ssp0_state.mutex); break;
        case SSP1: xSemaphoreGive(ssp1_state.mutex); break;
        default:   break;
    }
}

void SPIController::disableTxInterrupts(volatile LPC_SSP_TypeDef* ssp) {ssp->IMSC &= 0x7;}
void SPIController::disableRxInterrupts(volatile LPC_SSP_TypeDef* ssp) {ssp->IMSC &= 0x9;}
void SPIController::enableTxInterrupts(volatile LPC_SSP_TypeDef* ssp)  {ssp->IMSC |= 0x8;}
void SPIController::enableRxInterrupts(volatile LPC_SSP_TypeDef* ssp)  {ssp->IMSC |= 0x6;}
