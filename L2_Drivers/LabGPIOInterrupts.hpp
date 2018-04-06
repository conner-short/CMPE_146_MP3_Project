#ifndef LABGPIOINTERRUPTS_H
#define LABGPIOINTERRUPTS_H

typedef enum
{
    RISING,
    FALLING,
    BOTH
} InterruptCondition_E;

typedef void (*isr_ptr_t)(void);

class LabGPIOInterrupts
{
private:
    LabGPIOInterrupts() {}
    LabGPIOInterrupts(LabGPIOInterrupts const&) {}
    void operator=(LabGPIOInterrupts const&) {}

    /**
     * The ISR table has three dimensions: isr_table[x][y][z]
     *  x: port number: 0 for P0, 1 for P2
     *  y: 0 for rising-edge routine, 1 for falling-edge
     *  z: pin number
     */
    static isr_ptr_t m_isr_table[2][2][31];

public:
    static LabGPIOInterrupts& getInstance();
    /**
     * 1) Should setup register "externalIRQHandler" as the EINT3 ISR.
     * 2) Should configure NVIC to notice EINT3 IRQs.
     */
    void init();
    /**
     * This handler should place a function pointer within the lookup table for the externalIRQHandler to find.
     *
     * @param[in] port         specify the GPIO port
     * @param[in] pin          specify the GPIO pin to assign an ISR to
     * @param[in] pin_isr      function to run when the interrupt event occurs
     * @param[in] condition    condition for the interrupt to occur on. RISING, FALLING or BOTH edges.
     * @return should return true if valid ports, pins, isrs were supplied and pin isr insertion was sucessful
     */
    bool attachInterruptHandler(uint8_t port, uint32_t pin, isr_ptr_t pin_isr, InterruptCondition_E condition);
    /**
     * After the init function has run, this will be executed whenever a proper
     * EINT3 external GPIO interrupt occurs. This function figure out which pin
     * has been interrupted and run the ccorrespondingISR for it using the lookup table.
     *
     * VERY IMPORTANT! Be sure to clear the interrupt flag that caused this
     * interrupt, or this function will be called again and again and again, ad infinitum.
     *
     * Also, NOTE that your code needs to be able to handle two GPIO interrupts occurring
     * at the same time.
     */
    static void externalIRQHandler(void);
    ~LabGPIOInterrupts();
};

#endif
