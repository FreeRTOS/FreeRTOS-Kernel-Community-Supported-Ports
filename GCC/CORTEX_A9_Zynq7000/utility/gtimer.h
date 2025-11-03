#ifndef GTIMER_H
#define GTIMER_H

/*
 * The gtimer driver supports basic operations on the 
 * Cortex-A9 global timer.
 *
 * It does NOT support interrupts.
 * 
 * To extend the driver functionalities refer to the global
 * timer registers section on the Cortex-A9 MPCore TMR.
 *  
 */
#include <stdint.h>
#include "xparameters_ps.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Global timer registers offsets */
#define GT_COUNTER_REGISTER_LOW32_OFFSET            0x0
#define GT_COUNTER_REGISTER_HIGH32_OFFSET           0x4
#define GT_CONTROL_REGISTER_OFFSET                  0x8

/*
#define GT_INTERRUPT_STATUS_REGISTER_OFFSET         0xC
#define GT_COMPARATOR_VALUE_REGISTER_LOW32_OFFSET   0x10
#define GT_COMPARATOR_VALUE_REGISTER_HIGH32_OFFSET  0x14
#define GT_AUTO_INCREMENT_REGISTER_OFFSET           0x18
*/

/* Global timer registers addresses */
#define GT_BASE_ADDRESS                             XPAR_GLOBAL_TMR_BASEADDR
#define GT_COUNTER_REGISTER_LOW32_ADDRESS           (GT_BASE_ADDRESS + GT_COUNTER_REGISTER_LOW32_OFFSET)
#define GT_COUNTER_REGISTER_HIGH32_ADDRESS          (GT_BASE_ADDRESS + GT_COUNTER_REGISTER_HIGH32_OFFSET)
#define GT_CONTROL_REGISTER_ADDRESS                 (GT_BASE_ADDRESS + GT_CONTROL_REGISTER_OFFSET)

/* Global timer registers */
#define GT_COUNTER_REGISTER_LOW32                   (*((volatile uint32_t *) GT_COUNTER_REGISTER_LOW32_ADDRESS))
#define GT_COUNTER_REGISTER_HIGH32                  (*((volatile uint32_t *) GT_COUNTER_REGISTER_HIGH32_ADDRESS))
#define GT_CONTROL_REGISTER                         (*((volatile uint32_t *) GT_CONTROL_REGISTER_ADDRESS))

/* Utility */
#define GT_CONTROL_REGISTER_PRESCALER_MASK          0xFF00
#define GT_CONTROL_REGISTER_PRESCALER_OFFSET        8U
#define GT_CONTROL_REGISTER_CTRL_BITS_MASK          0xE
 
/* Functions declaration */
static inline uint32_t GlobalTimer_isEnabled( void );
static inline void GlobalTimer_Start( void );
static inline void GlobalTimer_Stop( void );
static inline uint32_t GlobalTimer_GetCounter_Low32( void );
static inline void GlobalTimer_SetCounter( uint64_t counterValue );
static inline void GlobalTimer_SetPrescaler( uint8_t prescalerValue ); 
static inline uint8_t GlobalTimer_GetPrescaler( void );


/**
 * @brief Check if the global timer is enabled
 * 
 * @return FALSE (0) if the timer is not enabled, TRUE (1) otherwise
 */
static inline uint32_t GlobalTimer_isEnabled( void ){ 
    return (GT_CONTROL_REGISTER & 1);
}

/**
 * @brief Enable the global timer
 * 
 */
static inline void GlobalTimer_Start( void ){
    GT_CONTROL_REGISTER |= 0x1;
}

/**
 * @brief Stop the global timer
 */
static inline void GlobalTimer_Stop( void ){
    GT_CONTROL_REGISTER &= ~0x1;
}

/**
 * @brief Read the lower 32 bits of the counter register
 * 
 * @return The read 32-bit value
 */
static inline uint32_t GlobalTimer_GetCounter_Low32( void ){
    return GT_COUNTER_REGISTER_LOW32;
}

/**
 * @brief Write the 64-bit counter register
 * 
 * @param counterValue The 64-bit value written into the counter register
 * 
 * @note The timer should be stopped before writing 
 */
static inline void GlobalTimer_SetCounter( uint64_t counterValue ){
    GT_COUNTER_REGISTER_HIGH32 = (uint32_t) (counterValue >> 32);
    GT_COUNTER_REGISTER_LOW32 = (uint32_t) counterValue;
}

/**
 * @brief Set the prescaler of the global timer
 * 
 * @param prescalerValue The 8-bit prescaler value 
 */
static inline void GlobalTimer_SetPrescaler( uint8_t prescalerValue ){ 
    GT_CONTROL_REGISTER = ((((uint32_t) (prescalerValue)) << GT_CONTROL_REGISTER_PRESCALER_OFFSET) | 
                                (GT_CONTROL_REGISTER & (~GT_CONTROL_REGISTER_PRESCALER_MASK)));
}

/**
 * @brief Get the prescaler value of the global timer
 * 
 * @return The 8-bit value of the prescaler field
 */
static inline uint8_t GlobalTimer_GetPrescaler( void ){
    return ((GT_CONTROL_REGISTER & GT_CONTROL_REGISTER_PRESCALER_MASK) >> GT_CONTROL_REGISTER_PRESCALER_OFFSET);
}

/**
 * @brief Clear the Auto-Increment, IRQ enable and Comp Enable
 * bits in the control register 
 */
static inline void GlobalTimer_ClearControlBits( void ){ 
    GT_CONTROL_REGISTER &= (~GT_CONTROL_REGISTER_CTRL_BITS_MASK);
}

#ifdef __cplusplus
}
#endif

#endif