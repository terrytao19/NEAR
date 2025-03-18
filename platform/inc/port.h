/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "compiler.h"
#include "main.h"

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

/* DW1000 IRQ (EXTI9_5_IRQ) handler type. */
typedef void (*port_deca_isr_t)(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_deca_isr_t deca_isr);

#define BUFFLEN     (64) //(4096+128)

#define BUF_SIZE    (64)

/*****************************************************************************************************************//*
**/

 /****************************************************************************//**
  *
  *                                 Types definitions
  *
  *******************************************************************************/
typedef uint64_t        uint64 ;

typedef int64_t         int64 ;


#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif

/****************************************************************************//**
 *
 *                              MACRO
 *
 *******************************************************************************/

#define DECAIRQ_EXTI_IRQn           EXTI4_15_IRQn

#define DECAIRQ                     DW_IRQn_Pin
#define DECAIRQ_GPIO                DW_IRQn_GPIO_Port

/****************************************************************************//**
 *
 *                              MACRO function
 *
 *******************************************************************************/

#define GPIO_ResetBits(x,y)             HAL_GPIO_WritePin(x,y, RESET)
#define GPIO_SetBits(x,y)               HAL_GPIO_WritePin(x,y, SET)
#define GPIO_ReadInputDataBit(x,y)      HAL_GPIO_ReadPin (x,y)


/* NSS pin is SW controllable */
#define port_SPIx_set_chip_select()     HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()   HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET)

/****************************************************************************//**
 *
 *                              port function prototypes
 *
 *******************************************************************************/

void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);

void port_wakeup_dw1000(void);

void port_set_dw1000_slowrate(void);
void port_set_dw1000_fastrate(void);

void process_deca_irq(void);

ITStatus EXTI_GetITEnStatus(uint32_t x);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
extern uint32_t     HAL_GetTick(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
