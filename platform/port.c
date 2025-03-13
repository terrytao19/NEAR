/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port.h"
#include "deca_device_api.h"
#include "stm32f1xx_hal_conf.h"
#include "usbd_cdc_if.h"

/****************************************************************************//**
 *
 *                              APP global variables
 *
 *******************************************************************************/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2; /*clocked from 36MHz*/

/****************************************************************************//**
 *
 *                  Port private variables and function prototypes
 *
 *******************************************************************************/
static volatile uint32_t signalResetDone;

/****************************************************************************//**
 *
 *                              Time section
 *
 *******************************************************************************/

/* @fn    portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency.
 *        The resolution of time32_incr is usually 1/1000 sec.
 * */
__INLINE uint32_t
portGetTickCnt(void)
{
    return HAL_GetTick();
}


/* @fn    usleep
 * @brief precise usleep() delay
 * */
#pragma GCC optimize ("O0")
int usleep(useconds_t usec)
{
    int i,j;
#pragma GCC ivdep
    for(i=0;i<usec;i++)
    {
#pragma GCC ivdep
        for(j=0;j<2;j++)
        {
            __NOP();
            __NOP();
        }
    }
    return 0;
}


/* @fn    Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
__INLINE void
Sleep(uint32_t x)
{
    HAL_Delay(x);
}

/****************************************************************************//**
 *
 *                              END OF Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              Configuration section
 *
 *******************************************************************************/

/* @fn    peripherals_init
 * */
int peripherals_init (void)
{
    /* All has been initialized in the CubeMx code, see main.c */
    return 0;
}


/* @fn    spi_peripheral_init
 * */
void spi_peripheral_init()
{

    /* SPI's has been initialized in the CubeMx code, see main.c */

    port_LCD_RS_clear();

    port_LCD_RW_clear();
}



/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t x)
{
    return ((NVIC->ISER[(((uint32_t)x) >> 5UL)] &\
            (uint32_t)(1UL << (((uint32_t)x) & 0x1FUL)) ) == (uint32_t)RESET)?(RESET):(SET);
}
/****************************************************************************//**
 *
 *                          End of configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                          DW1000 port section
 *
 *******************************************************************************/

/* @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(void)
{
    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);
    Sleep(1);
    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);
    Sleep(7);                       //wait 7ms for DW1000 XTAL to stabilise
}

/* @fn      port_set_dw1000_slowrate
 * @brief   set 2.25MHz
 *          note: hspi1 is clocked from 72MHz
 * */
void port_set_dw1000_slowrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    HAL_SPI_Init(&hspi1);
}

/* @fn      port_set_dw1000_fastrate
 * @brief   set 18MHz
 *          note: hspi1 is clocked from 72MHz
 * */
void port_set_dw1000_fastrate(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    HAL_SPI_Init(&hspi1);
}

/****************************************************************************//**
 *
 *                          End APP port section
 *
 *******************************************************************************/



/****************************************************************************//**
 *
 *                              IRQ section
 *
 *******************************************************************************/

/* @fn      HAL_GPIO_EXTI_Callback
 * @brief   IRQ HAL call-back for all EXTI configured lines
 *          i.e. DW_RESET_Pin and DW_IRQn_Pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DW_RESET_Pin)
    {
        signalResetDone = 1;
    }
    else if (GPIO_Pin == DW_IRQn_Pin)
    {
        process_deca_irq();
    }
    else
    {
    }
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW1000 IRQ
 *          it re-enters the IRQ routing and processes all events.
 *          After processing of all events, DW1000 will clear the IRQ line.
 * */
__INLINE void process_deca_irq(void)
{
    while(port_CheckEXT_IRQ() != 0)
    {

        port_deca_isr();

    } //while DW1000 IRQ line active
}


/* @fn      port_DisableEXT_IRQ
 * @brief   wrapper to disable DW_IRQ pin IRQ
 *          in current implementation it disables all IRQ from lines 5:9
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
    NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_EnableEXT_IRQ
 * @brief   wrapper to enable DW_IRQ pin IRQ
 *          in current implementation it enables all IRQ from lines 5:9
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
    NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);
}


/* @fn      port_GetEXT_IRQStatus
 * @brief   wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
    return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn);
}


/* @fn      port_CheckEXT_IRQ
 * @brief   wrapper to read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
    return HAL_GPIO_ReadPin(DECAIRQ_GPIO, DW_IRQn_Pin);
}


/****************************************************************************//**
 *
 *                              END OF IRQ section
 *
 *******************************************************************************/



/****************************************************************************//**
 *
 *                              USB report section
 *
 *******************************************************************************/
#include "usb_device.h"

#define REPORT_BUFSIZE  0x2000

extern USBD_HandleTypeDef  hUsbDeviceFS;

static struct
{
    HAL_LockTypeDef       Lock;     /*!< locking object                  */
}
txhandle={.Lock = HAL_UNLOCKED};

static char     rbuf[REPORT_BUFSIZE];               /**< circular report buffer, data to be transmitted in flush_report_buff() Thread */
static struct   circ_buf report_buf = { .buf = rbuf,
                                        .head= 0,
                                        .tail= 0};

static uint8_t  ubuf[CDC_DATA_FS_MAX_PACKET_SIZE];  /**< used to transmit new chunk of data in single USB flush */

/* @fn      port_tx_msg()
 * @brief   put message to circular report buffer
 *          it will be transmitted in background ASAP from flushing Thread
 * @return  HAL_BUSY - can not do it now, wait for release
 *          HAL_ERROR- buffer overflow
 *          HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef port_tx_msg(uint8_t   *str, int  len)
{
    int head, tail, size;
    HAL_StatusTypeDef   ret;

    /* add TX msg to circular buffer and increase circular buffer length */

    __HAL_LOCK(&txhandle);  //return HAL_BUSY if locked
    head = report_buf.head;
    tail = report_buf.tail;
    __HAL_UNLOCK(&txhandle);

    size = REPORT_BUFSIZE;

    if(CIRC_SPACE(head, tail, size) > (len))
    {
        while (len > 0)
        {
            report_buf.buf[head]= *(str++);
            head= (head+1) & (size - 1);
            len--;
        }

        __HAL_LOCK(&txhandle);  //return HAL_BUSY if locked
        report_buf.head = head;
        __HAL_UNLOCK(&txhandle);

#ifdef CMSIS_RTOS
        osSignalSet(usbTxTaskHandle, signalUsbFlush);   //RTOS multitasking signal start flushing
#endif
        ret = HAL_OK;
    }
    else
    {
        /* if packet can not fit, setup TX Buffer overflow ERROR and exit */
        ret = HAL_ERROR;
    }

    return ret;
}


/* @fn      flush_report_buff
 * @brief   FLUSH should have higher priority than port_tx_msg()
 *          it shall be called periodically from process, which can not be locked,
 *          i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_report_buff(void)
{
    USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData);

    int i, head, tail, len, size = REPORT_BUFSIZE;

    __HAL_LOCK(&txhandle);  //"return HAL_BUSY;" if locked
    head = report_buf.head;
    tail = report_buf.tail;
    __HAL_UNLOCK(&txhandle);

    len = CIRC_CNT(head, tail, size);

    if( len > 0 )
    {
        /*  check USB status - ready to TX */
        if((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) || (hcdc->TxState != 0))
        {
            return HAL_BUSY;    /**< USB is busy. Let it send now, will return next time */
        }


        /* copy MAX allowed length from circular buffer to non-circular TX buffer */
        len = MIN(CDC_DATA_FS_MAX_PACKET_SIZE, len);

        for(i=0; i<len; i++)
        {
            ubuf[i] = report_buf.buf[tail];
            tail = (tail + 1) & (size - 1);
        }

        __HAL_LOCK(&txhandle);  //"return HAL_BUSY;" if locked
        report_buf.tail = tail;
        __HAL_UNLOCK(&txhandle);

        /* setup USB IT transfer */
        if(CDC_Transmit_FS(ubuf, (uint16_t)len) != USBD_OK)
        {
            /**< indicate USB transmit error */
        }
    }

    return HAL_OK;
}


/* DW1000 IRQ handler definition. */
port_deca_isr_t port_deca_isr = NULL;

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
void port_set_deca_isr(port_deca_isr_t deca_isr)
{
    /* Check DW1000 IRQ activation status. */
    ITStatus en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW1000 IRQ during the installation of the new handler. */
    if (en)
    {
        port_DisableEXT_IRQ();
    }
    port_deca_isr = deca_isr;
    if (en)
    {
        port_EnableEXT_IRQ();
    }
}


/****************************************************************************//**
 *
 *******************************************************************************/

