/**
  ******************************************************************************
  * @file    LwIP/LwIP_IAP/Inc/main.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* IAP options selection ******************************************************/
#define USE_IAP_TFTP   /* enable IAP using TFTP */
//#define USE_IAP_HTTP   /* enable IAP using HTTP */
#define USE_LCD        /* enable LCD  */  
#define USE_DHCP       /* enable DHCP, if disabled static address is used */

/* Flash user area definition *************************************************/   
/* 
   IMPORTANT NOTE:
   ==============
   Be sure that USER_FLASH_FIRST_PAGE_ADDRESS do not overlap with IAP code.
   For example, with all option enabled the total readonly memory size using SW4STM32
   IDE v2.1.0, with optimization for size, is 59784 bytes.
   Based on this result four sectors of 16 Kbytes and a sector of 64 Kbytes will be used to store
   the IAP code, so the user Flash address will start from Sector5.

   In this application the define USER_FLASH_FIRST_PAGE_ADDRESS is set to 128K boundary,
   but it can be changed to any other address outside the 1st 128 Kbytes of the Flash.
   */
#define USER_FLASH_FIRST_PAGE_ADDRESS 0x08020000 /* Only as example see comment */
#define USER_FLASH_LAST_PAGE_ADDRESS  0x080E0000
#define USER_FLASH_END_ADDRESS        0x080FFFFF  

/* UserID and Password definition *********************************************/
#define USERID       "user"
#define PASSWORD     "stm32"
#define LOGIN_SIZE   (20+ sizeof(USERID) + sizeof(PASSWORD))
 
/* Static IP Address definition ***********************************************/
#define IP_ADDR0   (uint8_t) 192
#define IP_ADDR1   (uint8_t) 168
#define IP_ADDR2   (uint8_t) 0
#define IP_ADDR3   (uint8_t) 10

/* NETMASK definition *********************************************************/
#define NETMASK_ADDR0   (uint8_t) 255
#define NETMASK_ADDR1   (uint8_t) 255
#define NETMASK_ADDR2   (uint8_t) 255
#define NETMASK_ADDR3   (uint8_t) 0

/* Gateway Address definition *************************************************/
#define GW_ADDR0   (uint8_t) 192
#define GW_ADDR1   (uint8_t) 168
#define GW_ADDR2   (uint8_t) 0
#define GW_ADDR3   (uint8_t) 1

/* Exported macro ------------------------------------------------------------*/
#define LED_GREEN_Pin GPIO_PIN_9
#define LED_GREEN_GPIO_Port GPIOG
#define LED_YELLOW_Pin GPIO_PIN_10
#define LED_YELLOW_GPIO_Port GPIOG
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOG

#define LED1                                                                 0
#define LED2                                                                 1
#define LED3                                                                 2

#define DHCP_IDLE_PERIOD                                                  5000

/* Exported variables ---------------------------------------------------------*/
extern uint8_t gpu_row;
extern __IO uint16_t gwDhcpIdleTimeCount;

/** @defgroup LCD_LOG_Exported_Macros
  * @{
  */ 
#define  LCD_ErrLog(...)    do { \
                                 sprintf((char*)gabUartRxBuffer, __VA_ARGS__);\
                                 sprintf((char*)gabUartTxBuffer, "SXY(0,0);SBC(0);DS12(1,%d,'%s',17);\r\n", gpu_row, gabUartRxBuffer);\
                                 UART_Send(gabUartTxBuffer, strlen((char*)gabUartTxBuffer));\
                                 gpu_row += 12;\
                                 if(gpu_row > 152)\
                                   gpu_row = 12;\
                                 HAL_Delay(200);\
                               }while (0)

#define  LCD_UsrLog(...)    do { \
                                 sprintf((char*)gabUartRxBuffer, __VA_ARGS__);\
	                             sprintf((char*)gabUartTxBuffer, "SXY(0,0);SBC(0);DS12(1,%d,'%s',15);\r\n", gpu_row, gabUartRxBuffer);\
                                 UART_Send(gabUartTxBuffer, strlen((char*)gabUartTxBuffer));\
                                 gpu_row += 12;\
                                 if(gpu_row > 152)\
                                   gpu_row = 12;\
                                 HAL_Delay(200);\
                               } while (0)

#define  LCD_DbgLog(...)    do { \
                                 sprintf((char*)gabUartRxBuffer, __VA_ARGS__);\
                                 sprintf((char*)gabUartTxBuffer, "SXY(0,0);SBC(0);DS12(1,%d,'%s',15);\r\n", gpu_row, gabUartRxBuffer);\
                                 UART_Send(gabUartTxBuffer, strlen((char*)gabUartTxBuffer));\
                                 gpu_row += 12;\
                                 if(gpu_row > 152)\
                                   gpu_row = 12;\
                                 HAL_Delay(200);\
                               }while (0)

/* Exported functions ------------------------------------------------------- */
/* Exported function prototypes ----------------------------------------------*/
void BSP_LED_On(uint8_t led);
void BSP_LED_Off(uint8_t led);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

