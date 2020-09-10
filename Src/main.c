  /**
  ******************************************************************************
  * @file    LwIP/LwIP_IAP/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "lwip/opt.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "app_ethernet.h"
#include "tftpserver.h"
#include "httpserver.h"

/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pFunction Jump_To_Application;
uint32_t JumpAddress;
uint32_t Command_str;
struct netif gnetif;
uint8_t gpu_row;
__IO uint16_t gwDhcpIdleTimeCount = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Key_Init(void);
static uint8_t Sample_Input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t bState);
static void BSP_Config(void);
static void Netif_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None 
  * @retval None
  */
int main(void)
{
  /* Configure Key Button */      
  Key_Init();
  Command_str = *(__IO uint32_t*)0x0801C000;
  
  /* Test if Key push-button is pressed */
  if((Sample_Input(GPIOA, GPIO_PIN_0, 0) == 0) || (Command_str == 0xFFFFFFFF))
  { /* Enter in IAP mode */
    /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
    HAL_Init();  
    
    /* Configure the system clock to 150 MHz */
    SystemClock_Config();
    
    /* Configure the BSP */
    BSP_Config();
    
    /* Initialize the LwIP stack */
    lwip_init();
    
    /* Configure the Network interface */
    Netif_Config();
    
#ifdef USE_IAP_HTTP
    /* Initialize the webserver module */
    IAP_httpd_init();
#endif
    
#ifdef USE_IAP_TFTP    
    /* Initialize the TFTP server */
    IAP_tftpd_init();
#endif  
    
    /* Notify user about the network interface config */
    User_notification(&gnetif);
    
    /* Infinite loop */
    while (1)
    {
      /* Read a received packet from the Ethernet buffers and send it 
         to the lwIP for handling */
      ethernetif_input(&gnetif);

      /* Handle timeouts */
      sys_check_timeouts();

#ifdef USE_DHCP
      /* handle periodic timers for LwIP */
      DHCP_Periodic_Handle(&gnetif);
#endif 
    }
  }
  else
  { /* Key push-button not pressed: jump to user application */
    
    /* Check if valid stack address (RAM address) then jump to user application */
    if (((*(__IO uint32_t*)USER_FLASH_FIRST_PAGE_ADDRESS) & 0x2FF80000 ) == 0x20000000)
    {
      /* Jump to user application */
      JumpAddress = *(__IO uint32_t*) (USER_FLASH_FIRST_PAGE_ADDRESS + 4);
      Jump_To_Application = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t*) USER_FLASH_FIRST_PAGE_ADDRESS);
      Jump_To_Application();
      /* do nothing */
      while(1);
    }
    else
    {/* Otherwise, do nothing */
      /* LED3 (RED) ON to indicate bad software (when not valid stack address) */
      HAL_Init();

      /* Configure the system clock to 150 MHz */
      SystemClock_Config();

      /* Configure the BSP */
      BSP_Config();
      
      BSP_LED_On(LED3);
      LCD_ErrLog("Bad software, invalid stack address");
      /* do nothing */
      while(1);
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLCLK, RCC_MCODIV_3);
}

/**
  * @brief  Initialize the key.
  * @param  None
  * @retval None
  */
static void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  key scan.
  * @param  GPIOx
  * @param  GPIO_Pin
  * @param  bState
  * @retval return 0 if state is true or return 1
  */
static uint8_t Sample_Input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t bState)
{
    uint8_t bRet;
    uint8_t bCount = 0;
    uint8_t bIndex;

    while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == bState)
    {
        bCount++;
        if(bCount > 50)
        {
            break;
        }
        for(bIndex = 0; bIndex < 50; bIndex++);
    }

    if(bCount > 50)
    {
        bRet = 0;
    }
    else
    {
        bRet = 1;
    }

    return bRet;
}

/**
  * @brief  Initializes the STM324xG-EVAL's LCD and LEDs resources.
  * @param  None
  * @retval None
  */
static void BSP_Config(void)
{
  MX_GPIO_Init();

  /* Set Systick Interrupt to the highest priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0);

#ifdef USE_LCD
  MX_DMA_Init();
  MX_UART5_Init();

  HAL_Delay(1000);
  
  /* Clear screen */
  UART_Send((uint8_t*)("CLS(0)\r\n"), strlen("CLS(0)\r\n"));
  HAL_Delay(200);
  /* Show Header and Footer texts */
  sprintf((char*)gabUartTxBuffer, "SXY(0,0);SBC(0);DS12(30,%d,'Ethernet IAP Application',15);", gpu_row);
  gpu_row += 12;
  sprintf((char*)(&gabUartTxBuffer[strlen((char*)gabUartTxBuffer)]), "SXY(0,0);SBC(0);DS12(125,160,'STM324xG board',15);\r\n");
  UART_Send(gabUartTxBuffer, strlen((char*)gabUartTxBuffer));
  HAL_Delay(200);
  
  LCD_UsrLog("State: Ethernet Initialization...");
#endif
}

/**
  * @brief  Configurates the network interface
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  
#ifdef USE_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  IP_ADDR4(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
  IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif /* USE_DHCP */
  
  /* add the network interface */    
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
  
  /*  Registers the default network interface */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }

#ifdef USE_DHCP
  /* Start DHCP negotiation for a network interface (IPv4) */
  dhcp_start(&gnetif);
#endif  /* USE_DHCP */

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernetif_update_config);
}

/**
  * @brief  EXTI line detection callbacks
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_14)
  {
    ethernetif_set_link(&gnetif);
  }
}

/**
  * @brief  BSP_LED_On
  * @param  led
  * @retval None
  */
void BSP_LED_On(uint8_t led)
{
    if(led == 0)
    {
        HAL_GPIO_WritePin(GPIOG, LED_GREEN_Pin, GPIO_PIN_RESET);
    }
    else if(led == 1)
    {
        HAL_GPIO_WritePin(GPIOG, LED_YELLOW_Pin, GPIO_PIN_RESET);
    }
    else if(led == 2)
    {
        HAL_GPIO_WritePin(GPIOG, LED_RED_Pin, GPIO_PIN_RESET);
    }
}

/**
  * @brief  BSP_LED_On
  * @param  led
  * @retval None
  */
void BSP_LED_Off(uint8_t led)
{
    if(led == 0)
    {
        HAL_GPIO_WritePin(GPIOG, LED_GREEN_Pin, GPIO_PIN_SET);
    }
    else if(led == 1)
    {
        HAL_GPIO_WritePin(GPIOG, LED_YELLOW_Pin, GPIO_PIN_SET);
    }
    else if(led == 2)
    {
        HAL_GPIO_WritePin(GPIOG, LED_RED_Pin, GPIO_PIN_SET);
    }
}

/**
  * @brief System Tick Callback
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
    /* Delay for dhcp restart */
    gwDhcpIdleTimeCount++;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
