/**
  ******************************************************************************
  * @file    USART/USART_HyperTerminalInterrupt/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "main.h"

/** @addtogroup STM32F0xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup HyperTerminal_Interrupt
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TXBUFFERSIZE   150
#define RXBUFFERSIZE   150

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
uint8_t TxBuffer[TXBUFFERSIZE];
__IO uint16_t RxBuffer[RXBUFFERSIZE];
uint8_t NbrOfDataToTransfer = TXBUFFERSIZE;
uint8_t NbrOfDataToRead = RXBUFFERSIZE;
__IO uint8_t TxCount = 0; 
__IO uint16_t RxCount = 0; 
__IO uint16_t bb[16];
__IO uint8_t ren = 0;
__IO uint16_t RxCnt;
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
extern uint8_t ID[4];
__IO bool getSPI = false ;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(EVAL_COM1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    TxBuffer[TxCount++] = (USART_ReceiveData(EVAL_COM1) & 0x007F);
    if(RxCount == NbrOfDataToRead)
    {
      /* Disable the EVAL_COM1 Receive interrupt */
      USART_ITConfig(EVAL_COM1, USART_IT_RXNE, DISABLE);
    }
  }

  if(USART_GetITStatus(EVAL_COM1, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(EVAL_COM1, bb[ren++]);
    if(TxCount == NbrOfDataToTransfer)
    {
      /* Disable the EVAL_COM1 Transmit interrupt */
      USART_ClearITPendingBit(EVAL_COM1,USART_IT_TXE);
    }
  }
}
__IO uint8_t inbox = 0;
void SPI1_IRQHandler(void)
{
#if defined (SPI_SLAVE)   
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_TXE) == SET)
  {
    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, DISABLE);
  }
  /* SPI in Slave Receiver mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_RXNE) == SET)
  {
    bb[ren++] = SPI_I2S_ReceiveData16(SPIx);
    getSPI = true ;
  }
  
  /* SPI Error interrupt--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_OVR) == SET)
  {
    SPI_ReceiveData8(SPIx);
    SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_OVR);
  }
  
#endif /* SPI_SLAVE*/
  

  
#if defined (SPI_MASTER)
  /* SPI in Master Tramitter mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_TXE) == SET)
  {
    SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, DISABLE);
  }
  
  /* SPI in Master Receiver mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_RXNE) == SET)
  {
    bb[ren++] = SPI_ReceiveData8(SPIx);
  }
  
  /* SPI Error interrupt--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_OVR) == SET)
  {
    SPI_ReceiveData8(SPIx);
    SPI_I2S_GetITStatus(SPIx, SPI_I2S_IT_OVR);
  }
  
#endif /* SPI_MASTER*/
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

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
