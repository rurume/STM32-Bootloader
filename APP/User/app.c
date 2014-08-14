/**
 * @file    User/app.c
 * @author  Ruru Lin
 * @version V1.0
 * @date    25-July-2014
 * @brief   stm32f107vct6 user test program 1- LED and UART
 */
#include "stm32f10x_conf.h"
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BufferSize  32
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Peripheral InitStructure */
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef  GPIO_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;

/* Peripheral InitStructure */
uint8_t DST_Buffer[BufferSize];
uint8_t Send_Buffer[BufferSize];

/* Private functions ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void DMA_Configuration(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void *memcpy(void *dst, const void *src, unsigned n);
void Delay(__IO uint32_t nCount);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */
	/* System Clocks Configuration */
  RCC_Configuration();
	/* Configure DMA1 and use AHB_clock */
  DMA_Configuration();    
  /* Configure the GPIO ports */
  GPIO_Configuration();
	/* Configure the USART1 */
	USART_Configuration();
	/* GPIOD Periph clock enable */

  while (1)
  {
    /* Set PD0 and PD2 */
    GPIOE->BSRR = 0x00000005;
    /* Reset PD0 and PD2 */
		Delay(10000000);
    GPIOE->BRR  = 0x00000005;
		Delay(10000000);
		
		/* Start DMA1 with UART: Recive 32bytes data then send to pc */
		while(1){
			if(DMA_GetITStatus(DMA1_IT_TC5)){
				DMA_ClearITPendingBit(DMA1_IT_GL5);

				memcpy(&Send_Buffer[0x00],DST_Buffer, sizeof(DST_Buffer));
			
				DMA_Cmd(DMA1_Channel5, DISABLE);
				DMA_Cmd(DMA1_Channel4, ENABLE);
				while (DMA_GetFlagStatus(DMA1_IT_TC4) == RESET);
				
				DMA_Cmd(DMA1_Channel4, DISABLE);
				DMA_ClearITPendingBit(DMA1_IT_GL4);
				DMA_Cmd(DMA1_Channel5, ENABLE);
			}
		}
  }
}


/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void USART_Configuration(void)
{
	/* USART1 and USARTz configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
	/* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure USART1 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure PD0 and PD2 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
}

/**
  * @brief  Configures the DMA.
  * @param  None
  * @retval None
  */
void DMA_Configuration(void)
{
	/* DMA1 channel5 Rx configuration */ 
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)DST_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	
	
	/* DMA1 channel4 Tx configuration */ 
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Send_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	
	DMA_Cmd(DMA1_Channel5, ENABLE);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{    
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}


/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

/**
  * @brief  Memcpy Function.
  * @param  [out] dst Target memory.
  * @param  [in]  src Source memory.
  * @param  [in]  n Size of Source Memory.
  * @retval None
  */
void *memcpy(void *dst, const void *src, unsigned n)
{
	const char *p = src;
	char *q = dst;

	while (n--) {
		*q++ = *p++;
	}

	return dst;
}
