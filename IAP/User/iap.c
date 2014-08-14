/**
 * @file    User/bootloader.c
 * @author  Ruru Lin
 * @version V1.0
 * @date    25-July-2014
 * @brief   stm32f107vct6 usart bootloader
 */
#include "stm32f10x_conf.h"
#include "stm32f10x.h"

/** @addtogroup STM32F107VCT6_IAP_Bootloader
  * @{
  */

/** @addtogroup IAP_UART_Program
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;
typedef void(*pFunction)(void);

/* Private define ------------------------------------------------------------*/
/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#endif

/* Define Application Address Area */
#define BANK1_WRITE_START_ADDR  ((uint32_t)0x08003000)
#define BANK1_WRITE_END_ADDR    ((uint32_t)(BANK1_WRITE_START_ADDR + ProgramSize)) ///< BANK1_WRITE_START_ADDR + ProgramSize

#ifdef STM32F10X_XL
 #define BANK2_WRITE_START_ADDR   ((uint32_t)0x08088000)
 #define BANK2_WRITE_END_ADDR     ((uint32_t)0x0808C000)
#endif /* STM32F10X_XL */

/* Define IAP USART Setting */
#if defined(STM32F10X_CL)
  #define USARTy                   USART1
  #define USARTy_GPIO              GPIOA
  #define USARTy_CLK               RCC_APB2Periph_USART1
  #define USARTy_GPIO_CLK          RCC_APB2Periph_GPIOA
  #define USARTy_RxPin             GPIO_Pin_10
  #define USARTy_TxPin             GPIO_Pin_9
  #define USARTy_IRQn              USART1_IRQn
  #define USARTy_IRQHandler        USART1_IRQHandler
	#define BufferSize  32
#endif

/* Define Application Parameters */
#define ProgramSize  5952
#define ApplicationAddress    0x08003000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Flash Parameters */
uint32_t EraseCounter = 0x00, Address = 0x00, addr_data = 0x00;
uint32_t Data = 0x00;
uint32_t NbrOfPage = 0x00;
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;

/* Jump Parameters */
pFunction Jump_To_Application;
uint32_t JumpAddress;

/* Peripheral InitStructure */
USART_InitTypeDef USART_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

/* Peripheral InitStructure */
uint8_t DST_Buffer[BufferSize];
uint8_t Send_Buffer[ProgramSize];
uint32_t number_SendBuf = 0x00;

/* Private function prototypes -----------------------------------------------*/
void DMA_Configuration(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void Delay(__IO uint32_t nCount);
void *memcpy(void *dst, const void *src, unsigned n);
void jump_to_app(void);

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
	/* Configure the USARTy */
	USART_Configuration();

  while (1)
  {
		/**
			* UART receive data with DMA1 channel5. If DST_Buffer is full, DMA1_IT_TC5 will be set.
		  */
		if(DMA_GetITStatus(DMA1_IT_TC5)){
			DMA_ClearITPendingBit(DMA1_IT_GL5);		///< Clear DMA1_IT_GL5 Flags
			memcpy(&Send_Buffer[number_SendBuf],DST_Buffer, sizeof(DST_Buffer));	///< Copy DST_Buffer[BufferSize] to Send_Buffer[ProgramSize]
			number_SendBuf += BufferSize;
		}
		/**
			* If receive data size = ProgramSize, Program will be write to flash at BANK1_WRITE_START_ADDR
		  */
		if(number_SendBuf >= ProgramSize){
			DMA_Cmd(DMA1_Channel5, DISABLE);			///< Disable UART Rx
			DMA_Cmd(DMA1_Channel4, ENABLE);				///< Enable UART Tx
			while (DMA_GetFlagStatus(DMA1_IT_TC4) == RESET);	///< Until DMA1 channel4 send data finish
			number_SendBuf = 0;
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA_ClearITPendingBit(DMA1_IT_GL4);
			
			/* Porgram APP to FLASH -------------------------------------------------------------*/ 
			/* Unlock the Flash Bank1 Program Erase controller */
			FLASH_Unlock();

			/* Define the number of page to be erased */
			NbrOfPage = (BANK1_WRITE_END_ADDR - BANK1_WRITE_START_ADDR) / FLASH_PAGE_SIZE;

			/* Clear All pending flags */
			FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

			/* Erase the FLASH pages */
			for(EraseCounter = 0; (EraseCounter <= NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
			{
				FLASHStatus = FLASH_ErasePage(BANK1_WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
			}
			
			/* Program Flash Bank1 */
			Address = BANK1_WRITE_START_ADDR;

			while((Address < BANK1_WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
			{
				/* Convert u8 to u32 then write to flash */
					Data = Send_Buffer[addr_data] | (Send_Buffer[addr_data+1] << 8) | (Send_Buffer[addr_data+2] << 16) | (Send_Buffer[addr_data+3] << 24);
					FLASHStatus = FLASH_ProgramWord(Address, Data);
					Address = Address + 4;
					addr_data = addr_data +4;
			}

			FLASH_LockBank1();
			
			/* Jump to User define Application Address */
			jump_to_app();
			
			while (1)
			{
				;
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
	/* USARTy and USARTz configured as follow:
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

  /* Configure USARTy */
  USART_Init(USARTy, &USART_InitStructure);
  /* Enable the USARTy */
  USART_Cmd(USARTy, ENABLE);
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
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTy->DR));
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
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTy->DR));
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Send_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = ProgramSize;
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
	USART_DMACmd(USARTy,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{    
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(USARTy_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(USARTy_CLK, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  /* Configure USARTy Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USARTy_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USARTy_GPIO, &GPIO_InitStructure);
  
  /* Configure USARTy Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USARTy_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USARTy_GPIO, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

/**
  * @brief  Jump to a given address and execute it 
  * @param  None
  * @retval define jump address
  */
void jump_to_app(void)
{
	/* If Program has been written */
	if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
		{
			/* Clear DMA1 Flags, disable UART and DMA */
			DMA_ClearITPendingBit(DMA1_IT_GL5);
			DMA_ClearITPendingBit(DMA1_IT_GL4);
			USART_Cmd(USARTy, DISABLE);
			USART_DMACmd(USARTy,USART_DMAReq_Rx|USART_DMAReq_Tx,DISABLE);
			DMA_Cmd(DMA1_Channel5, DISABLE);
			DMA_Cmd(DMA1_Channel4, DISABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
			
			/* Set system control register SCR->VTOR  */
			NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
			
			JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
			Jump_To_Application = (pFunction) JumpAddress;
			__set_MSP(*(__IO uint32_t*) ApplicationAddress);
			Jump_To_Application(); 
		}
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

/**
  * @}
  */ 

/**
  * @}
  */ 
