#include <stm32f4xx.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include "spi.h"
#include "W5200.h"
#include <stdio.h>	
#define RESET_LOW()	GPIOA->BSRRL |= GPIO_Pin_3;
#define RESET_HIGH()    GPIOA->BSRRH |= GPIO_Pin_3;
#define CSON()		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
#define CSOFF()		GPIOA->BSRRL |= GPIO_Pin_4; // chip select  


/* ----------------------------------------------------------*/
/* --    this function initializes the SPI1 peripheral     --*/


/* initialize SPI1 */
void init_SPI1(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	// MOSI PA7
	GPIO_InitStruct.GPIO_Pin = SPIx_MOSI_PIN ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	// SCK PA5
	GPIO_InitStruct.GPIO_Pin = SPIx_SCK_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	// MISO PA6
	GPIO_InitStruct.GPIO_Pin = SPIx_MISO_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	
	/* Configure the chip select pin
	   in this case we will use PA4 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIOA->BSRRL |= GPIO_Pin_4; // set PA4 high

	/* Configure the hard reset pin
	   in this case we will use PA3 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIOA->BSRRL |= GPIO_Pin_3; // set PA3 high

	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at second edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct); 
	
	SPI_Cmd(SPI1, ENABLE);			
	

	/* setup DMA */

	// enable clock 
	RCC_AHB1PeriphClockCmd (SPIx_DMA_CLK, ENABLE); 
	
	// start with blank DMA configuration
	DMA_DeInit (SPIx_TX_DMA_STREAM);
	DMA_DeInit (SPIx_RX_DMA_STREAM);

	// check if DMA stream is disabled before enabling 
	// this is useful when stream is enabled and disabled multiple times. 
	while (DMA_GetCmdStatus (SPIx_TX_DMA_STREAM) != DISABLE);
	while (DMA_GetCmdStatus (SPIx_RX_DMA_STREAM) != DISABLE);
	
	
	DMA_StructInit(&DMA_InitStruct);
  	//Configure DMA Initialization Structure
	//DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable ;
 	//DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull ;
  	//DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  	DMA_InitStruct.DMA_PeripheralBaseAddr =(uint32_t) (&(SPIx->DR)) ;
  	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  	// Configure TX DMA 
  	DMA_InitStruct.DMA_Channel = SPIx_TX_DMA_CHANNEL ;
  	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &bufferTX ;
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
  	DMA_Init(SPIx_TX_DMA_STREAM, &DMA_InitStruct);
	// Configure RX DMA 
  	DMA_InitStruct.DMA_Channel = SPIx_RX_DMA_CHANNEL ;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&bufferRX; 
	DMA_InitStruct.DMA_BufferSize = MAX_BUFFER_LENGTH;
	DMA_Init(SPIx_RX_DMA_STREAM, &DMA_InitStruct);	
	
	DMA_ITConfig(SPIx_TX_DMA_STREAM, DMA_IT_TC, ENABLE); 
	DMA_ITConfig(SPIx_RX_DMA_STREAM, DMA_IT_TC, ENABLE); 
  
	SPI_I2S_ClearFlag(SPIx, SPI_I2S_FLAG_TXE);
	SPI_I2S_ClearFlag(SPIx, SPI_I2S_FLAG_RXNE);
  	
	// enable the interrupt in the NVIC
 	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream3_IRQn;
  	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x1;
 	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0;
  	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init (&NVIC_InitStruct);
	// enable the interrupt in the NVIC
 	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x1;
 	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0;
  	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init (&NVIC_InitStruct);
  	// Enable dma tx and rx request
	SPI_I2S_DMACmd (SPIx, SPI_I2S_DMAReq_Tx, ENABLE);	
	SPI_I2S_DMACmd (SPIx, SPI_I2S_DMAReq_Rx, ENABLE);	
	

	/* Configure interrupt pin in this case we will use PA0 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct1;
	/* Connect EXTI Line to appropriate GPIO Pin */ 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	
	/* Configure EXTI Line */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
 	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
 	
 	/* Enable and set EXTI Line Interrupt */
	NVIC_InitStruct1.NVIC_IRQChannel = EXTI0_IRQn;
 	NVIC_InitStruct1.NVIC_IRQChannelPreemptionPriority = 0x1;
	NVIC_InitStruct1.NVIC_IRQChannelSubPriority = 0x0;
 	NVIC_InitStruct1.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct1);


	
}

void DMA2_Stream2_IRQHandler()
{
  // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (SPIx_RX_DMA_STREAM, DMA_IT_TCIF2)) {
    
	DMA_ClearITPendingBit (SPIx_RX_DMA_STREAM, DMA_IT_TCIF2);
	
	while (SPI_I2S_GetFlagStatus (SPIx, SPI_I2S_FLAG_BSY) == SET);
    /*
     * The DMA stream is disabled in hardware at the end of the transfer
     * Now we can deselect the display. If more than one peripheral was being run
     * on this SPI peripheral, we would have to do both/all of them, or work out
     * which one was active and deselect that one.i
	
     */
	CSOFF();
	DMA_Cmd(SPIx_TX_DMA_STREAM, DISABLE);		
	DMA_Cmd(SPIx_RX_DMA_STREAM, DISABLE);		
 
	taskENTER_CRITICAL(); 
       xSemaphoreGive( xSemaphoreDMASPI);
//	xHigherPriorityTaskWoken = pdFalse;
//	xSemaphoreGiveFromISR( xSemaphoreDMASPI, &xHigherPriorityTaskWoken );
	taskEXIT_CRITICAL(); //
  }	

  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
void DMA2_Stream3_IRQHandler()
{
  // Test if DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus (SPIx_TX_DMA_STREAM, DMA_IT_TCIF3)) {
    
	DMA_ClearITPendingBit (SPIx_TX_DMA_STREAM, DMA_IT_TCIF3);
	
	while (SPI_I2S_GetFlagStatus (SPIx, SPI_I2S_FLAG_BSY) == SET);
    /*
     * The DMA stream is disabled in hardware at the end of the transfer
     * Now we can deselect the display. If more than one peripheral was being run
     * on this SPI peripheral, we would have to do both/all of them, or work out
     * which one was active and deselect that one.i
	
     */
	CSOFF();
	DMA_Cmd(SPIx_TX_DMA_STREAM, DISABLE);		
	DMA_Cmd(SPIx_RX_DMA_STREAM, DISABLE);		
       
	taskENTER_CRITICAL(); 
	 xSemaphoreGive( xSemaphoreDMASPI );
//	xHigherPriorityTaskWoken = pdFalse;
//	xSemaphoreGiveFromISR( xSemaphoreDMASPI, &xHigherPriorityTaskWoken );
 	taskEXIT_CRITICAL();  
 }
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/* This funtion is used to transmit and/ receive data 
 * with SPI1
 * 			data --> data to be transmitted
 * 			returns received value

*/
uint8_t SPI1_send(uint8_t data){

	
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI1, data);
	
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	uint8_t d = SPI_I2S_ReceiveData(SPI1);
	return d;
 	
}

uint8_t SPI1_recive(void)
{

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_I2S_SendData(SPI1,0x0 );
	
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	return SPI_I2S_ReceiveData(SPI1);
	
}


/*---------------------------- Matic Knap 25 Jun 2014 ---------------------*/

void spi_dma_send(uint16_t address, uint16_t data_len, uint8_t *data_buf)
{
		uint8_t buffer[data_len+4];
		buffer[0] = ((address & 0xff00) >> 8); // addres byte 1 
		buffer[1] = ((address & 0x00ff)); //address byte 2 
		buffer[2] = ((0x80 | (data_len & 0x7f00) >> 8 ));
		buffer[3] = (data_len & 0x00ff);

		int i;
		for (i = 4 ; i < data_len+4; i++) 
			buffer[i] = (data_buf[i-4]);
		DMA_SetCurrDataCounter(SPIx_TX_DMA_STREAM, data_len + 4);
		SPIx_TX_DMA_STREAM->M0AR =(uint32_t) &buffer;	
		CSON(); // chip select 
		DMA_Cmd(SPIx_TX_DMA_STREAM, ENABLE);		
		DMA_Cmd(SPIx_RX_DMA_STREAM, ENABLE);	
		/* Block until the semaphore is given */
        	 xSemaphoreTake(xSemaphoreDMASPI, portMAX_DELAY);

	//	CSOFF(); // chip deselect 
			
	
}

void spi_dma_send2B(uint16_t address,  uint16_t data_buf)
{
	uint8_t buffer[6];
	uint16_t data_len = 2;
	buffer[0] = ((address & 0xff00) >> 8); // addres byte 1 
	buffer[1] = ((address & 0x00ff)); //address byte 2 
	buffer[2] = ((0x80 | (data_len & 0x7f00) >> 8 ));
	 buffer[3] = (data_len & 0x00ff);
	 buffer[4] = ((data_buf & 0xff00) >> 8);
	 buffer[5] = (data_buf & 0x00ff);
	DMA_SetCurrDataCounter(SPIx_TX_DMA_STREAM, 6);
	SPIx_TX_DMA_STREAM->M0AR =(uint32_t) &buffer;	
	CSON(); // chip select 
	DMA_Cmd(SPIx_TX_DMA_STREAM, ENABLE);		
	DMA_Cmd(SPIx_RX_DMA_STREAM, ENABLE);	
	/* Block until the semaphore is given */
        xSemaphoreTake(xSemaphoreDMASPI, portMAX_DELAY);	
	
//	CSOFF(); // chip deselect 
			
}

void spi_dma_sendByte(uint16_t address,  uint8_t data_buf)
{
		uint8_t buffer[5];
		uint16_t data_len=0x1;
		buffer[0] = ((address & 0xff00) >> 8); // addres byte 1 
		buffer[1] = ((address & 0x00ff)); //address byte 2 
		buffer[2] = ((0x80 | (data_len & 0x7f00) >> 8 ));
		buffer[3] = (data_len & 0x00ff);
		buffer[4] = data_buf ; 
		DMA_SetCurrDataCounter(SPIx_TX_DMA_STREAM, 5);
		SPIx_TX_DMA_STREAM->M0AR =(uint32_t) &buffer;	
		CSON(); // chip select 
		DMA_Cmd(SPIx_TX_DMA_STREAM, ENABLE);		
		DMA_Cmd(SPIx_RX_DMA_STREAM, ENABLE);
		/* Block until the semaphore is given */
	        xSemaphoreTake(xSemaphoreDMASPI, portMAX_DELAY);	
				
//		CSOFF(); // chip deselect 
			
}


void spi_send(uint16_t address, uint16_t data_len, uint8_t *data_buf)
{
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x80 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		int i;
		for (i = 0; i < data_len; i++) SPI1_send(data_buf[i]);
		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
			
	
}

void spi_send2B(uint16_t address,  uint16_t data_buf)
{
		int16_t data_len = 2; 
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x80 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		SPI1_send((data_buf & 0xff00) >> 8);
		SPI1_send(data_buf & 0x00ff);

		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
			
	
}


void spi_sendByte(uint16_t address,  uint8_t data_buf)
{
		int16_t data_len = 1; 
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x80 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		SPI1_send(data_buf);
		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
			
	
}


uint16_t spi_read2B(uint16_t address)
{
		uint16_t data_len = 2; 
		uint8_t data_buf[2];
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x00 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		int i;
		for (i = 0; i < data_len; i++)
		{
		
			 data_buf[i] = SPI1_recive();
			
		}

		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
		
		data_len = 0x0000;
		return (data_buf[0] << 8) | data_buf[1];

}
uint16_t spi_dma_read2B(uint16_t address)
{
		uint16_t data_len = 2; 
		uint8_t data_buf[4];
		
		data_buf[0] = ((address & 0xff00) >> 8); // addres byte 1 
		data_buf[1] = ((address & 0x00ff)); //address byte 2 
		data_buf[2] = ((0x00 | (data_len & 0x7f00) >> 8 ));
		data_buf[3] = (data_len & 0x00ff);

		int i;
		for (i = 0; i < data_len; i++) data_buf[i] = 0x0;

		DMA_SetCurrDataCounter(SPIx_RX_DMA_STREAM, data_len+4);
		DMA_SetCurrDataCounter(SPIx_TX_DMA_STREAM, 4+data_len);
		SPIx_TX_DMA_STREAM->M0AR =(uint32_t) &bufferTX;	
		SPIx_RX_DMA_STREAM->M0AR =(uint32_t)&bufferRX;	

		CSON(); // chip select 
		DMA_Cmd(SPIx_TX_DMA_STREAM, ENABLE);		
		DMA_Cmd(SPIx_RX_DMA_STREAM, ENABLE);
		//CSOFF(); // chip deselect		

		return (data_buf[0] << 8) | data_buf[1];

}
void spi_dma_read(uint16_t address, uint16_t data_len)
{
		uint8_t buffer[data_len+4];
		buffer[0] = ((address & 0xff00) >> 8); // addres byte 1 
		buffer[1] = ((address & 0x00ff)); //address byte 2 
		buffer[2] = ((0x00 | (data_len & 0x7f00) >> 8 ));
		buffer[3] = (data_len & 0x00ff);

		int i;
		for (i = 4 ; i < data_len+4; i++) buffer[i] = 0x0;

		DMA_SetCurrDataCounter(SPIx_RX_DMA_STREAM, data_len+4);
		DMA_SetCurrDataCounter(SPIx_TX_DMA_STREAM, 4+data_len);
		SPIx_TX_DMA_STREAM->M0AR =(uint32_t) &buffer;	
		SPIx_RX_DMA_STREAM->M0AR =(uint32_t)&bufferRX;	

		CSON(); // chip select 
		DMA_Cmd(SPIx_TX_DMA_STREAM, ENABLE);		
		DMA_Cmd(SPIx_RX_DMA_STREAM, ENABLE);
		/* Block until the semaphore is given */
        	xSemaphoreTake(xSemaphoreDMASPI, portMAX_DELAY);	
		//CSOFF(); // chip deselect		

}	
void spi_read(uint16_t address, uint16_t data_len, uint8_t *data_buf)
{
		GPIOA->BSRRH |= GPIO_Pin_4; // chip select  
		
		SPI1_send((address & 0xff00) >> 8); // addres byte 1 
		SPI1_send((address & 0x00ff)); //address byte 2 
		SPI1_send((0x00 | (data_len & 0x7f00) >> 8 ));
		SPI1_send(data_len & 0x00ff);

		int i;
		for (i = 0; i < data_len; i++)
		{
		
			 data_buf[i] = SPI1_recive();
			
		}
		GPIOA->BSRRL |= GPIO_Pin_4; // chip deselect 
			
	
}
/*---------------------------- Matic Knap 25 Jun 2014 ---------------------*/

