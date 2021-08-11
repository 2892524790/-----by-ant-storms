#include "sys.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"
#include "queue.h"
#endif
#include "usart.h"	
#include "string.h"
#include "start_task.h"
#include "Tx2_task.h"
#include "detect_task.h"


unsigned char fff[10];
static uint8_t _USART6_DMA_RX_BUF[2][BSP_USART6_DMA_RX_BUF_LEN];
QueueHandle_t TxCOM6;
QueueHandle_t RxCOM6;
void Device_Usart6_ENABLE_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStruct;
	DMA_InitTypeDef DMA_InitStructure;
	
	USART_StructInit(&USART_InitStructure);
	USART_ClockStructInit(&USART_ClockInitStruct);
	{
		//开启USART6时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);	
		//开启USART6时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		  
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		USART_InitStructure.USART_BaudRate = bound;//波特率
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为 8 位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
		USART_Init(USART6, &USART_InitStructure); //初始化串口
	}
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
		USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
		//串口6接收DMA设置
		DMA_DeInit(DMA2_Stream1);
		DMA_StructInit(&DMA_InitStructure);
		DMA_InitStructure.DMA_Channel = DMA_Channel_5;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)& _USART6_DMA_RX_BUF[0][0];
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	//外设到储存器
		DMA_InitStructure.DMA_BufferSize = BSP_USART6_DMA_RX_BUF_LEN;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream1, &DMA_InitStructure);	
		//串口6发送DMA设置
		DMA_DeInit(DMA2_Stream7);
		DMA_StructInit(&DMA_InitStructure);
		DMA_InitStructure.DMA_Channel = DMA_Channel_5;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);  //外设地址
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)fff;//(uint32_t)(Pc_Send_Data.PcDataArray); //储存器0地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  //储存器到外设
		DMA_InitStructure.DMA_BufferSize = 1;//TO_PC_LENGTH;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream7, &DMA_InitStructure);
	}
	{
		//配置Memory1,Memory0是第一个使用的Memory
		DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)&_USART6_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
		DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);   
	}
	{
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);        //usart IDLE line interrupt  enabled
		USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
		USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
		USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
		USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
		USART_ClockInit(USART6, &USART_ClockInitStruct);
	}
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;   //上位机接收中断                      
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PC_NVIC;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&NVIC_InitStructure);
	}
	{
		/* 创建SEND消息队列 */
		TxCOM6 = xQueueCreate(50, sizeof(DataSend));
		if( TxCOM6 == 0 ){/* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
		}
		/* 创建REVICE消息队列 */
		RxCOM6 = xQueueCreate(50, sizeof(DataRevice));
		if( RxCOM6 == 0 ){/* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
		}
	}
		USART_Cmd(USART6, ENABLE); 
		DMA_Cmd(DMA2_Stream1, ENABLE);
}

void USART6_IRQHandler()
{
	static uint32_t this_time_rx_len = 0;
	DataRevice Buffer;
	BaseType_t  TaskWoken;
	
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{

		(void)USART6->SR;
		(void)USART6->DR;
//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)
		{
		DMA_Cmd(DMA2_Stream1, DISABLE);
		this_time_rx_len = BSP_USART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
		DMA2_Stream1->NDTR = (uint16_t)BSP_USART6_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
		DMA2_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
		DMA_Cmd(DMA2_Stream1, ENABLE);
			if(this_time_rx_len == MINIPC_FRAME_LENGTH)
			{ 			
			 Buffer.len=MINIPC_FRAME_LENGTH;
			 memcpy(Buffer.buffer,_USART6_DMA_RX_BUF[0],MINIPC_FRAME_LENGTH);
			 if(PcDataCheck(Buffer.buffer)==1)
				{
				if(verify_crc8_check_sum(&Buffer.buffer[1],13)!=NULL)
					{
						xQueueSendFromISR(TxCOM6,&Buffer,&TaskWoken);	
					}
					DetectHook(TX2DataTOE);
					
				}
				else 
				{
					return; 
				}
			}
		}
//Target is Memory1
		else  
		{
			DMA_Cmd(DMA2_Stream1, DISABLE);
			this_time_rx_len = BSP_USART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
			DMA2_Stream1->NDTR = (uint16_t)BSP_USART6_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream1->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream1, ENABLE);
			if(this_time_rx_len == MINIPC_FRAME_LENGTH)
			{	
			Buffer.len=MINIPC_FRAME_LENGTH;
			memcpy(Buffer.buffer,_USART6_DMA_RX_BUF[1],MINIPC_FRAME_LENGTH);
				if(PcDataCheck(Buffer.buffer)==1)
				{
				if(verify_crc8_check_sum(&Buffer.buffer[1],13)!=NULL)
				 
					{
						xQueueSendFromISR(TxCOM6,&Buffer,&TaskWoken);	
					}
					
				 DetectHook(TX2DataTOE);

				}
				else 
				{
					return; 
				}
			}
		}
	}
	USART_ClearITPendingBit(USART6, USART_IT_IDLE);	
}
