#include "Tx2_task.h"
#include "gimbal_task.h"
#include "protocol.h"

static void PcDataClean(unsigned  char* pData, int num);
static void UART_PutChar(USART_TypeDef* USARTx, u8 ch);
static void MinipcDatePrcess(PC_Ctrl_t* pData);
PC_send_measure_t SEND_PC;
PC_Ctrl_Union_t PcData;
extern QueueHandle_t TxCOM6;
extern QueueHandle_t RxCOM6;
extern Gimbal_Control_t gimbal_control;
extern char mouse_r;
extern ext_game_robot_state_t GameRobotStat;
send_Tx2_t Robot_State;
char key_b = 0, flag_b = 0;
u8 robot__id;
void Tx2_task(void*  pvParameters)
{
    DataRevice Buffer;
    while(1)
    {
        xQueueReceive(TxCOM6, &Buffer, portMAX_DELAY);
        memcpy(PcData.PcDataArray, Buffer.buffer, MINIPC_FRAME_LENGTH);
    }
}
void SEND_PC_TASK(void*  pvParameters)
{
    while(1)
    {
       Send_to_PC(USART6);
			 vTaskDelay(5);
    }
}
//返回PC接收端变量地址，通过指针方式获取原始数据-----通过返回数据控制云台
const PC_Ctrl_Union_t* get_PC_Ctrl_Measure_Point(void)
{
    return &PcData;
}
//PC数据校验函数
uint8_t PcDataCheck(uint8_t* pData)
{
    if(pData[0] == 0xA5)/*&& (pData[MINIPC_FRAME_LENGTH - 1] == 0x5A)*/
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
//数据清零
static void PcDataClean(unsigned  char* pData, int num)
{
    if(pData == NULL)
    {
        return;
    }
    int i = 0;
    for(i = 0; i < num; i++)
    {
        pData[i] = 0;
    }
}
//上位机数据解码
static void MinipcDatePrcess(PC_Ctrl_t* pData)
{
    static PC_Ctrl_t PC_CtrlData;     //PC control data
    if(pData == NULL)
    {
        return ;
    }
    PC_CtrlData.angle_yaw = pData->angle_yaw;
    PC_CtrlData.angle_pitch = pData->angle_pitch;

//	PC_CtrlData.Z=pData->Z;
}
char x=0;
extern ext_game_robot_state_t			GameRobotStat;
union SendData {
uint8_t c[4];
float angle;
} Date1,Date2,Date3,Date4;
unsigned char TX_data[19];
void Send_to_PC(USART_TypeDef* USARTx)
{
		SEND_PC.yaw=Date1.angle=gimbal_control.gimbal_yaw_motor.absolute_angle;
		SEND_PC.pitch=Date2.angle=gimbal_control.gimbal_pitch_motor.absolute_angle;
		 
		SEND_PC.yaw_add= 0;Date3.angle=0;
		SEND_PC.pit_add= 0;Date4.angle=0;
	
    robot__id = GameRobotStat.robot_id;
    if(robot__id < 9)
    {
        SEND_PC.color = BLUE ;

    }
    else if(robot__id >= 9 && robot__id <= 109)
    {
        SEND_PC.color = RED ;
    }
    else
    {
        SEND_PC.color = ERROR_COLOR ;
    }
		
		if(IF_KEY_PRESSED_B && key_b == 0)
		{
				key_b = 1;
			if(mouse_r==0x01)
				flag_b++;
			else 
				flag_b=0;
			
			if(flag_b==2) flag_b=0;
		}
		else if(!IF_KEY_PRESSED_B)
				key_b = 0;
		

		switch(flag_b)
		{
			case 0: SEND_PC.mode=0; break;
			case 1: SEND_PC.mode=3; break;
		}
		SEND_PC.speed=GameRobotStat.shooter_id1_42mm_speed_limit;
		TX_data[0]=Date1.c[0];
		TX_data[1]=Date1.c[1];
		TX_data[2]=Date1.c[2];
		TX_data[3]=Date1.c[3];
		TX_data[4]=Date2.c[0];
		TX_data[5]=Date2.c[1];
		TX_data[6]=Date2.c[2];
		TX_data[7]=Date2.c[3];
		TX_data[8]= SEND_PC.mode;
		TX_data[9]=	SEND_PC.color;
		TX_data[10]=SEND_PC.speed;
		TX_data[11]=Date3.c[0];
		TX_data[12]=Date3.c[1];
		TX_data[13]=Date3.c[2];
		TX_data[14]=Date3.c[3];
		TX_data[15]=Date4.c[0];
		TX_data[16]=Date4.c[1];
		TX_data[17]=Date4.c[2];
		TX_data[18]=Date4.c[3];
		
		SEND_PC.CRC8=get_crc8_check_sum(TX_data,19,0xff); 
    UART_PutChar(USARTx, 0x5a);
		UART_PutChar(USARTx, Date1.c[0]);
    UART_PutChar(USARTx, Date1.c[1]);
    UART_PutChar(USARTx, Date1.c[2]);
    UART_PutChar(USARTx, Date1.c[3]);
	  UART_PutChar(USARTx, Date2.c[0]);
    UART_PutChar(USARTx, Date2.c[1]);
    UART_PutChar(USARTx, Date2.c[2]);
    UART_PutChar(USARTx, Date2.c[3]);
		UART_PutChar(USARTx, SEND_PC.mode);
    UART_PutChar(USARTx, SEND_PC.color);
		UART_PutChar(USARTx, SEND_PC.speed);
		UART_PutChar(USARTx, Date3.c[0]);
    UART_PutChar(USARTx, Date3.c[1]);
    UART_PutChar(USARTx, Date3.c[2]);
    UART_PutChar(USARTx, Date3.c[3]);
	  UART_PutChar(USARTx, Date4.c[0]);
    UART_PutChar(USARTx, Date4.c[1]);
    UART_PutChar(USARTx, Date4.c[2]);
    UART_PutChar(USARTx, Date4.c[3]);
    UART_PutChar(USARTx, SEND_PC.CRC8);
}
static void UART_PutChar(USART_TypeDef* USARTx,u8 ch)
{
	USART_SendData(USARTx, (u8)ch);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
}
