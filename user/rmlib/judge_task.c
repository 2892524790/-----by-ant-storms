#include "Judge_Task.h"
#include "judgement_info.h"
#include "RM_Client_UI.h"
#define JUDGE_BUFFER_LEN 		100
extern QueueHandle_t TxCOM1;
extern QueueHandle_t RxCOM1;

u8 Judge_Buffer[JUDGE_BUFFER_LEN];
u8 	Judgmessage;
Judge_FLAG Judge_Flag;


//读取裁判系统
void Judge_task(void* pvParameters)
{

    while(1)
    {

        xQueueReceive(TxCOM1, &Judgmessage, portMAX_DELAY);

        if(Judge_Flag.Flag == 0 && Judgmessage == FRAME_HEADER)	//帧头的第一个字节
        {
            Judge_Flag.Flag = 1;
            Judge_Buffer[0] = Judgmessage;
        }
        else if(Judge_Flag.Flag == 1)				//长度
        {
            Judge_Flag.Flag = 2;
            Judge_Buffer[1] = Judgmessage;
        }
        else if(Judge_Flag.Flag == 2)				//长度
        {
            Judge_Flag.Flag = 3;
            Judge_Buffer[2] = Judgmessage;
            Judge_Flag.data_len = (u16)(Judgmessage << 8) | Judge_Buffer[1] + LEN_CMDID + LEN_TAIL;
            if(Judge_Flag.data_len > 100)
            {
                Judge_Flag.Flag = 0;
            }
        }
        else if(Judge_Flag.Flag == 3)				//包序号
        {
            Judge_Flag.Flag = 4;
            Judge_Buffer[3] = Judgmessage;
        }
        else if(Judge_Flag.Flag == 4)				//CRC8
        {
            Judge_Flag.Flag = 5;
            Judge_Buffer[4] = Judgmessage;
            if(verify_crc8_check_sum(Judge_Buffer, HEADER_LEN) != NULL)	//CRC8校验
            {
                Judge_Flag.data_cnt = 0;
            }
            else				//校验没有通过，重新开始
            {
                Judge_Flag.data_cnt = 0;
                Judge_Flag.Flag = 0;
            }
        }
        else if(Judge_Flag.Flag == 5 && Judge_Flag.data_len > 0)	//开始接受数据
        {
            Judge_Flag.data_len--;
//			Judge_Flag.data_cnt=Judge_Flag.data_cnt+1;
            Judge_Buffer[LEN_HEADER + Judge_Flag.data_cnt++] = Judgmessage;
            if(Judge_Flag.data_len == 0)
            {
                Judge_Flag.Flag = 6;
            }
        }
        else
        {
            Judge_Flag.Flag = 0;
            Judge_Flag.data_len = 0;
            Judge_Flag.data_cnt = 0;
        }
        if(Judge_Flag.Flag == 6)
        {
            Judge_DataVerify(Judge_Buffer);		//帧头部分检测完毕，读取裁判系统数据
            Judge_Flag.Flag = 0;								//其他数据清零
            Judge_Flag.data_len = 0;
            Judge_Flag.data_cnt = 0;
        }
    }
}





//裁判系统数据解析
void Judge_DataVerify(u8* Buff)
{
    Dateframe_t*	frame;
    if(Buff != NULL)
    {

        frame = (Dateframe_t*)Buff;
        //将进行帧头与整帧数据CRC校验  帧头CRC8  整帧CRC16
        if(verify_crc16_check_sum((uint8_t*)frame, HEADER_LEN + CMD_LEN + frame->FrameHeader.DataLength + CRC_LEN))
        {
            judgement_data_handler(Buff);  //通过校验进行数据解析
        }

    }

}




