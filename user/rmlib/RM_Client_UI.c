/*************************************************************
  ****************************(C) COPYRIGHT 2021 NCIST****************************
  * @file       UI_task.c/h
  * @brief      上位机通信任务，视觉数据处理
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     2021          	zhuxunfu              finished
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 NCIST****************************
  */
#include "GIMBAL_TASK.h"
#include "SHOOT_TASK.h"
#include "chassis_task.h"
#include "RM_Client_UI.h"
#include "protocol.h"
#include "adc.h"
#include "main.h"



#define Get_CRC16_Check_Sum_UI get_crc16_check_sum
#define Get_CRC8_Check_Sum_UI get_crc8_check_sum 
 
char k_v = 0, f_v = 0;
extern  u8 flag_g;
extern Gimbal_Control_t gimbal_control;
extern chassis_move_t chassis_move;
extern Shoot_t shoot;
extern ext_game_robot_state_t			  	GameRobotStat;				//0x0201
extern char mouse_r; //自瞄标志位
extern u32 Robot_ID,Cilent_ID;
extern char flag_r;
extern PC_send_measure_t SEND_PC;
fp32 x0_ui,y0_ui,x1_ui,y1_ui;
String_Data CH_FRICTION,CH_GIMBAL,CH_CHASSIC,PIT;
Graph_Data S1,S2,S3;
Graph_Data G1,G2,G3,G4,G5,G6,G7,G8;
Graph_Data F1,F2,F3,F4,F5,F6,F7,F8;
Graph_Data E1,E2,E3,E4,E5,E6,E7,E8;
Graph_Data D1,D2,D3,D4,D5,D6;
Graph_Data H1,H2,H3,H4,H5;

char trigger_on[10] = "TRIGGER:ON",trigger_off[11] = "TRIGGER:OFF";
char chassis_follow_mode[14]= "CHASSIC:FOLLOW",chassis_dodge_mode[13] = "CHASSIC:DODGE",chassic_sperate_mode[16]="CHASSIC:SEPERATE";
char track_auto[14] = "GIMBAL:Autoaim",track_normal[13] = "GIMBAL:Normal",track_sentry[13]="GIMBAL:Sentry";
char Pitch[7]={0};

void UI_task(void *pvParameters)
{	
	memset(&CH_CHASSIC,0,sizeof(CH_CHASSIC));
	memset(&CH_GIMBAL,0,sizeof(CH_GIMBAL));
	memset(&CH_FRICTION,0,sizeof(CH_FRICTION));	
	memset(&PIT,0,sizeof(PIT));
	memset(&S1,0,sizeof(S1));
	memset(&S2,0,sizeof(S2));
	memset(&S3,0,sizeof(S3));
	memset(&G8,0,sizeof(G8)); 
	memset(&G1,0,sizeof(G1)); 
	memset(&G2,0,sizeof(G2)); 
	memset(&G3,0,sizeof(G3)); 
	memset(&G4,0,sizeof(G4)); 
	memset(&G5,0,sizeof(G5)); 
	memset(&G6,0,sizeof(G6)); 
	memset(&G7,0,sizeof(G7));
 
	memset(&F1,0,sizeof(F1));
	memset(&F2,0,sizeof(F2));
	memset(&F3,0,sizeof(F3));
	memset(&F4,0,sizeof(F4));
	memset(&F5,0,sizeof(F5));
	memset(&F6,0,sizeof(F6));
	memset(&F7,0,sizeof(F7));
	memset(&F8,0,sizeof(F8));
	memset(&E1,0,sizeof(E1));
	memset(&E2,0,sizeof(E2));
	memset(&E3,0,sizeof(E3));
	memset(&E4,0,sizeof(E4));
	memset(&E5,0,sizeof(E5));
	memset(&E6,0,sizeof(E6));
	memset(&E7,0,sizeof(E7));
	memset(&E8,0,sizeof(E8));
	memset(&D1,0,sizeof(D1));
	memset(&D2,0,sizeof(D2));
	memset(&D3,0,sizeof(D3));
	memset(&D4,0,sizeof(D4));
	memset(&D5,0,sizeof(D5));
	memset(&D6,0,sizeof(D6));
	memset(&H1,0,sizeof(H1));
	memset(&H2,0,sizeof(H2));
	memset(&H3,0,sizeof(H3));
	memset(&H4,0,sizeof(H4));
 	for(;;)
	{		
				if(IF_KEY_PRESSED_V && k_v == 0)
			{
					k_v = 1;
					f_v = 0;
			}
			else if(!IF_KEY_PRESSED_V)
			{
				f_v =1;
				k_v = 0;
			}
			else f_v =1;

		
		if(GameRobotStat.robot_id>10)
		{	
			Robot_ID=GameRobotStat.robot_id;
			Cilent_ID=0x0165;
		}
		else 
		{//red
			Robot_ID=GameRobotStat.robot_id;
			Cilent_ID=0x0101;
		}
#if (Hero_Robot==0)

		sprintf(Pitch,"%0.3f",(-gimbal_control.gimbal_pitch_motor.absolute_angle));
#else
		sprintf(Pitch,"%0.3f",(gimbal_control.gimbal_pitch_motor.absolute_angle));

#endif
		Char_Draw(&CH_FRICTION,"085",UI_Graph_ADD,1,UI_Color_Green,24,10,2,430,780,&trigger_on[0]);//摩擦轮状态
		
		Char_Draw(&CH_GIMBAL,"087",UI_Graph_ADD,1,UI_Color_Yellow,24,14,2,430,450,&track_auto[0]);
		
		Char_Draw(&CH_CHASSIC,"086",UI_Graph_ADD,1,UI_Color_Pink,24,16,2,420,700,&chassic_sperate_mode[0]);//底盘模式
		Char_Draw(&PIT,"088",UI_Graph_ADD,1,UI_Color_Cyan,24,7,2,500,350,&Pitch[0]);
		Line_Draw(&S1,"095",UI_Graph_ADD,1,UI_Color_Orange,10,1000,100,(Super_power.input_volt-17000)/10+1000,100);//超级电容
		Circle_Draw(&S2,"097",UI_Graph_ADD,9,UI_Color_Green,20,800,800,15);
		Line_Draw(&G7,"077",UI_Graph_ADD,9,UI_Color_White,1,990,140,990,340);
				Line_Draw(&H1,"023",UI_Graph_ADD,9,UI_Color_Cyan,2,980,220,1000,220);
				Line_Draw(&H2,"024",UI_Graph_ADD,9,UI_Color_White,1,980,200,1000,200);
				Line_Draw(&H3,"025",UI_Graph_ADD,9,UI_Color_Cyan,2,980,180,1000,180);
				Line_Draw(&H4,"026",UI_Graph_ADD,9,UI_Color_White,1,980,160,1000,160);
		Line_Draw(&G1,"071",UI_Graph_ADD,9,UI_Color_White,1,945,0,945,540);
		Line_Draw(&G2,"072",UI_Graph_ADD,9,UI_Color_Green,2,895,540,1025,540);
				Line_Draw(&F1,"001",UI_Graph_ADD,9,UI_Color_White,2,935,520,955,520);
				Line_Draw(&F2,"002",UI_Graph_ADD,9,UI_Color_White,2,935,500,955,500);
				Line_Draw(&F3,"003",UI_Graph_ADD,9,UI_Color_White,2,935,480,955,480);
				Line_Draw(&F4,"004",UI_Graph_ADD,9,UI_Color_White,2,935,460,955,460);
		
		Line_Draw(&G3,"073",UI_Graph_ADD,9,UI_Color_Green,2,895,440,1025,440);
				Line_Draw(&F5,"005",UI_Graph_ADD,9,UI_Color_Orange,2,935,420,955,420);
				Line_Draw(&F6,"006",UI_Graph_ADD,9,UI_Color_White,1,935,400,955,400);
				Line_Draw(&F7,"007",UI_Graph_ADD,9,UI_Color_White,1,935,380,955,380);
				Line_Draw(&F8,"008",UI_Graph_ADD,9,UI_Color_White,1,935,360,955,360);

		Line_Draw(&G4,"074",UI_Graph_ADD,9,UI_Color_Green,2,895,340,1025,340);
				Line_Draw(&E1,"009",UI_Graph_ADD,9,UI_Color_Orange,2,935,320,955,320);
				Line_Draw(&E2,"010",UI_Graph_ADD,9,UI_Color_Orange,2,935,300,955,300);
				Line_Draw(&E3,"011",UI_Graph_ADD,9,UI_Color_Purplish_red,2,935,280,955,280);
				Line_Draw(&E4,"012",UI_Graph_ADD,9,UI_Color_White,1,935,260,955,260);

		Line_Draw(&G5,"075",UI_Graph_ADD,9,UI_Color_Green,2,895,240,1025,240);
				Line_Draw(&E5,"013",UI_Graph_ADD,9,UI_Color_Cyan,2,935,220,955,220);
				Line_Draw(&E6,"014",UI_Graph_ADD,9,UI_Color_White,1,935,200,955,200);
				Line_Draw(&E7,"015",UI_Graph_ADD,9,UI_Color_Cyan,2,935,180,955,180);
				Line_Draw(&E8,"016",UI_Graph_ADD,9,UI_Color_White,1,935,160,955,160);

		Line_Draw(&G6,"076",UI_Graph_ADD,9,UI_Color_Green,2,895,140,1025,140);
				Line_Draw(&D1,"017",UI_Graph_ADD,9,UI_Color_White,1,935,120,955,120);
				Line_Draw(&D2,"018",UI_Graph_ADD,9,UI_Color_White,1,935,100,955,100);
				Line_Draw(&D3,"019",UI_Graph_ADD,9,UI_Color_White,1,935,80,955,80);
				Line_Draw(&D4,"020",UI_Graph_ADD,9,UI_Color_White,1,935,60,955,60);
				Line_Draw(&D5,"021",UI_Graph_ADD,9,UI_Color_White,1,935,40,955,40);
				Line_Draw(&D6,"022",UI_Graph_ADD,9,UI_Color_White,1,935,20,955,20);
				
		Line_Draw(&G8,"081",UI_Graph_ADD,1,UI_Color_Green,2,x0_ui,y0_ui,x1_ui,y1_ui);//底盘位置
		Circle_Draw(&S3,"082",UI_Graph_ADD,1,UI_Color_Green,2 ,1120,800, 40);

		if(f_v)
		{
/***************************************补弹情况*******************************************************/

	#if (Hero_Robot==0)

		#if (version_ball == 0)
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
			#else
		if(!((Get_Adc_Average(ADC_Channel_1,5)*3.3f/4096)<0.4f))

			#endif
	#else
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))

		#endif			
		Circle_Draw(&S2,"097",UI_Graph_Change,9,UI_Color_Purplish_red,20,800,800,15);
			else
			Circle_Draw(&S2,"097",UI_Graph_Change,9,UI_Color_Green,20,800,800,15);

/*****************************************摩擦轮*****************************************************/
			if(shoot.shoot_mode ==shoot_not_fire)
			{
				memset(&CH_FRICTION,0,sizeof(CH_FRICTION));
				Char_Draw(&CH_FRICTION,"085",UI_Graph_Change,1,UI_Color_Green,24,11,2,430,780,&trigger_off[0]);
			}
			else if(shoot.shoot_mode ==shoot_fire)
			{
				if(flag_r==0)
				{
					memset(&CH_FRICTION,0,sizeof(CH_FRICTION));	
					Char_Draw(&CH_FRICTION,"085",UI_Graph_Change,1,UI_Color_Green,24,10,2,430,780,&trigger_on[0]);
				}
				else if(flag_r)
				{
					memset(&CH_FRICTION,0,sizeof(CH_FRICTION));	
					Char_Draw(&CH_FRICTION,"085",UI_Graph_Change,1,UI_Color_Purplish_red,24,10,2,430,780,&trigger_on[0]);
				}
			}
/******************************************自瞄*****************************************************/
			if(mouse_r==1)
			{
				if(SEND_PC.mode==0)
				{		
					memset(&CH_GIMBAL,0,sizeof(CH_GIMBAL));	
					Char_Draw(&CH_GIMBAL,"087",UI_Graph_Change,1,UI_Color_Yellow,24,14,2,430,450,&track_auto[0]);
				}
				else if(SEND_PC.mode==3)
				{
					memset(&CH_GIMBAL,0,sizeof(CH_GIMBAL));	
					Char_Draw(&CH_GIMBAL,"087",UI_Graph_Change,1,UI_Color_Yellow,24,13,2,430,450,&track_sentry[0]);
				}
			}
			else if(mouse_r==0)
			{
				memset(&CH_GIMBAL,0,sizeof(CH_GIMBAL));
				Char_Draw(&CH_GIMBAL,"087",UI_Graph_Change,1,UI_Color_Yellow,24,13,2,430,450,&track_normal[0]);
			}
/*****************************************运动模式****************************************************/

			if(chassis_move.chassis_mode==CHASSIS_DODGE_MODE)
			{
				memset(&CH_CHASSIC,0,sizeof(CH_CHASSIC));
				Char_Draw(&CH_CHASSIC,"086",UI_Graph_Change,1,UI_Color_Pink,24,13,2,420,700,&chassis_dodge_mode[0]);
			}
			else if(chassis_move.chassis_mode==CHASSIS_FOLLOW_GIMBAL)
			{	
				if(flag_g)
				{
					memset(&CH_CHASSIC,0,sizeof(CH_CHASSIC));
					Char_Draw(&CH_CHASSIC,"086",UI_Graph_Change,1,UI_Color_Pink,24,16,2,420,700,&chassic_sperate_mode[0]);
				}
				else 
				{
					memset(&CH_CHASSIC,0,sizeof(CH_CHASSIC));
					Char_Draw(&CH_CHASSIC,"086",UI_Graph_Change,1,UI_Color_Pink,24,14,2,420,700,&chassis_follow_mode[0]);
				}
			}
/****************************************超级电容***************************************************/
			Line_Draw(&S1,"095",UI_Graph_Change,1,UI_Color_Orange,10,1000,100,(Super_power.input_volt-17000)/10+1000,100);
			memset(&PIT,0,sizeof(PIT));
			Char_Draw(&PIT,"088",UI_Graph_Change,1,UI_Color_Cyan,24,7,2,500,350,&Pitch[0]);
			
/****************************************底盘位置***************************************************/
			x0_ui=1120.0f;
			y0_ui=800.0f;
			x1_ui=x0_ui+arm_sin_f32(gimbal_control.gimbal_yaw_motor.relative_angle*ANGLE_TO_RAD)*40.0f;
			y1_ui=y0_ui+arm_cos_f32(gimbal_control.gimbal_yaw_motor.relative_angle*ANGLE_TO_RAD)*40.0f ;
			Line_Draw(&G8,"081",UI_Graph_Change,1,UI_Color_Green,2,x0_ui,y0_ui,x1_ui,y1_ui);
			Circle_Draw(&S3,"082",UI_Graph_Change,1,UI_Color_Green,2 ,1120,800, 40);
		}
/***************************************UI刷新***************************************************/
		Char_ReFresh(CH_FRICTION);
		Char_ReFresh(CH_GIMBAL);
		Char_ReFresh(CH_CHASSIC);
		Char_ReFresh(PIT);
		UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7);      
		UI_ReFresh(7,E1,E2,E3,E4,E5,E6,E7);      
		UI_ReFresh(7,F1,F2,F3,F4,F5,F6,F7);      
		UI_ReFresh(7,E8,F8,D1,D2,D3,D4,S1);
		UI_ReFresh(5,G8,D5,D6,S2,S3);			
		UI_ReFresh(5,H1,H2,H3,H4,H5);
  
		vTaskDelay(100);
	}
}


/****************************************************************************************/
unsigned char UI_Seq;                      //包序号
u32 Robot_ID,Cilent_ID;
/****************************************串口驱动映射************************************/
void UI_SendByte(unsigned char ch)
{
   USART_SendData(USART1,ch);
   while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	
}

/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(u8 Del_Operate,u8 Del_Layer)
{

   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   int loop_control;                       //For函数循环控制
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   UI_Data_Delete del;
   
   framepoint=(unsigned char *)&framehead;
   
   framehead.SOF=UI_SOF;
   framehead.Data_Length=8;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   datahead.Data_ID=UI_Data_ID_Del;
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   del.Delete_Operate=Del_Operate;
   del.Layer=Del_Layer;                                     //控制信息
   
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&del;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16校验值计算
   
   framepoint=(unsigned char *)&framehead;
   for(loop_control=0;loop_control<sizeof(framehead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(loop_control=0;loop_control<sizeof(datahead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&del;
   for(loop_control=0;loop_control<sizeof(del);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                                 //发送所有帧
   framepoint=(unsigned char *)&frametail;
   for(loop_control=0;loop_control<sizeof(frametail);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   UI_Seq++;                                                         //包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
        
void Line_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
        
void Rectangle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
        
void Circle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius)
{
   int i;
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}



/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
        
void Float_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->graphic_name[2-i]=imagename[i];
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = Graph_Float;
}



/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_x    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,char *Char_Data)
{
   int i;
   
   for(i=0;i<3&&imagename[i]!=0;i++)
      image->Graph_Control.graphic_name[2-i]=imagename[i];
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
   for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
   int i,n;
   Graph_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   
   for(i=0;i<cnt;i++)
   {
      imageData=va_arg(ap,Graph_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验
      
      for(n=0;n<sizeof(imageData);n++)
      {
         UI_SendByte(*framepoint);
         framepoint++;             
      }                                               //发送图片帧
   }
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   va_end(ap);
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}


/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
   int i;
   String_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   imageData=string_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   

   datahead.Data_ID=UI_Data_ID_DrawChar;

   datahead.Sender_ID=Robot_ID;
   datahead.Receiver_ID=Cilent_ID;                          //填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //发送操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //发送图片帧
   
   
   
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}


/*****************************************************CRC8校验值计算**********************************************/
//const unsigned char CRC8_INIT_UI = 0xff; 
//const unsigned char CRC8_TAB_UI[256] = 
//{ 
//0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
//0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
//0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
//0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
//0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
//0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
//0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
//0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
//0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
//0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
//0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
//0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
//0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
//0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
//0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
//0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
//};
//unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
//{ 
//unsigned char ucIndex; 
//while (dwLength--) 
//{ 
//ucIndex = ucCRC8^(*pchMessage++); 
//ucCRC8 = CRC8_TAB_UI[ucIndex]; 
//} 
//return(ucCRC8); 
//}

//uint16_t CRC_INIT_UI = 0xffff; 
//const uint16_t wCRC_Table_UI[256] = 
//{ 
//0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
//0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
//0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
//0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
//0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
//0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
//0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
//0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
//0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
//0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
//0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
//0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
//0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
//0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
//0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
//0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
//0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
//0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
//0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
//0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
//0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
//0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
//0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
//0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
//0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
//0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
//0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
//0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
//0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
//0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
//0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
//0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
//};
///* 
//** Descriptions: CRC16 checksum function 
//** Input: Data to check,Stream length, initialized checksum 
//** Output: CRC checksum 
//*/ 
//uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
//{ 
//Uint8_t chData; 
//if (pchMessage == NULL) 
//{ 
//return 0xFFFF; 
//} 
//while(dwLength--) 
//{ 
//chData = *pchMessage++;
//(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 
//0x00ff]; 
//} 
//return wCRC; 
//}

