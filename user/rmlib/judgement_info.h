#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__

#include "stm32f4xx.h"                  // Device header

#define		JUDGE_21		21			//21年裁判系统
#define		JUDGE_VERSION	JUDGE_21

#if JUDGE_VERSION==JUDGE_21

//帧头详细定义
#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16

/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;				//起始字节，固定0XA5
	uint16_t DataLength;//帧数据长度
	uint8_t  Seq;				//包序号
	uint8_t  CRC8;			//CRC8校验值
	u16 CMD_ID;					//命令ID
} xFrameHeader;// UI

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  2    比赛机器人存活数据   		1Hz发送  
	ID: 0x0004  Byte:  3    飞镖发射状态数据   			  
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0102  Byte:  3    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0103  Byte:  2    场地补给站预约子弹数据      参赛队发送，10Hz 
	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID  0x0208  Byte:  2    子弹剩余发射数据            10Hz
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/

//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       			= 0x0001,//比赛状态数据
	ID_game_result 	   				= 0x0002,//比赛结果数据
	ID_game_robot_survivors       	= 0x0003,//比赛机器人存活数据
	ID_game_dart_status      		= 0x0004,//比赛飞镖状态数据	
	ID_event_data  					= 0x0101,//场地事件数据 
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
	ID_supply_projectile_booking 	= 0x0103,//场地补给站预约子弹数据
	ID_supply_dart_remaing 			= 0x0105,//飞镖发射口倒计时数据
	ID_game_robot_state    			= 0x0201,//机器人状态数据
	ID_power_heat_data    			= 0x0202,//实时功率热量数据
	ID_game_robot_pos        		= 0x0203,//机器人位置数据
	ID_buff_musk					= 0x0204,//机器人增益数据
	ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
	ID_robot_hurt					= 0x0206,//伤害状态数据
	ID_shoot_data					= 0x0207,//实时射击数据
	ID_bullet_remaing				= 0x0208,//子弹剩余发射量数据
	ID_ROBORT_COM_DATA				=0X0301,//10HZ
} CmdID;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;				//比赛类型
	uint8_t game_progress : 4;		//当前比赛状态
	uint16_t stage_remain_time;		//当前阶段剩余时间
} ext_game_state_t; 

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  2    比赛机器人存活数据 */
typedef __packed struct 
{ 
	uint16_t robot_legion;
} ext_game_robot_survivors_t; 

/* ID: 0x0004  Byte:  3    比赛飞镖发射状态数据 */
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 

/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t; 

/* ID: 0X0103  Byte:  2    场地补给站预约子弹数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 

/* ID: 0X0105  Byte:  1    飞镖发射口倒计时 */
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 15    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;   //机器人ID，可用来校验发送
	uint8_t robot_level;  //1一级，2二级，3三级
	uint16_t remain_HP;  //机器人剩余血量
	uint16_t max_HP; //机器人满血量
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;

	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t; 

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 

/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type;   
	uint8_t bullet_freq;   
	float bullet_speed;  
} ext_shoot_data_t; 

/* ID: 0x0208  Byte:  2    子弹剩余数据 */

typedef __packed struct
{
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef struct 
{
  uint8_t	Flag;				//帧头位数
	uint16_t data_len;	//数据长度
	uint16_t data_cnt;		//自加位置
	uint8_t	data;
}Judge_FLAG;


/* 
	
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。 
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。 
*/

typedef __packed struct
{
uint16_t data_cmd_id;//内容ID
uint16_t send_ID;//发送者ID
uint16_t receiver_ID;//接受者ID
}ext_student_interactive_header_data_t;//操作定义帧


typedef __packed struct
{
	ext_student_interactive_header_data_t interactive_data;
  float data1;
  float data2;
  float data3;
	u8    mask;
} client_show_data_t;

typedef __packed struct
{
	xFrameHeader		FrameHeader;
	CmdID		CmdID;

  __packed union
	{ 
		ext_game_robot_state_t game_information;
		ext_robot_hurt_t  blood_changed_data;
		ext_shoot_data_t       real_shoot_data;
		ext_power_heat_data_t   real_powerheat_data;
		ext_event_data_t      rfid_data;
		ext_game_result_t      game_result_data;
		ext_buff_musk_t         get_buff_data;
		ext_game_robot_pos_t		 gameRobotPos;
		client_show_data_t  client_show_data;
	}Data;
	uint16_t		CRC16;	//所有数据CRC校验
}Dateframe_t;//数据帧

/* 交互数据接收信息：0x0301  */

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz


	1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		送者的 ID 			需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		数据的内容 ID 	0x0200~0x02FF 
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性， 
	
	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	
	6 			n 		数据段 			n 需要小于 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //数据段,n需要小于113
} robot_interactive_data_t;

/*-------------1. 交互数据接收信息： 0x0301。 发送频率：上限 10Hz---------
字节偏移量 大小 说明 备注
0 2 数据段的内容ID
2 2 发送者的ID   需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
4 2 接收者的ID  需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID
6 x 内容数据段x 最大为 113
----*/

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
	xFrameHeader   					txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	client_custom_data_t  					clientData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_t;


//图形
typedef __packed struct {
	uint8_t graphic_name[5]; 
	uint32_t operate_tpye:3;  
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9; 
	uint32_t end_angle:9; 
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10;  
	uint32_t end_x:11;  
	uint32_t end_y:11;  
 
	uint32_t text_lenght;
	uint32_t text[30];
} ext_client_graphic_draw_t;


typedef __packed struct
{
	ext_student_interactive_header_data_t interactive_data;

ext_client_graphic_draw_t  graphic_draw;
} client_show_graph_t;

typedef __packed struct
{
  uint8_t  data[64];
} user_to_server_t;

typedef __packed struct
{
  uint8_t  data[32];
} server_to_user_t;

/** 
  * @brief  the data structure receive from judgement
  */
/* data send (forward) */
/* data receive */


#define INFANTRY 1
#define HERO 2
#define ENGINEER 3
#define SENTINEL 4
typedef struct
{
	int16_t Voltage;
	int16_t Current;
	int16_t Supply_Num;
	int16_t ShooterHeat_17mm;
	int16_t ShooterHeat_42mm;
	int16_t Power;
	int16_t PowerBuffer;
	u8 level;
	u8 hurt_type;
}JudgementType;

typedef struct
{
	u8 graph_operate_type;
	u8 graph_type;
	u8 graph_name[5];
	u8 graph_color;
	u8 graph_line_width;
	u16 graph_start_x;
	u16 graph_start_y;
	u16 graph_radius;
	u16 graph_dst_x;
	u16 graph_dst_y;
	uint8_t text_lengh;
	uint8_t text[30];
}Graph_Data_Type;

typedef struct
{
	int16_t ShootLevel;
	int16_t SuperCapacitorComment;
	float bullet_can_shoot;
	u8 State_Mask;
	u8 SuperCapacitorState;
	Graph_Data_Type Graph_Data ;
}SendToJudgementDataType;

//机器人交互信息
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;


#endif


void  judgement_data_handler(uint8_t *p_frame);
void determine_ID(void);

#endif
