#ifndef __CONFIG_H_
#define __CONFIG_H_

//定义步兵参数   
#define INFANTRY    2 


/** 
  * @brief  最多一次监测MAX_TASK_NUM个任务，如果多于该数，则需要定义多个事件标志组
  */
#define WDG_BIT_MINIPC_TASK	 		(1 << 0)
#define WDG_BIT_CONTROL_TASK	 	(1 << 1)
#define WDG_BIT_REMOTE_TASK	 		(1 << 2)
#define WDG_BIT_CAN1TX_TASK 		(1 << 3)
#define WDG_BIT_CAN1RX_TASK 		(1 << 4)
#define WDG_BIT_CAN2TX_TASK 		(1 << 5)
#define WDG_BIT_CAN2RX_TASK 		(1 << 6)
#define WDG_BIT_IMU_TASK 				(1 << 7)
#define WDG_BIT_DEFAULT_TASK 		(1 << 8)
#define WDG_BIT_TASK_ALL				(	WDG_BIT_MINIPC_TASK | WDG_BIT_CONTROL_TASK | WDG_BIT_REMOTE_TASK | WDG_BIT_CAN1TX_TASK| WDG_BIT_CAN1RX_TASK|WDG_BIT_DEFAULT_TASK|WDG_BIT_IMU_TASK)



#if INFANTRY==1  //英雄

#define Yaw_LeftOFFEST   0.0f
#define Yaw_RightOFFEST  0.0f  //上电超前角
#define CHASSISMAXPOWERRATE  0.90F
#define CHASSISMAXPOWER      120.0F       //底盘最大功率 

#elif INFANTRY==2  //工程


#endif

#endif
