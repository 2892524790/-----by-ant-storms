//模糊控制函数 
#include "Algorithms_Control.h"
#include "math.h"
 
static float DeFuzzy(int eLevel,int ecLevel,u8 ID_item,Algori_Control* AL)  //重心法解模糊
{
 
	switch(ID_item)
	{
	 
		case ID_dKp:
		return fuzzyRuleKp[ecLevel+3][eLevel+3] / AL->Gkp;
		 
		case ID_dKi:
		return fuzzyRuleKi[ecLevel+3][eLevel+3] / AL->Gki;
		 
		case ID_dKd:
		return fuzzyRuleKd[ecLevel+3][eLevel+3] / AL->Gkd;
		 
		default:
		return 0;
	 
	}
 
}
 
static sPID Fuzzifier(float e, float ec,Algori_Control* AL)
{
 
	int eLeftIndex,eRightIndex,ecLeftIndex,ecRightIndex;   //误差模糊等级：exxx：偏差，ecxxx：偏差变化率
	float eLeftMs,eRightMs,ecLeftMs,ecRightMs;						 //隶属度
	sPID fuzzyDetPID;																			 //PID参数结构体
	 
	//*****感觉应该是偏差标量化，但是他除以1.0有点搞不懂这句******或许他测得的偏差就属于0~1内不用标量化
	e /= AL->Ge;
	ec /= AL->Gec;
	//马丹妮模糊推理   确定模糊等级(整数)  
	//采集偏差e左侧模糊等级 和 偏差e右侧模糊等级
	eLeftIndex = (e/levelInterval)>3.0f?3:(e/levelInterval)<-3.0f?-4:(e/levelInterval)>0?(int)(e/levelInterval):(int)(e/levelInterval)-1;
	eRightIndex = eLeftIndex + 1;
	//计算偏差左侧和右侧隶属度
	eLeftMs = eLeftIndex<-3?0:eLeftIndex==3?1.0f:eRightIndex-e/levelInterval;
	eRightMs = eRightIndex>3?0:eRightIndex==-3?1.0f:e/levelInterval-eLeftIndex;
	//采集偏差变化率ec左侧模糊等级 和 偏差变化率ec右侧模糊等级 
	ecLeftIndex = (ec/levelInterval)>3.0f?3:(ec/levelInterval)<-3.0f?-4:(ec/levelInterval)>0?(int)(ec/levelInterval):(int)(ec/levelInterval)-1;
	ecRightIndex = ecLeftIndex + 1;
	//计算偏差变化率左侧和右侧隶属度 
	ecLeftMs = ecLeftIndex<-3?0:ecLeftIndex==3?1.0f:ecRightIndex-ec/levelInterval;
	ecRightMs = ecRightIndex>3?0:ecRightIndex==-3?1.0f:ec/levelInterval-ecLeftIndex;
	//重心法解模糊  得出值即为模糊控制器输出 
	fuzzyDetPID.Kp = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKp,AL)
	+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKp,AL)
	+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKp,AL)
	+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKp,AL));
	fuzzyDetPID.Ki = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKi,AL)
	+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKi,AL)
	+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKi,AL)
	+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKi,AL));
	fuzzyDetPID.Kd = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKd,AL)
	+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKd,AL)
	+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKd,AL)
	+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKd,AL));
 
return fuzzyDetPID;
 
}
//创建一个模糊控制器
void  AlgoriCreate(Algori_Control *AL,u16 MaxMotorSpeed,u16 Gkp,u16 Gki,u16 Gkd)
{
     AL->MaxMotorSpeed=MaxMotorSpeed;
	   AL->Ge=1.0;
		 AL->Gec=1.0;
		 AL->Gkp=Gkp;
		 AL->Gki=Gki;
		 AL->Gkd=Gkd;
}
void  AlgoriReset(Algori_Control *AL)
{
     AL->MaxMotorSpeed=0;
	   AL->Ge=1.0;
		 AL->Gec=1.0;
		 AL->Gkp=1.0;
		 AL->Gki=1.0;
		 AL->Gkd=1.0;
}
void  PidControler(Algori_Control *AL,PID* motor_type)
{
 
		sPID dPID = {
		0, 0, 0
	};				//动态PID调节————模糊PID输出，每次进入函数清零动态PID
	//电机转速限幅		
	motor_type->ref = motor_type->ref>AL->MaxMotorSpeed?AL->MaxMotorSpeed:motor_type->ref<-AL->MaxMotorSpeed?-AL->MaxMotorSpeed:motor_type->ref;
	
	motor_type->error_last=motor_type->error_now;
	motor_type->error_now = motor_type->ref - motor_type->fdb;
	motor_type->error_rate=motor_type->error_now- motor_type->error_last;
	motor_type->error_inter += motor_type->error_now;	

		// limit intergration of pid
	if(motor_type->error_inter>motor_type->Inter_Max)
		  motor_type->error_inter = motor_type->Inter_Max;
	if(motor_type->error_inter<-motor_type->Inter_Max)
		  motor_type->error_inter = -motor_type->Inter_Max;
	
	if(fabs(motor_type->error_now/motor_type->ref)<0.05)//误差小于5%使用模糊控制PID
	dPID = Fuzzifier(motor_type->error_now, motor_type->error_rate,AL); 
	
  motor_type->pid_out = (motor_type->Kp+dPID.Kp) * motor_type->error_now+	(motor_type->Kd+dPID.Kd) * motor_type->error_rate +(motor_type->Ki+dPID.Ki) * motor_type->error_inter ;	
   	 
}
