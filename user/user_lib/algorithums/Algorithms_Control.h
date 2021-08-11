#ifndef _ALGORITHMS_CONTROL_H_
#define _ALGORITHMS_CONTROL_H_
 

#include "stm32f4xx.h"                  // Device header
#include "PID_Control.h"

#define ID_dKp 1
#define ID_dKi 2
#define ID_dKd 3
 
 
typedef struct Algorithms{
	
 u16 MaxMotorSpeed;
 float Ge;
 float Gec;
 float Gkp;
 float Gki;
 float Gkd;
	
} Algori_Control;

static const float levelInterval = 5;//����ķּ����������ͬ���˴�����Ϊ5
static const float NL = -3 * levelInterval;
static const float NM= -2 * levelInterval;
static const float NS= -1 * levelInterval;
static const float ZE= 0;
static const float PS= 1 * levelInterval;
static const float PM= 2 * levelInterval;
static const float PL= 3 * levelInterval;
 
/************************************
dKpģ�����ƹ����
dKpeNLNMNSZEPSPMPL
ec
NLPLPLPMPMPSPSZE
NMPLPLPMPMPSZEZE
NSPMPMPMPSZENSNM
ZEPMPSPSZENSNMNM
PSPSPSZENSNSNMNM
PMZEZENSNMNMNMNL
PLZENSNSNMNMNLNL
************************************
dKiģ�����ƹ����
dKieNLNMNSZEPSPMPL
ec
NLNLNLNLNMNMZEZE
NMNLNLNMNMNSZEZE
NSNMNMNSNSZEPSPS
ZENMNSNSZEPSPSPM
PSNSNSZEPSPSPMPM
PMZEZEPSPMPMPLPL
PLZEZEPSPMPLPLPL
************************************
dKdģ�����ƹ����
dKdeNLNMNSZEPSPMPL
ec
NLPSPSZEZEZEPLPL
NMNSNSNSNSZENSPM
NSNLNLNMNSZEPSPM
ZENLNMNMNSZEPSPM
PSNLNMNSNSZEPSPS
PMNMNSNSNSZEPSPS
PLPSZEZEZEZEPLPL
************************************/
 
static const float fuzzyRuleKp[7][7]={
 
PL,PL,PM,PM,PS,PS,ZE,
PL,PL,PM,PM,PS,ZE,ZE,
PM,PM,PM,PS,ZE,NS,NM,
PM,PS,PS,ZE,NS,NM,NM,
PS,PS,ZE,NS,NS,NM,NM,
ZE,ZE,NS,NM,NM,NM,NL,
ZE,NS,NS,NM,NM,NL,NL
 
};//dKpģ�������
 
static const float fuzzyRuleKi[7][7]={
 
NL,NL,NL,NM,NM,ZE,ZE,
NL,NL,NM,NM,NS,ZE,ZE,
NM,NM,NS,NS,ZE,PS,PS,
NM,NS,NS,ZE,PS,PS,PM,
NS,NS,ZE,PS,PS,PM,PM,
ZE,ZE,PS,PM,PM,PL,PL,
ZE,ZE,PS,PM,PL,PL,PL
 
};//dKiģ�������
 
static const float fuzzyRuleKd[7][7]={
 
PS,PS,ZE,ZE,ZE,PL,PL,
NS,NS,NS,NS,ZE,NS,PM,
NL,NL,NM,NS,ZE,PS,PM,
NL,NM,NM,NS,ZE,PS,PM,
NL,NM,NS,NS,ZE,PS,PS,
NM,NS,NS,NS,ZE,PS,PS,
PS,ZE,ZE,ZE,ZE,PL,PL
 
};//dKdģ�������
 
typedef struct {
 
float Kp;
float Ki;
float Kd;
 
}sPID;
 
void PidControler(Algori_Control *AL,PID* motor_type);
void  AlgoriCreate(Algori_Control *AL,u16 MaxMotorSpeed,u16 Gkp,u16 Gki,u16 Gkd);
void  AlgoriReset(Algori_Control *AL);
#endif
