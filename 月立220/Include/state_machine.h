/*******************************************************************************
 * ��Ȩ���� (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� state_machine.h
 * ����ժҪ�� state machine
 * ����˵���� ��
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2019��11��19��
 *
 *******************************************************************************/
#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

#include "basic.h"
#include "mc_type.h"

typedef struct 
{
    s32 wStartCurSet1;              //��һ�ζ�λ����
    s32 wStartCurSet2;              //�ڶ��ζ�λ����
    s32 wSvcMinFreq;                //�����϶�Ƶ��
    u16 nStartAngleComp;            //��ʼλ�ò����Ƕ�

    u8  bChargeFlag;                //Ԥ����־λ
    u8  bChargeEndFlag;             //Ԥ�����ɱ�־λ

    s16 nDirTrackFreq;              //�������ջ�Ƶ��
    s16 nDirEbreakFreq;             //���ɲ��Ƶ��
    s16 nCurrentACC;                //�������ٵ���ֵ
    s16 nCurrentDEC;                //�������ٵ���ֵ
    s32 wSpeedRef;                  //�ٶȸ���ֵ
    
    s16 nLoopCntr;                  //��·����

    u8  bOpenRunFlag;               //�������б�־λ
    s32 wThetaErr;                  //����͸����Ƕ����
    s16 nMatchCnt;                  //����Ƕ�ƥ�����

    s16 nPowerFistFlag;             //��һ�ν��빦�ʻ���־λ

    s16 nTempADC;                   //NTC����ADֵ
    
    u16 nMotorRunDirPre;            //�ϴε��ת��
    
    s32 wPWMPowerSet;              /* PWM���ٸ������� */
    
    s32 wPWMSpeedSet;              /* PWM���ٸ���ת�� */
    
    s16 nVspADC;                   /* VSP������ѹ */
    s32 wVSPSpeedSet;              /* VSP��������ת�� */
    
    s32 wPIMatchSpeed;           /* ��������PLL��������ת��*/
    
    s32 wCloseSpeed;          /* ���빦�ʻ���Ƶ�� */
}stru_CommData;

typedef struct
{
    s16 nChargeTime;       //���ʱ�䣬��1msΪ��λ
    s16 nAlignTime;        //��λʱ�䣬��1msΪ��λ
    s16 nDirCheckTime;     //˳���ʱ�䣬��1msΪ��λ
    s16 nLoopDelyTime;     //�ٶȻ��ӳ�ʱ�䣬��1msΪ��λ
    s16 nStopDelayTime;    //���ֹͣ�ӳ�ʱ�䣬��1msΪ��λ
    s16 nStopDelayCntr;    //���ֹͣ�˲�ʱ�䣬�������жϵ��ֹͣ��ʱ�䣬��1msΪ��λ
    s16 nOpenRunCntr;      //�����бջ�ʱ�䣬��1msΪ��λ
}stru_Time_t;

extern stru_Time_t                 struTime;

extern void StateInit(void);
extern void StateCalib(void);
extern void StateCharge(void);
extern void StateDirCheck(void);
extern void StatePosSeek(void);
extern void StateAlign(void);
extern void StateOpen(void);
extern void StateRun(void);
extern void StateFault(void);
extern void StateIdle(void);
extern void StateStop(void);

extern void FluxObserveInit(void);
extern void MatchPICoef(void);
#endif

