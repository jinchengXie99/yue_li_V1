/*******************************************************************************
 * ��Ȩ���� (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� SpeedScan.h
 * �ļ���ʶ��
 * ����ժҪ�� ��λ�����١�PWM����
 * ����˵���� ��
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2019��12��27��
 *
 *******************************************************************************/
#ifndef __SPEEDSCAN_H
#define __SPEEDSCAN_H

#include "basic.h"
#include "FOC_Drive.h"
#include "PowerCalculation.h"


#define PWM_TIME_PERIOD      4800 //����Ƶ�ʶ�ӦTimer����ֵ����Բ���PWM��ʱ�����      4800--10K
#define DUTY_ZERO            0   //0ռ�ձ�
#define DUTY_FULL            1000//100%ռ�ձȣ���ֵ������10��

#define PWM_HIGH             (GPIO0_PDI & BIT9) //PWM����IO��ƽΪ�ߣ�һ����˵���ռ�ձ�Ϊ100%�����Ϊ0%

//#define DIR_FLITER           10//��λ��ms ���򰴼��˲�ʱ�䣬���ڰ���ȥ����ʱ�䳤�̾���������
//#define DIR_CW               (GPIO0_PDI & BIT11)//��������IO�ڵ�ƽΪ��

//#define DIR_DELAY            (500)//��λ:ms

#define FG_HIGH              (GPIO1_PDO |= BIT4)  //FG���IO�ڵ�ƽΪ��    P0.6
#define FG_LOW               (GPIO1_PDO &= (~BIT4)) //FG���IO�ڵ�ƽΪ��

/*********************PWM����趨******************************/
#define PWM_OFF               (100)  //�ر�ռ�ձ�   9%
#define PWM_MIN               (120)  //��Сռ�ձȣ�10%
#define PWM_MAX               (950)  //���ռ�ձȣ�90%

#define PWM_POWER_OFF         POWER_CALC(2.5)
#define PWM_POWER_MIN         POWER_CALC(3.0)
#define PWM_POWER_MAX         POWER_CALC(32.0)     //31

#define PWM_POWER_SCALE       (float)(PWM_POWER_MAX - PWM_POWER_MIN)/(PWM_MAX - PWM_MIN)  //PWM ����б��

#define PWM_SPEED_OFF         (s32)(10.0)      //��λ��Hz
#define PWM_SPEED_MIN         (s32)(20.0)    //��λ��Hz
#define PWM_SPEED_MAX         (s32)(100.0)   //��λ��Hz
//#define PWM_SPEED_TOTAL       (s32)(3300.0)   //��λ��Hz

#define PWM_SPEED_SCALE       (float)(PWM_SPEED_MAX - PWM_SPEED_MIN)/(PWM_MAX - PWM_MIN)  //PWM ����б��

/***********************VSP����趨****************************/
#define VSP_OFF               (1820.0) //��λ����ѹ�ػ�ֵ   ���㹫ʽ�� 0.2/3.6*32767 ��0.2Ϊ�ػ���ѹ
#define VSP_MIN               (2730.0) //��λ����ѹ��Сֵ   ���㹫ʽ�� 0.3/3.6*32767 ��0.3Ϊ��С��ѹ
#define VSP_MAX               (20024.0)//��λ����ѹ���ֵ   ���㹫ʽ�� 2.2/3.6*32767 ��2.2Ϊ����ѹ

#define VSP_SPEED_MIN         (30.0)    //��λ��Hz
#define VSP_SPEED_MAX         (350.0)   //��λ��Hz

#define VSP_SPEED_SCALE       (float)(VSP_SPEED_MAX - VSP_SPEED_MIN)/(VSP_MAX - VSP_MIN)  //PWM ����б��

typedef struct
{
    s32 wHighLevelTime;    //�ߵ�ƽ����ֵ
    s32 wLowLevelTime;     //�͵�ƽ����ֵ
    s32 wPeriod;           //���ڼ���ֵ
    u16 nDuty;             //ռ�ձ�
    u16 nFlagPeriod;       //���ڼӲ��־λ
    s32 wPreTime;          //�ϴμ���ֵ
    u16 nHighCntr;         /*IO��Ϊ��ʱ�ļ���ֵ*/
    u16 nLowCntr;         /*IO��Ϊ��ʱ�ļ���ֵ*/

    float fPwmSpeedScale; /* ת�ټ������ֵ*/
    s32 wPwmSpeedMax;     /* ռ�ձȸ������ֵ*/
} stru_PWMTime_t;


extern stru_PWMTime_t  struPWMTime;

extern void PWMScan(void);
extern void PWMScanInit(void);
extern void PWMDutyScan(void);
extern void VspSpeedScan(void);
extern void PWMSpeedScan(void);
extern void MotorDirScan(void);

void PWMOutputs(MCPWM_REG_TypeDef *MCPWMx, FuncState t_state);

extern void FGScan(u16 nTheta);

#endif

