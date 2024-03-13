/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� PID_regulators.h
 * �ļ���ʶ��
 * ����ժҪ�� PID����ͷ�ļ�
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� Howlet
 * ������ڣ� 2020��8��5��
 *
 * �޸ļ�¼1��
 * �޸����ڣ� 2020��8��5��
 * �� �� �ţ� V 1.0
 * �� �� �ˣ� Howlet
 * �޸����ݣ� ����
 *
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
 
#ifndef __PI_REGULATORS__H
#define __PI_REGULATORS__H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    s16 hKp_Gain;
    u16 hKp_Divisor;
    s16 hKi_Gain;
    u16 hKi_Divisor;
    s32 hLower_Limit_Output;     //Lower Limit for Output limitation
    s32 hUpper_Limit_Output;     //Lower Limit for Output limitation
    s32 wLower_Limit_Integral;   //Lower Limit for Integral term limitation
    s32 wUpper_Limit_Integral;   //Lower Limit for Integral term limitation
    s32 wIntegral;
} stru_PIparams;

typedef struct 
{
    s32 wUpperLimitOutput;       /* Lower Limit for Output limitation */
    s32 wLowerLimitOutput;       /* Lower Limit for Output limitation */
    u16 KP;                      /* ����ϵ�� */
    u16 KI;                      /* ����ϵ�� */
    s32 wIntegral;               /* ������ */
    s32 wInError;                /* ������� */
    s32 wLastError;              /* �ϴ�������� */
    s32 wErrorC;
}stru_PIRegulator;

typedef struct
{
    s32 wUpperLimitOutput;       /*�ٶȻ�������ֵ*/
    s32 wLowerLimitOutput;       /*�ٶȻ������Сֵ*/
    u16 KP;                      /* ����ϵ�� */
    u16 KI;                      /* ����ϵ�� */
    s32 wInError;                /* ������� */
    s64 wIntegral;               /* ������ */
    s64 lACC;                    /* �м�������*/
}stru_SpeedPIRegulator;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
s16 PI_Regulator(s16, s16, stru_PIparams *);

s16 PD_Regulator(s16 hPresentFeedback, stru_PIparams *PD_Struct);

s16 HL_PI_AntiDump(s16 DesiredValue,s16 MeasuredValue,stru_PIparams *pParams);

s16 CurrentPIRegulator(stru_PIRegulator *Reg);

s16 SpeedPIRegulator(stru_SpeedPIRegulator *pReg);

/* Exported variables ------------------------------------------------------- */

#endif 

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

