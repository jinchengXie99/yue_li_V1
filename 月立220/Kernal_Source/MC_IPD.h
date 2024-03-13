/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� MC_IPD.h
 * �ļ���ʶ��
 * ����ժҪ�� �����ʼλ�ö�λ��������
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� Howlet Li
 * ������ڣ� 2020��8��16��
 *
 * �޸ļ�¼1��
 * �޸����ڣ�2020��8��16��
 * �� �� �ţ�V 1.0
 * �� �� �ˣ�Howlet Li
 * �޸����ݣ�����
 *
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_IPD_H
#define __MC_IPD_H

#include "mc_type.h"


typedef struct
{
    u32 wIPD_PlusWidthSet;                        /* IPDλ�ü�⣬ע�����������ã���λ��clock������ */
    u32 wIPD_IdleWaitSet;                         /* IPDλ�ü�⣬���������ȴ�������ã���λ��clock������ */
    u16 nPWM_PERIOD;                              /* PWM ռ�ձ� */

    u8  bIPD_State;                               /* IPD״̬��־ */
    u8  bIPD_StepFlg;                             /* IPD�����־ */

    u16 IPD_Angle;                                /* IPD�õ��ĽǶ�ֵ */

    s16 AdcSampCurr[7];                           /* ADC ����ֵ */
  
    volatile u16 hDriverPolarity;                 /* ȡMCPWM_IO1��MCPWIO23��������CHxN��CHxP�ļ��Ե����� */
  
    u8 bCurrentSampleType;                        /* �����������ͣ������裬˫���裬������ */

} stru_IPD_CtrProcDef;


void IPD_RotorPosEst(void);

#endif

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

