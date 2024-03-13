/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� PID_regulators.c
 * �ļ���ʶ��
 * ����ժҪ�� PID������
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

/* Standard include ----------------------------------------------------------*/
#include "basic.h"
#include "MC_type.h"
#include "PID_regulators.h"


 /*******************************************************************************
 �������ƣ�    s16 PI_Regulator(s16 hReference, s16 hPresentFeedback, stru_PIparams *PID_Struct)
 ����������    PI������ ����ֵʽ
 ���������    s16 DesiredValue: ����ֵ 
               s16 MeasuredValue:����ֵ 
               stru_PIparams *PID_Struct �ṹ��ָ��
 ���������    PI���ڽ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
s16 PI_Regulator(s16 DesiredValue, s16 MeasuredValue, stru_PIparams *PID_Struct)
{
    s32 wError, wProportional_Term, wIntegral_Term;
    volatile s32 houtput_32;
    s64 dwAux;

    // error computation
    wError = (s32)(DesiredValue - MeasuredValue);

    // Proportional term computation
    wProportional_Term = PID_Struct->hKp_Gain * wError;

    // Integral term computation
    if (PID_Struct->hKi_Gain == 0)
    {
        PID_Struct->wIntegral = 0;
    }
    else
    {
        wIntegral_Term = PID_Struct->hKi_Gain * wError;
        dwAux = PID_Struct->wIntegral + (s64)(wIntegral_Term);

        if (dwAux > PID_Struct->wUpper_Limit_Integral)
        {
            PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
        }
        else if (dwAux < PID_Struct->wLower_Limit_Integral)
        {
            PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
        }
        else
        {
            PID_Struct->wIntegral = (s32)(dwAux);
        }
    }

    houtput_32 = ((wProportional_Term >> PID_Struct->hKp_Divisor)
                  + (PID_Struct->wIntegral >> PID_Struct->hKi_Divisor));
    if (houtput_32 >= PID_Struct->hUpper_Limit_Output)
    {
        return (PID_Struct->hUpper_Limit_Output);
    }
    else if (houtput_32 < PID_Struct->hLower_Limit_Output)
    {
        return (PID_Struct->hLower_Limit_Output);
    }
    else
    {
        return ((s16)(houtput_32));
    }
}


 /*******************************************************************************
 �������ƣ�    s16 CurrentPIRegulator(stru_PIRegulator *Regulator)
 ����������    ������PI  ����ʽPI
 ���������    stru_PIRegulator *Regulator �ṹ��ָ��
 ���������    PI���ڽ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           WenCai Zhao          ����
 *******************************************************************************/
s16 CurrentPIRegulator(stru_PIRegulator *Reg)
{
    long    ACC;
    int AX;

    ACC = (long)(Reg->wInError - Reg->wLastError) * Reg->KP;   /* ��������� */
    ACC = (ACC << 4) + (long)(Reg->wInError) * Reg->KI;        /* ��������� */
    Reg->wIntegral = ACC + Reg->wIntegral;

    if(Reg->wIntegral > Reg->wUpperLimitOutput)                /* ������ֵ�޷� */
    {
        Reg->wIntegral = Reg->wUpperLimitOutput;
    }
    else if(Reg->wIntegral < Reg->wLowerLimitOutput)         /* �����Сֵ�޷� */
    {
        Reg->wIntegral = Reg->wLowerLimitOutput;
    }

    AX = Reg->wIntegral >> 16;

    Reg->wLastError = Reg->wInError;                           /* ��¼�ϴ����ֵ */

    return(AX);                                                          
}

 /*******************************************************************************
 �������ƣ�   s16 SpeedPIRegulator(stru_SpeedPIRegulator *Reg)
 ����������    �ٶȻ�PI  ����ʽPI
 ���������    stru_SpeedPIRegulator *Reg �ṹ��ָ��
 ���������    PI���ڽ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/10/7      V1.0           WenCai Zhao          ����
 *******************************************************************************/
s16 SpeedPIRegulator(stru_SpeedPIRegulator *pReg)
{
    s32 t_AX;
    s64 t_OUT_LIM_H, t_OUT_LIM_L;
    
    t_OUT_LIM_H = (s64)(pReg->wUpperLimitOutput) << 22;      /*����޷�ֵ*/
    t_OUT_LIM_L = (s64)(pReg->wLowerLimitOutput) << 22; 
   
    pReg->lACC = (s64)pReg->wInError * pReg->KI;             /*���������*/
  
    pReg->wIntegral = pReg->wIntegral + pReg->lACC;
    
    if(pReg->wIntegral > t_OUT_LIM_H)
    {
       pReg->wIntegral = t_OUT_LIM_H;
    }
    else if(pReg->wIntegral < t_OUT_LIM_L)
    {
        pReg->wIntegral = t_OUT_LIM_L;
    }
    
    pReg->lACC = (s64)(pReg->wInError) * pReg->KP * 128;     /*���������*/

    pReg->lACC = pReg->lACC + pReg->wIntegral;
    
    if(pReg->lACC > t_OUT_LIM_H)
    {
        pReg->lACC = t_OUT_LIM_H;
    }
    else if(pReg->lACC < t_OUT_LIM_L)
    {
        pReg->lACC = t_OUT_LIM_L;
    }
    
    t_AX = (pReg->lACC >> 22);
    
    return (t_AX);
}

/******************** (C) COPYRIGHT 2008 LINKO Semiconductor *******************/

