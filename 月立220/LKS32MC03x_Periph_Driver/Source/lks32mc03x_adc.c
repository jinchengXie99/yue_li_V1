/*******************************************************************************
 * ��Ȩ���� (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� lks32mc03x_adc.c
 * �ļ���ʶ��
 * ����ժҪ�� ADC������������
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ�   HMG
 * ������ڣ� 2021��10��14��
 *
 *******************************************************************************/
#include "lks32mc03x_adc.h"
#include "lks32mc03x.h"
#include "lks32mc03x_sys.h"

/*******************************************************************************
 �������ƣ�    void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
 ����������    ADC��ʼ��
 �����ı�    ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
2021/10/14    V1.0             HMG               ����
 *******************************************************************************/
void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct)
{

    uint16_t t_reg;

    SYS_AnalogModuleClockCmd(SYS_AnalogModule_ADC, ENABLE); // ADCģ��ʹ��

    ADCx->IE = ADC_InitStruct->IE;

    t_reg = (ADC_InitStruct->Align << 10) | (ADC_InitStruct->Trigger_En) | (ADC_InitStruct->SEL_En << 12) |
            (ADC_InitStruct->Trigger_Cnt << 4) | (ADC_InitStruct->Trigger_Mode << 8);
    ADCx->CFG = t_reg;

    ADCx->CHNT = ADC_InitStruct->FirSeg_Ch | (ADC_InitStruct->SecSeg_Ch << 4) |
                  (ADC_InitStruct->ThrSeg_Ch << 8) | (ADC_InitStruct->FouSeg_Ch << 12);
}

/*******************************************************************************
 �������ƣ�    void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
 ����������    ADC�ṹ���ʼ��
 �����ı�    ��
 ���������    ��
 ���������    ��
 �� �� ֵ��     ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
2021/10/14    V1.0             HMG               ����
 *******************************************************************************/
void ADC_StructInit(ADC_InitTypeDef *ADC_InitStruct)
{
    ADC_InitStruct->IE = 0;
    ADC_InitStruct->Align = 0;
    ADC_InitStruct->Con_Sample = 0;
    ADC_InitStruct->Trigger_Cnt = 0;
    ADC_InitStruct->FirSeg_Ch = 0;
    ADC_InitStruct->SecSeg_Ch = 0;
    ADC_InitStruct->ThrSeg_Ch = 0;
    ADC_InitStruct->FouSeg_Ch = 0;
    ADC_InitStruct->Trigger_Mode = 0;
    ADC_InitStruct->Trigger_En = 0;
    ADC_InitStruct->SEL_En = 0;
    ADC_InitStruct->ADC_GEN_En = 0;
    ADC_InitStruct->ADC_GEN_HTH = 0;
    ADC_InitStruct->ADC_GEN_LTH = 0;
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
