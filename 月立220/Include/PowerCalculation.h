/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� PowerCalculation.h
 * �ļ���ʶ��
 * ����ժҪ�� ���ʴ���
 * ����˵���� ��
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2020��10��9��
 *
 *******************************************************************************/
#ifndef __POWER_H
#define __POWER_H

#include "basic.h"
#include "MC_Parameter.h"
#include "SpeedScan.h"
#include "mc_type.h"

#define POWER_LOWPASS_SCALE   (5)     //���ʼ����˲�ϵ��
#define POWER_SHIFT           (7)     //���ʼ�����λϵ��������ʵ�ʼ�����ֵ���޸Ĵ˲����������ֵС���ʵ���С�˲����������ʵ��Ӵ�

//***********************������ϲ���*******************//
#define POWER_SLOPE                     (27.502)  //����б��
#define POWER_DIFF                      (18.187) //���߲�ֵ
#define POWER_CALC(val)                 (POWER_SLOPE * val - POWER_DIFF)


typedef struct                        //��ͨ
{
    s32 wInput;                       //������
    s32 wOutput;                      //�����
    s16 nK1;                          //ϵ��
    s64 lTemp;                        //�м�ֵ
} stru_LowPass_t;

typedef struct
{
    s32 wRef;                        //����ֵ
    s32 wSet;                        //�趨ֵ
    s16 nACCStep;                    //����ֵ
    s16 nDECStep;                    //����ֵ
} stru_Ramp_t;


typedef struct                      
{
    s32 wPowerValue;                  //����ʵ��ֵ
	s32 wPowerValueLowPass;                  //�����˲�ֵ
    s32 wTemp;                        //�м�ֵ
    
    stru_LowPass_t struPowerLowPass;  //��ͨ�˲�
    
    stru_Ramp_t  struPowerRamp;       //����б�½ṹ��
    
    stru_PIRegulator struPowerPI;     //���ʻ�PI�ṹ��
    
    s16 nSpeedLimitCntr;              //��ת�����ڼ���
    s32 wSpeedLimitValue;             //ת������ֵ
    s32 wSpeedLimitPowerSet;          //��ת�ٹ����趨ֵ
    s32 wSpeedLimitPowerRef;          //��ת�ٹ��ʸ���ֵ
    
    s32 wPowerRef;                    //���ʸ���ֵ
    s32 wPowerSet;                    //�����趨ֵ
	
	s16 BUSCURR;
	s32 BUSpower;
	
} stru_PowerBasic_t;

extern stru_PowerBasic_t struPower;

extern u16 DIV(u32 Dividend, s16 Divisor);
extern s32 LowPassControl(stru_LowPass_t *pstruLowPass);
extern s32 RampControl(stru_Ramp_t *pstruRamp);
extern void PowerCalc(stru_CurrVoctorDQ *pstruCurrDQ, stru_VoltVoctorDQ *pstruVoltDQ, stru_PowerBasic_t *pstruPower);

extern void PowerLimitCalc(MechanicalQuantity *pstruMotorSpeed, s32 PowerValue);
extern void SpeedLimitCalc(s32 wSpeedFbk, stru_PowerBasic_t *pstruPower);

extern void PowerLoopInit(void);

extern void PowerLoopReg(stru_PowerBasic_t *this);
#endif
