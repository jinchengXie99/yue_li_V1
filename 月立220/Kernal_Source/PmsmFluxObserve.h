/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� PmsmFluxObserve.h
 * �ļ���ʶ��
 * ����ժҪ�� ��ͨ�۲�����ļ�
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� WenCai Zhao
 * ������ڣ� 2020��8��20��
 *
 * �޸ļ�¼1��
 * �޸����ڣ� 2020��8��20��
 * �� �� �ţ� V 2.0
 * �� �� �ˣ� Howlet
 * �޸����ݣ� ����
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __PMSM_FLUX_OBSERVE_H_
#define __PMSM_FLUX_OBSERVE_H_

#include "mc_type.h"
#include "PID_regulators.h"

typedef struct
{ /* ˳�����������ת���ٶ�λ�÷����� */
    u16 nElectAngle;          /* ��Ƕ� */
    s16 nFreq;                /* ת�ӵ�Ƶ�� */
    s16 nFreqAvg;             /* ת�ӵ�Ƶ��ƽ��ֵ */
    s32 wBemfFreq;            /* �����Ƽ���˲�ֵ */
    s32 wVolMagAvg;           /* ��ǰ�����Ƶ�ѹƽ��ֵ */
    s16 nMotorVolMag;         /* ��ǰ�����ѹ���� */
    s32 wBEMF_DppValueAcc;    /* �ٶȻ����� */
    volatile s32 wAngleDpp;   /* ���ٶ�ֵ */
    s16 nPWM1msTick;          /* ��PWMΪʱ��1mS����ֵ */
    u8  bMotorStopDecFlag;    /* ���ͣת��־ */
    s16 nMotorStopCurrentThd; /* ���ͣת���������ֵ */
    s16 nMotorStopCurDifThd;  /* ���ͣת������ֵ�����ֵ */
} Stru_OnTheFlyDetect;


typedef struct {
    s32 wSpeedRef;                     /* �ٶȸ���ֵ */
    s32 wSpeedfbk;                     /* �ٶȷ���ֵ */
    s32 we;                            /* we,omegaE; */
    s32 wSpeedSet;                     /* �ٶ��趨 */
    s32 wSpeedEst;                     /* �۲������ٶ���� */
    u16 nSpeedLoopKP;                  /* �ٶȹ���Kp ֵ */
    u16 nSpeedLoopKI;                  /* �ٶȹ���Ki ֵ */ 

    stru_SpeedPIRegulator mSpeedPI; /* �ٶȻ� PI */

    s32 wSpeedRampACCStep;             /*�ٶ����¼���ֵ*/
    s32 wSpeedRampDECStep;             /*�ٶ����¼���ֵ*/
    
    s32 wPowerLimitSpeedRef;           /*�޹���ת�ٸ���ֵ*/
    s32 wPowerLimitSpeedSet;           /*�޹���ת���趨ֵ*/
    s32 wPowerLimitValue;              /*�޹��ʴ�С*/
    s16 nPowerLimitCntr;               /*�޹��ʼ������*/
	
    s16 nBrakeCurrent;                 /* Q��ɲ������ */
	  s32 wBrakeSpeed;                   /* ɲ��ת�� */
	
} MechanicalQuantity;

typedef const struct
{   /* ��������ṹ�� */
    s8  bMotorType;           /* 0:im,1:spm,2:ipm,3...*/
    s16 nRatedPower;          /* ����� 0.1W */
    s32 wRatedFreq;           /* �Ƶ�� ��λ��0.01Hz*/
    s8  bPolePairs;           /* ����ż����� */
    s16 nRatedCur;            /* ����� ��λ��0.1A */
    s32 wMotorRs;             /* ������� ��λ��0.0001ŷķ */
    s32 wMotorLd;             /* ���D���� ��λ��uH */
    s32 wMotorLq;             /* ���Q���� ��λ��uH */
    u32 nFluxConst;           /* ����������� 0.00001WB */
    s32 wMotorRr;             /* ���ת������ ��λ��0.0001ŷķ */
    s32 wMotorLr;             /* ���ת�ӵ�� ��λ��uH */
    s32 wMotorlm;             /* ������� ��λ��uH */
    s16 nNoLoadCur;           /* ������ص��� ��λ��0.1A */
    u16 nRatedTorque;         /* ����ת�� ��λ��0.001Nm */
} stru_MotorParameterDef, *pStru_MotorParameterDef;

typedef const struct
{   /* ���ʰ�Ӳ����·���Բ��� */
    s16 nRatedUdc;            /* �ֱ��ĸ�ߵ�ѹ */
    s16 nMaxUdc;              /* ���ֱ��ĸ�ߵ�ѹ */
    s16 nMaxCur;              /* �������� ��λ��0.1A */
    s16 nDeadTime;            /* ����ʱ�� ��λ��ns */ 
    s32 nMaxFreq;             /* �����Ƶ�� ��λ��Hz */
    u16 nPWMFreq;             /* PWM��Ƶ */
  
    float fAdcSupplyVol;      /* ADC�ο���ѹ ��λ������ */
    float fRShunt;            /* ��ѹ���� ��λ��ŷķ*/
    float fAmpGain;           /* �Ŵ����Ŵ��� ��λ��n�� */ 
    float fVolShuntRatio;     /* ĸ�ߵ�ѹ������ѹ�� */ 
    float fBemfShuntRatio;    /* �����Ƶ�ѹ������ѹ�� */  

} stru_BoardParameterDef, *pStru_BoardParameterDef;

typedef struct
{ /* �۲��������������� */
  
   u16 nCurrLoopIniKP;        /* ������KP��ʼֵ, �������ݵ����е��������˲��� */
   u16 nCurrLoopIniKI;        /* ������KI��ʼֵ, �������ݵ����е��������˲��� */ 
	
	 u16 nCurrLoopIniKPL;        /* ������KP��ʼֵ, �������ݵ����е��������˲��� */
   u16 nCurrLoopIniKIL;        /* ������KI��ʼֵ, �������ݵ����е��������˲��� */ 

   u16 nD_CurrLoopKP;         /* D�������KP����ֵ */
   u16 nD_CurrLoopKI;         /* D�������KI����ֵ */  
   u16 nQ_CurrLoopKP;         /* Q�������KP����ֵ */
   u16 nQ_CurrLoopKI;         /* Q�������KI����ֵ */   
   
   u16 nSTO_KP;               /* �۲���PLL KPֵ */
   u16 nSTO_KI;               /* �۲���PLL KPֵ */
	
	
	 u16 nSTO_KP_H;               /* �۲���PLL KPֵ */
   u16 nSTO_KI_H;               /* �۲���PLL KPֵ */
   
   u32 wElectAngleEst;        /* ��������Ƕ� */ 
   u32 wElectAngleOpen;       /* �����Ƕ� */
   u32 wElectAngle;           /* ��ǰ���нǶ� */   

   s32 CurrentPerUnitGAIN;    /* �������ۻ����������� */
   s32 VDC_PerUnitGAIN;       /* ��ѹ�������ۻ��������� */
   s32 FREQ_PerUnit_GAIN;     /* Ƶ�ʱ��ۻ��������� */
   s32 BEMF_PerUnit_GAIN;     /* �����Ʊ��ۻ��������� */

   /* ��������ṹ�� */
   pStru_MotorParameterDef pStruMotorParame;
   /* ���ʰ�Ӳ����·���Բ��� */  
   pStru_BoardParameterDef pStruBoardParame; 

   u16 nObserveMinSpeed;      /* �۲�����С���ֵ */
   u16 nSpeedFiltFac;         /* �ٶȹ����˲�ϵ�� */
  
}stru_FluxOB_ParameterDef;

typedef struct
{ /* �۲����������� */
   s32 wOb_F1;              /* �۸���ϵ��F1 */
   s32 wOb_F2;              /* �۸���ϵ��F2 */
   s32 wOb_F3;              /* �۸���ϵ��F3 */
  
   s8 bSftN_F1;             /* �۸���ϵ��F1 ��λ�� */
   s8 bSftN_F2;             /* �۸���ϵ��F2 ��λ�� */
   s8 bSftN_F3;             /* �۸���ϵ��F3 ��λ�� */

   s32 wOBPllInteg;         /* �۲����������ֵ */  

   u16 nBemfLowPassFac;     /* �������˲�ϵ��  */  
  
   u16 nSpeedFiltFac;       /* �ٶȹ����˲�ϵ�� */
  
   u16 nOpenLoopDppKp;      /* ��������ģʽ���ٶȲο����� */
  
   u16 nProductPLL;         /* PLL �������ϵ��  */
  
   u16 uBusVolt_PerUnit;    /* ��һ���ֱ��ĸ�ߵ�ѹ */
  
}stru_FluxOB_VariableDef;

struct PIRegFieldW {
    INT32 LIM_H;
    INT32 LIM_L;
    UINT16    KP;
    UINT16    KI;
    INT32    INTEG;
    INT32    inError;//ERROR
};


extern Stru_OnTheFlyDetect mOnTheFlyDetect;

void FluxObserveParaCalc(void);
void ModuCircle_Limitation(void);
void PmsmFluxObserve(void);
void PmsmFluxObserve1(void);
void OnTheFlyDetectPro(void);
void OnTheFlyDetectPro1(void);
void OpenCloseAngleSwitch(void);
void PmsmFluxObIni(void);
void OnTheFlyDetectInit(void);


/* ---------------------------�۲��������������� ----------------------------- */ 
#define NOT_APPLICABLE                 (0)               /* δ���� */

#define MOTOR_PARAMETER_TAB                                       \
{                                                                 \
    NOT_APPLICABLE,           /* ������� */                      \
    NOT_APPLICABLE,           /* ����� */                      \
    (s32)(U_MAX_FREQ * 100),  /* �Ƶ�� ��λ��0.01Hz*/          \
    U_MOTOR_PP,               /* ����ż����� */                  \
    (s32)(U_RATED_CURRENT*10),/* ����� ��λ��0.1A */           \
    (s32)(U_MOTOR_RS*10000),  /* ������� ��λ��0.0001ŷķ */     \
    (s32)U_MOTOR_LD,          /* ���D���� ��λ��uH */          \
    (s32)U_MOTOR_LQ,          /* ���Q���� ��λ��uH */          \
    U_MOTOR_FLUX_CONST*100000,/* ����������� ��λ0.00001WB */    \
    NOT_APPLICABLE,           /* ���ת������ ��λ��0.0001ŷķ */ \
    NOT_APPLICABLE,           /* ���ת�ӵ�� ��λ��uH */         \
    NOT_APPLICABLE,           /* ������� ��λ��uH */             \
    NOT_APPLICABLE,           /* ������ص��� ��λ��0.1A */       \
    NOT_APPLICABLE,           /* ����ת�� ��λ��0.001Nm */    \
}

#define BOARD_PARAMETER_TAB                                       \
{  /* ���ʰ�Ӳ����·���Բ����� */                                 \
    (s16)(U_RATED_VOLTAGE*10),/* �ֱ��ĸ�ߵ�ѹ */              \
    NOT_APPLICABLE,           /* ���ֱ��ĸ�ߵ�ѹ */              \
    NOT_APPLICABLE,           /* �������� ��λ��0.1A */         \
    DEADTIME_NS,              /* ����ʱ�� ��λ��ns */             \
    (s32)(U_MAX_FREQ * 100),  /* �����Ƶ�� ��λ��0.01Hz */     \
    PWM_FREQ,                 /* PWM��Ƶ */                       \
                                                                  \
    ADC_SUPPLY_VOLTAGE,       /* ADC�ο���ѹ ��λ������ */        \
    RSHUNT,                   /* ��ѹ���� ��λ��ŷķ*/            \
    AMPLIFICATION_GAIN,       /* �Ŵ����Ŵ��� ��λ��n�� */      \
    VOLTAGE_SHUNT_RATIO,      /* ĸ�ߵ�ѹ������ѹ�� */            \
    BEMF_SHUNT_RATIO,         /* �����Ƶ�ѹ������ѹ�� */          \
}


#endif

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

