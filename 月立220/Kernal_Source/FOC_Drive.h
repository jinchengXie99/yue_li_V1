/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� FOC_Drive.h
 * �ļ���ʶ��
 * ����ժҪ�� ���ʸ�����Ƶ����ڻ�
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
#ifndef __MC_FOC_DRIVE_H
#define __MC_FOC_DRIVE_H

#include "mc_type.h"
#include "PID_regulators.h"
#include "function_config.h"
#include "pmsmFluxObserve.h"

typedef struct
{
    void (*ADC_A_B_Set)(void); /* ADC���������ʱ�� */
    void (*ADC_A_C_Set)(void); /* ADC���������ʱ�� */
    void (*ADC_B_C_Set)(void); /* ADC���������ʱ�� */
} Stru_ADC_SampleDef;


typedef const struct
{
    u16 MIN_PWMSampTime3Shunt;    /* ���ŵ������ʱ,���������ռ�ձ���,����ADC�ɿ���������С��PWMֵ */
    u16 SampStableTime3Shunt;     /* ���ŵ������ʱ,��Ҫ��֤ADC����ȷ�������ڣ��ж�����PWMռ��֮��Ĳ�ֵ�Ƿ������������ */
                                     
    s16 ShiftSampTime3Shunt;      /* �����������λʱ�� */
    s16 SampWaitTime3Shunt;       /* ����������ȴ�ʱ�� */
  
    s16 SampStableTime1Shunt;     /* ������������ź�������ʱ�� */
    u16 ShiftSampTime1Shunt;      /* �������������ʱ�� */


    u16 nPWM_PERIOD;              /* PWM����ֵ */
    u16 nDEADTIMECOMP;            /* ��������ֵ */

} Stru_FOC_ConstParame, *pStru_FOC_ConstParame;

typedef struct
{
    stru_CurrPhaseUVW mStatCurrUVW;              /* �������A��B�����  */
    stru_CurrPhaseUVW mLastStatCurrUVW;          /* ��һ�ε������A��B�����  */
    stru_CurrVoctorDQ mStatCurrDQ;               /* �������Q��D�����  */
    stru_CurrAlphBeta mStatCurrAlfaBeta;         /* �������alfa beta�����  */

    stru_VoltVoctorDQ mStatVoltDQ;               /* �������Q��D���ѹ  */
    stru_VoltAlphBeta mStatVoltAlfaBeta;         /* �������alfa beta���ѹ�� */

    stru_TrigComponents mTrigSinCos;             /* ���ӽǶ�sin Cosֵ */
    u16 wElectrical_Angle;                       /* ��ǰת��λ�ýǶ�ֵ��0~65535��Ӧ0~360�� */

    s16 nBusVoltage;                             /* ֱ��ĸ�ߵ�ѹ */             
                                                                          
    stru_VoltPhaseUVW mVdtComp;                  /* ����ʱ�䲹����ѹ */
    stru_VoltPhaseUVW mVoltUVW_PWM;              /* UVW PWM�����ѹ */
    stru_VoltPhaseUVW mVoltUVW_NegPWM;           /* UVW PWM���������ѹ */
  
    u16 nVoltageCircleLim;                       /* ��ѹ����Բ���� ��λ��Q12 ���ֵ4096 */  

    u8 bSampCur0;                                /* ��ǰ��������ͨ����־ */
    u8 bSampCur1;                                /* ��ǰ��������ͨ����־ */
    u8 bSampCur2;                                /* ��ǰ��������ͨ����־ */

    u16 nADC_TrigPoint0;                         /* ADC ����������0 */
    u16 nADC_TrigPoint1;                         /* ADC ����������1 */
    
    u16 ADC_A_B_CurrentSamp;                     /* ADC�����ʱ�� */
    u16 ADC_A_C_CurrentSamp;                     /* ADC�����ʱ�� */
    u16 ADC_B_C_CurrentSamp;                     /* ADC�����ʱ�� */

    s16 nPhaseAOffset;                           /* A��ADC���� Offsetֵ */
    s16 nPhaseBOffset;                           /* B��ADC���� Offsetֵ */
    s16 nPhaseCOffset;                           /* C��ADC���� Offsetֵ */
    s16 nBusShuntOffset;                         /* ֱ��ĸ�ߵ���ADC���� Offsetֵ */
    
    s16 nQCur_Reference;                         /* Q��������ο����� */
    s16 nDCur_Reference;                         /* D��������ο����� */
    
    s16 nQCurrentSet;                            /* Q������趨ֵ */
    s16 nDCurrentSet;                            /* D������趨ֵ */    

    s16 nRequestPower;                           /* Q���ѹ���� */
    
    s16 nWeakFieldLim;                           /* ���ŵ������� */

    s8 bCntFirCurA;                              /* A������˲������� */
    s8 bCntFirCurB;                              /* B������˲������� */
    stru_RC_Def mCurrA_RC;                       /* A�������RC�˲��ṹ��  */
    stru_RC_Def mCurrB_RC;                       /* B�������RC�˲��ṹ��  */

    u8 bSVPWM_Type;                              /* PWM �������ͣ�1��7��SVPWM,0���PWM */

    u16 nCurrentSqrtOrg;                         /* �����������ֵ  */

    stru_PIRegulator mPI_Torque;                 /* Q�������PI���� */
    stru_PIRegulator mPI_Flux;                   /* D�������PI���� */

    s16 nMAX_CurrentLimThd;                      /* ����������ֵ */

    pStru_FOC_ConstParame pStruFOC_ConstPar;     /* FOC����Const���� */
    
    void (*MCPWMx_RegUpdate)(void);              /* ����PWM�Ĵ������� */  
		
    stru_PIRegulator mPI_Torque_BRAKE;                 /* ɲ��Q�������PI���� */
    stru_PIRegulator mPI_Flux_BRAKE;                   /* ɲ��D�������PI���� */

} stru_FOC_CurrLoopDef;


  

typedef struct
{

 
    volatile u16 nSys_TimerPWM;                  /* PWM���ڼ���Cnt */

    u8 bMotorDirtionCtrl;                        /* �������Ƶĵ�����з��� */

    u8 bMC_RunFlg;                               /* �������ָ�� */
    enum_SystStatus eSysState;                   /* ��ǰ�������״̬�� */
    u16 nSetTimeLeftCnt;                         /* ״̬���ȴ�ʱ������� */

    stru_RC_Def struBusCurr_RC;                  /* ĸ�ߵ���RC�˲��ṹ��  */
  

} stru_FOC_CtrProcDef;


typedef struct
{
   s32 wMotorSpeedEst;          /* �۲�������ٶ� */
   stru_RC_Def mRC_Obs;         /* �۲�����ͨ�˲� */
  
   u16 nObsFactor;              /* �۲�������ϵ�� */
   u8 bDivisor;                 /* �۲���������ȣ�����ϵ�� */
   s32 wIS2;                    /* ��������ģֵ */
   stru_PIRegulator mPI_Obs;    /* D�������PI���� */
  
   s8 bMotorDir;                /* ���ת���з��� */
   u16 nObsCoef;                /* �۲����˲�ϵ�� */
   u16 nMaxOutFreq;             /* ���۲��ٶ� */
   u16 nMinOutFreq;             /* ��С�۲��ٶ� */
}stru_FluxOBS_Def;


#define MAX_DELTA_CUR           20000            /* ������˲�������ADC�����������仯�ʲ�Ӧ����1000��ADC Lsb */



#define SAMP_NO               0
#define SAMP_IA               1
#define SAMP_IB               2
#define SAMP_IC               3
#define SAMP_NIA              4
#define SAMP_NIB              5
#define SAMP_NIC              6
#define SAMP_OLDA             7
#define SAMP_OLDB             8
#define SAMP_OLDC             9

#if (ROTOR_SENSOR_TYPE == ROTOR_HALL_SENSOR)  
#define GET_ELECT_ANGLE()     {tElectAngle = struHallProcess.nElectrical_Angle; struFluxOB_Param.wElectAngle = tElectAngle<<16;}
#endif

#if (ROTOR_SENSOR_TYPE == ROTOR_SENSORLESS) 
#define GET_ELECT_ANGLE()     {tElectAngle = struFluxOB_Param.wElectAngle>>16;}
#endif

/* Exported functions ------------------------------------------------------- */
void FOC_Model(stru_FOC_CtrProcDef *pStruFOC_CtrProc);
void MCL_Init(stru_FOC_CtrProcDef *this);
stru_TrigComponents Trig_Functions(s16 hAngle);
void RevPark_Circle_Limitation(stru_VoltVoctorDQ *mStatVolt_q_d, u16 MAX_ModuleCof);

stru_CurrPhaseUVW SVPWM_1ShuntGetPhaseCurrent(stru_FOC_CurrLoopDef *this);
stru_CurrPhaseUVW SVPWM_2ShuntGetPhaseCurrent(stru_FOC_CurrLoopDef *this);
stru_CurrPhaseUVW SVPWM_3ShuntGetPhaseCurrent(stru_FOC_CurrLoopDef *this);

void SVPWM_1ShuntGEN(stru_FOC_CurrLoopDef *this);
void SVPWM_2ShuntGEN(stru_FOC_CurrLoopDef *this);
void SVPWM_3ShuntGEN(stru_FOC_CurrLoopDef *this);

s16 lowPass_filter(stru_RC_Def *rc,s16 signal);

void ADC_SampPhaseA_B_Set(void);
void ADC_SampPhaseA_C_Set(void);
void ADC_SampPhaseB_C_Set(void);


void MCPWM0_RegUpdate(void);
void MCPWM1_RegUpdate(void);

void FOC_InitstruParam(void);

s16 DcVoltPerUnitCalc(s16 sav);
s16 DcVoltPerUnitCalcInt(s16 sav);

void CurrentLoopReg(stru_FOC_CurrLoopDef *this);

void SpeedLoopReg(MechanicalQuantity *this);

void BrakeStateCheck(void);

s32 App2CoreCurTrans(s32 val);
s32 App2CoreVolTrans(s32 val);
s32 App2CoreFreqTrans(s32 val);
s32 App2CoreVdcTrans(s32 val);
u32 App2CoreAngleTrans(u16 val);
s32 Core2AppCurTrans(s32 val);
s32 Core2AppVolTrans(s32 val);
s32 Core2AppFreqTrans(s32 val);
s32 Core2AppVdcTrans(s32 val);
u16 Core2AppAngleTrans(u32 val);
s32 User2AppCurTrans(float val);
s32 User2AppVolTrans(float val);
s32 User2AppFreqTrans(float val);
u16 User2AppAngleTrans(float val);


#endif /* __MC_FOC_DRIVE_H */
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

