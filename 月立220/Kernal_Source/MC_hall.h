/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO Semiconductor
 *
 * �ļ����ƣ� mc_hall.c
 * �ļ���ʶ��
 * ����ժҪ�� Hall�źŴ���
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� Howlet Li
 * ������ڣ� 2016��2��16��
 *
 * �޸ļ�¼1��
 *    �޸����ڣ�2016��2��26��
 *    �� �� �ţ�V 1.0
 *    �� �� �ˣ�Howlet Li
 *    �޸����ݣ�����
 *
 *
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HALL_H
#define __HALL_H

/* Includes ------------------------------------------------------------------*/
#include "hardware_config.h"
#include "mc_type.h"


#define S16_360_PHASE_SHIFT    (u16)(65535)   /* 360�Ƚǹ�һ������ 1/360*65536, �Ƕ�0~360�ȱ仯����Ӧ��һ������0~65535 */
#define S16_330_PHASE_SHIFT    (u16)(60075)
#define S16_320_PHASE_SHIFT    (u16)(58253)
#define S16_318_PHASE_SHIFT    (u16)(57889) //58000--57889
#define S16_315_PHASE_SHIFT    (u16)(57344)
#define S16_310_PHASE_SHIFT    (u16)(56433)
#define S16_300_PHASE_SHIFT    (u16)(54613)   /* 300�Ƚǹ�һ������ 1/360*65536, �Ƕ�0~360�ȱ仯����Ӧ��һ������0~65535 */
#define S16_270_PHASE_SHIFT    (u16)(49152)
#define S16_240_PHASE_SHIFT    (u16)(43691)
#define S16_180_PHASE_SHIFT    (u16)(32768)
#define S16_120_PHASE_SHIFT    (u16)(21845)
#define S16_90_PHASE_SHIFT     (u16)(16384)
#define S16_75_PHASE_SHIFT     (u16)(13653)
#define S16_70_PHASE_SHIFT     (u16)(12743)
#define S16_63_PHASE_SHIFT     (u16)(11468)
#define S16_60_PHASE_SHIFT     (u16)(10923)
#define S16_45_PHASE_SHIFT     (u16)(8192)
#define S16_30_PHASE_SHIFT     (u16)(5461)
#define S16_20_PHASE_SHIFT     (u16)(3641)
#define S16_15_PHASE_SHIFT     (u16)(2731)
#define S16_10_PHASE_SHIFT     (u16)(1820)
#define S16_1_PHASE_SHIFT      (u16)(182)     /* 1�Ƚǹ�һ������ 1/360*65536, �Ƕ�0~360�ȱ仯����Ӧ��һ������0~65535 */

#define HALL_COM_TIMEOUT       BIT0           /* ��ʱ������� */
#define HALL_COM_FLG           BIT1           /* �����־��Hall�����仯ʱ��1 */
#define HALL_COM_ERR           BIT2           /* Hall�仯��Ԥ��ֵ��һ�� */
#define HALL_RUN_ERR           BIT6           /* Hall ���г��� */
#define HALL_DIR_FLG           BIT7           /* ������з����־��0:��ת 1:��ת */

#define REDUCE_TOQUE_PUSE      0x01           /* ����ٶȽϵ�ʱ�����򽵵ͻ��������ת�ز�����־ */
#define REDUCE_TOQUE_MINUS     0x02           /* ����ٶȽϵ�ʱ�����򽵵ͻ��������ת�ز�����־ */

#define HALL_SPEED_FIFO_SIZE   ((u8)6)        /* Hall�źŴ���Buffer��С��6����ƽ�� */


/* -----------------------Hall�ź��ٶȼ����໷��غ궨�� -------------------------------- */

#define ROTOR_SPEED_FACTOR_1US         (MCU_MCLK/6.0*65536)          /* 1��ʱ�ӽ����ٶ�ֵ���ۣ�360�ȶ�Ӧ65536 */
/* ����ٱ���ֵ����Ӧһ��Hall����ֻ����1��ʱ�����ڵ���� */
/* ���ٶȼ���ϵ��,Ϊ������ٶȵı�����: һ��PWM�����ڽǶ��ۼ�ֵ = ROTOR_SPEED_FACTOR/Hall�仯����, ����Hall PLL����͵�ǰת���ٶ�ֵ���� */
#define ROTOR_SPEED_FACTOR             (u32)(ROTOR_SPEED_FACTOR_1US/PWM_FREQ) 

#define ROTOR_SPEED_FACTOR_SAT         (u32)(ROTOR_SPEED_FACTOR - 100)        /* ���ٶȼ���ϵ�����ʹ��� */
#define ROTOR_SPEED_LOW_FACTOR         (u32)(ROTOR_SPEED_FACTOR*2/3)         /* ���ٱ���ֵ */
#define ROTOR_SPEED_FACTOR_REDUCE      (u32)(ROTOR_SPEED_FACTOR*3/2)         /* ������ʱ��Ϊ����Ť�ز������ٶ�ϵ������ֵ */
#define ROTOR_SPEED_FACTOR_PER_UNIT    (u16)(MCU_MCLK/PWM_FREQ)              /* PWM���ڶ�Ӧ��ʱ�����ڣ����ٶ�ϵ����ϵ */
#define ROTOR_SPEED_FACTOR_PER_UNIT1   (u16)(ROTOR_SPEED_FACTOR_PER_UNIT/3) 
#define ROTOR_FACTOR_REDUCE_PER_UNIT   (u16)(ROTOR_SPEED_FACTOR_PER_UNIT)    /* ����ʱ, ���ٶ�ϵ�������ϵ */
#define ROTOR_FACTOR_ECC_UNIT          (u16)(ROTOR_SPEED_FACTOR_PER_UNIT*3/5)/* ���ദ��ʱ, ���ٶ�ϵ�������ϵ */

#define RPM_TO_DPP                     (65536/PWM_FREQ/60.0)                 /* 1Rpm�Ծ�Dppֵ��DPP:ÿ��PWM�����½Ƕ�����ֵ��0~65536��Ӧ0~360�� */
#define DPP_TO_RPM                     (1.0/RPM_TO_DPP)                        /* 1��λ�Ƕ�������ӦRpmֵ */
#define MOTOR_POLES                    (u16)(35)                             /* ��������� */
#define MAX_SPEED_RPM                  (u32)(2000)                           /* ������Rpmֵ */
#define MAX_SPEED_DPP                  (u16)(MAX_SPEED_RPM * MOTOR_POLES * RPM_TO_DPP)/* ϵͳ���Ƶ����Dppֵ */ 

#define RPM_TO_PERIOD(rpm)             (u32)(MCU_MCLK/(rpm/60.0*6))          /* RPMתHall����ֵ����λ��ʱ�Ӽ���ֵ  */

#define MOROR_SPEED_RPM_150            (150)                                 /* ����ٶȣ� 150RPM 1�Լ� */
#define MOROR_SPEED_RPM_160            (160)                                 /* ����ٶȣ� 160RPM 1�Լ� */
#define MOROR_SPEED_RPM_130            (130)                                 /* ����ٶȣ� 130RPM 1�Լ� */
#define MOROR_SPEED_RPM_1250           (1250)                                /* ����ٶȣ� 750RPM 1�Լ� */
#define MOROR_SPEED_RPM_750            (750)                                 /* ����ٶȣ� 750RPM 1�Լ� */
#define MOROR_SPEED_RPM_640            (640)                                 /* ����ٶȣ� 640RPM 1�Լ� */
#define MOROR_SPEED_RPM_550            (550)                                 /* ����ٶȣ� 550RPM 1�Լ� */

#define HALL_PLL_OPEN2CLOSE_THD        RPM_TO_PERIOD(MOROR_SPEED_RPM_160)    /* HALL PLL �����бջ��ٶ�����ֵ */
#define HALL_PLL_CLOSE2OPEN_THD        RPM_TO_PERIOD(MOROR_SPEED_RPM_130)    /* HALL PLL �����бջ��ٶ�����ֵ */

#define LOW_CLOSE_SPEED_THD            RPM_TO_PERIOD(MOROR_SPEED_RPM_1250)   /* Hall���໷����ջ����� */
#define LOW_CLOSE_SPEED_THD1           RPM_TO_PERIOD(MOROR_SPEED_RPM_750)    /* Hall���໷����ջ�����1 */
#define LOW_CLOSE_SPEED_THD2           RPM_TO_PERIOD(MOROR_SPEED_RPM_550)    /* Hall���໷����ջ�����2 */

#define HALL_FAST_ADD_DPP              (u16)(MOROR_SPEED_RPM_150 * RPM_TO_DPP)/* �쳣�����Ƕȿ����ۼ�Dppֵ */ 

typedef const struct
{
    s32 RotorSpeed_FACTOR;           /* ���ٶȼ���ϵ��,Ϊ������ٶȵı����� */
    s32 RotorSpeed_FACTOR_33;        /* 1/3���ٶȼ���ϵ�� */
    s32 RotorSpeed_FACTOR_SAT;       /* ���ٶȼ���ϵ�����ʹ��� */
    s16 RotorSpeed_FACTOR_PER_UNIT;  /* PWM���ڶ�Ӧ��ʱ�����ڣ����ٶ�ϵ����ϵ */
    s16 RotorSpeed_FACTOR_PER_UNIT1; /* PWM���ڶ�Ӧ��ʱ�����ڣ����ٶ�ϵ����ϵ */
    s16 RotorSpeed_REDUCE_PER_UNIT;  /* ����ʱ, ���ٶ�ϵ�������ϵ */
    s16 RotorSpeed_ECC_UNIT;         /* ���ദ��ʱ, ���ٶ�ϵ�������ϵ */

    u32 LowCloseSpeed_THD;           /* Hall���໷����ջ����� */
    u32 LowCloseSpeed_THD1;          /* Hall���໷����ջ�����1 */
    u32 LowCloseSpeed_THD2;          /* Hall���໷����ջ�����2 */

    u16 hPWM_PERIOD;                 /* PWM����ֵ */

    u16 hPWM_TIME_1MS;               /* ��PWM����ֵΪʱ�ڣ�����1ms�������� */
    u16 hPWM_TIME_2MS;               /* ��PWM����ֵΪʱ�ڣ�����1ms�������� */
    u16 hPWM_TIME_4MS;               /* ��PWM����ֵΪʱ�ڣ�����4ms�������� */
    u16 hPWM_TIME_10MS;              /* ��PWM����ֵΪʱ�ڣ�����10ms�������� */
    u16 hPWM_TIME_20MS;              /* ��PWM����ֵΪʱ�ڣ�����20ms�������� */

    u32 hHALL_LEARN_ADDR;            /* HallѧϰFlash�洢��ַ */

} Stru_HallParaConstDef, *pStru_HallParama;

typedef struct
{
    u8 bLast_hall;           /* ��һ��Hall״̬ */
    u8 bBeforeLast_hall;     /* ����һ��Hall״̬ */
    u8 bCheckCnt;            /* �˲����� */
    u8 bBrokenHallFlg;       /* ��Hall��־ BIT0 BIT1 BIT2��Ӧ3��Hall, ��λ�Ĵ������Hallλ�� */
    u8 bOldBrokenHallFlg;    /* ��һ���𻵱�־ */
    u8 bBrokenTypeFlg;       /* ������ */
    u8 bNeed_DelayComm;      /* ��Ҫ��ʱ�����־ */
    u8 bHallState_org;       /* Hallԭʼ�ź� */
    u8 bRepareDirect;        /* �޸���������Ԥ�����Ԥ�� */
    u16 bDelay_timeCnt;      /* ��ʱʱ������� */
    u8 bChangedHall;         /* ��ǰ�仯��Hall״̬ */

    u16 nFocusCommCnt;       /* ǿ�ƻ�������� */

    u8 bCommErrCnt;          /* ���������������� */
    u16 nSeekAngleLowThd;    /* 60���������޺����� */
    u16 nSeekAngleHighThd;   /* 60���������޺����� */

    u8 nSeekAngleFlg;        /* ��ȷ�Ƕ�������־ */

    u16 nSeekAngleIncValue;  /* �Ƕ�����ʱ�ĽǶ����� */


} stru_HallRepairDef;

typedef struct
{
    u8  bFstHallSigFlg;                      /* ��һ��Hall�仯��־������Hall�ź�ȥ�� */
    u32 wOldSensorPeriod;                    /* �ϴ�Hall�ź��˲�������ֵ */
    u32 wMotorSpeedAvgCnt;                   /* Hall�ź��˲�������ֵ, ��HAllģ��ʱ�����ڼ��� */
    u32 wSensorPeriod[HALL_SPEED_FIFO_SIZE]; /* Hall�ź����ڱ仯���飬�������ź�ƽ��ֵ */
    u8  bSpeedFIFO_Index;                    /* Hall�ź����ڱ仯����ָ�� */
    volatile u32 wHallPWMTimerCnt;           /* Hall�仯������PWM�������������������ֵ */
    u16 nElectrical_Angle;                   /* ��ǰת��λ�ýǶ�ֵ��0~65535��Ӧ0~360�� */
    u16 nOldElectrical_Angle;                /* ��һ��ת��λ�ýǶ�ֵ��0~65535��Ӧ0~360�� */  
    s16 nEndElectricalAngle;                 /* ���Ƕ���������ֵ����Hall�������ʱ�����������һ���ȷ�Ƕ� */
    u16 nTabElectrical_Angle;                /* ֱ�Ӳ��ȵ��ĵ�ǰת�ӵ�Ƕ�ֵ */
    u16 nRotorFreqDpp;                       /* ��ǰ���ٶ�ֵ, һ��PWM�����µĽǶ�����ֵ��0~65536��Ӧ0~360�� */
    u16 nRotorFreqDppFir;                    /* ��ǰ���ٶ�ֵ(�˲���), һ��PWM�����µĽǶ�����ֵ��0~65536��Ӧ0~360�� */
	  u16 nReduceToqueDpp;                     /* ������ģʽ�µ�ǰ���ٶ�ֵ, һ��PWM�����µĽǶ�����ֵ��0~65536��Ӧ0~360�� */
    u16 nReduceToqueAngle;                   /* ����ģʽ��, �Ƕ����� */
    u32 wHallCapCnt;                         /* Hallģ�鲶׽�ۼ�ֵ */
    s16 nMaxReduceIncAngle;                  /* ��Reduce Toqueģʽ�£��Ƕ�������� 0~65536��Ӧ0~360�� */
  
    u8 bHallState;                           /* ��ǰ�˲����Hall״̬ */
    u8 bOldHallState;                        /* ��һ���˲����Hall״̬ */ 
    u8 bHallStateOrg;                        /* �˲�ǰHall ԭʼֵ */

    u16 nCommErrType1Cnt;                    /* Hall�˲�����ʱ���趨 */
    u16 nCommErrType2Cnt;                    /* Hall�˲�����ʱ���趨 */
    u16 nHallCheckCnt;                       /* Hall�˲�������*/
    u16 nHallCheckCnt2;                      /* Hall�˲�������*/

    u8 bMotorDirtionCtrl;                    /* �������Ƶĵ�����з��� */

    u16 nOverAngleCnt;                       /* �Ƕ��ۼӳ�����������ֵ */
    u8 bHallType;                            /* Hall���ͣ�60��Hall��120��Hall */ 
    u8 bHallRunFlg;                          /* Hall����״̬��־�����ϡ����򡢳�ʱ��״̬ HALL_COM_TIMEOUT��
                                                  HALL_COM_FLG��HALL_COM_ERR��HALL_DIR_FLG*/
    u8 bHallLearnFlg;                        /* Hall������ѧϰ��־ ����������ѧϰ״̬*/                                                  
    u16 nLearnHallTimeCnt;                   /* Hallѧϰ��������ֵ  */ 
    u16 nLearnHallStep;                      /* ��ǰHall��ѧϰ���в��� */
    u8 bHallLearnStateFlg;                   /* Hallѧϰ״̬ */
    u8 bMotorTypeLearnAngle;                 /* Hall��ѧϰ����ƫ�ƽ� */

    u8 bLearn_AngleOffset;                   /* ������ģʽѧϰHall�Ƕ�ƫ�Ʊ�־ */
    u8 bLearn_AngleCnt;                      /* ������ģʽѧϰHall�Ƕ�ƫ�Ƽ��� */
    u8 bFirHallState;                        /* Hall��ѧϰʱHall�˲�״̬ */
    s32 wHallErrTimeMark;                    /* Hall��ѧϰʱHall�˲�ʱ����� */

    
    u8 bReduceToqueFlg;                      /* Reduce Toqueģʽ������־������ٶȹ��ͣ������ڴ�ģʽ */ 
    u8 bFastAddElecAngleFlg;                 /* ��Hall�������ʱ�����������ۼӵ���ȷ�ĽǶ� */
    u8 bCloseLoopCnt;                        /* Hall�ջ����м������Ƿ��������໷ */
    u8 bCloseLoopAngleFlg;                   /* Hall�Ƕ���ϣ�PLL�ջ�������־  */
    u16 nHallChangeCnt;                      /* Hall�仯һ�μ�����ֵ��1,������0 ȡֵ��0~65536 */  
    s16 nHallQEP_Cnt;                        /* Hallģ�������ģʽ������������ֵ, ȡֵ��S16_MIN~S16_MAX */
  
    u16 nPhaseShift;                         /* ��ǰHall�Ƕȼ���Ƕ�ƫ��ֵ */
    u16 nPhaseShiftOffset;                   /* ��ǰHall�Ƕ�ƫ��ֵ���� */
    
    u32 wCloseLoopPeriodThd;                 /* HALL PLL�ջ���������ֵ */
    u32 wOpenLoopPeriodThd;                  /* HALL PLL������������ֵ */
    u8  bOpenStartUpCntThd;                  /* ����״̬����ת�ջ���������ֵ */
    
    u16 nMaxIncAngle;                        /* ���Ƕ����� */
    u16 nCloseLoopMaxAngle;                  /* PLLģʽ�����Ƕ����� */
    
    stru_RC_Def stru_HallDppRC;              /* Hall���ٶȵ�ͨ�˲� */
    
    enum_SystStatus eMotorState;             /* ��ǰ�������״̬ */
    u8 bHallErrorCnt;                        /* Hall���д������ */
    
    u8 bCommErrType1;                        /* Hall�˲����������ͱ�־ */
    u8 bCommErrType2;                        /* Hall�˲����������ͱ�־ */  




    u16 nHallCaliCnt;                        /* Hallƫ�ƽ�У��������� */
    u8  bCalibrationHallFlg;                 /* Hallƫ�ƽ�У����־ */
    u16 nHallOffsetArry[6];                  /* Hallƫ�ƽ����������໷���������Hall��װ����Ļ�е��װƫ�ƽ� */
    s16 nHallCaliArray[6];                   /* Hallƫ�ƽ�У��ֵ���顡*/
    
    u8  bHallPossableTab[8];                 /* Hall������洢��һ��Hallֵ */
    u8  bHallPossableTab2[8];                /* Hall������洢��һ��Hallֵ */
    u8  bHallCommTab[8];                     /* Hall����� */
    
    pStru_HallParama pStru_HallParama;       /* Hall�������Const���� */
    
    stru_HallRepairDef stru_HallRepare;      /* Hall�޸��ṹ�� */
		
		s32 wSpeedfbkHall;                       /* ��ǰ���ٶ�ֵ(�˲���),������ϵϵ��ת��Ϊ�ջ�core�ٶ� ADD*/

		//u8 gHall_ExpHallEdge;                   /* ��ǰhall��һʱ�̵�hall״̬�������ֵ ADD*/
		 s8 bHallSpeedDirFlag;                    /* ��ǰhall��һʱ�̵�hall״̬�������ֵ ADD*/
 
} stru_HallProcessDef;

/*******************************************************************************
 �������ƣ�    u8 ReadHallState(HALL_TypeDef *HALLx)
 ����������    ������ǰHallʵʱ״̬
 ���������    Hallͨ����
 ���������    ��
 �� �� ֵ��    ��ǰHallʵʱ״̬
 ����˵����    ��̬���������ٳ���ִ���ٶ�
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
static __inline u8 ReadHallState(HALL_TypeDef *HALLx)
{
    volatile u8 t_ReadValue;

    t_ReadValue = HALLx->INFO & 0x07;
	
	  //add 120�㲻��Ҫ���;60����Ҫ���
//	  t_ReadValue ^= (BIT0 | BIT2);

    return(t_ReadValue);   
}

/*******************************************************************************
 �������ƣ�    u8 ReadHallCapState(HALL_TypeDef *HALLx)
 ����������    �����ϴ�Hall��׽����״ֵ̬
 ���������    Hallͨ����
 ���������    ��
 �� �� ֵ��    ��ǰHallʵʱ״̬
 ����˵����    ��̬���������ٳ���ִ���ٶ�
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Howlet Li          ����
 *******************************************************************************/
static __inline u8 ReadHallCapState(HALL_TypeDef *HALLx)
{
    volatile u8 t_ReadValue;

    t_ReadValue = (HALLx->INFO >> 8) & 0x07;
	
	//add 120�㲻��Ҫ���;60����Ҫ���
//	  t_ReadValue ^= (BIT0 | BIT2);

    return(t_ReadValue);
}         

u32 GetAvrgHallPeriod(stru_HallProcessDef *struHallProcess);
u8 check_hall_state(u8 t_hall, stru_HallProcessDef *struHallProcess);
void HALL_Init_Electrical_Angle(stru_HallProcessDef *struHallProcess);
void Angle_Init_Process(stru_HallProcessDef *struHallProcess);
void HALL_IRQProcess(HALL_TypeDef *HALLx, stru_HallProcessDef *struHallProcess);
void Verify_Hall_State(HALL_TypeDef *HALLx, stru_HallProcessDef *struHallProcess);
void GetHall_edgeAngle(stru_HallProcessDef *struHallProcess);
s16 lowPass_filter(stru_RC_Def *rc,s16 signal);
void closeLoopAnglePLLInit(u16 t_hElectrical_Angle, stru_HallProcessDef *struHallProcess);
void calc_first_ElectAngle(u16 angle,stru_HallProcessDef *struHallProcess);
void HALL_InitHallMeasure(stru_HallProcessDef *struHallProcess);
void Hall_FastAdd_ElecAngle(stru_HallProcessDef *struHallProcess);

void closeLoopAnglePLL(stru_HallProcessDef *struHallProcess);
void reduceToqueAnglePll(stru_HallProcessDef *struHallProcess);

void Hall_ElecAnglePWM_Process(stru_HallProcessDef *struHallProcess);

void learn_hall_proc(stru_HallProcessDef *this);

void Hall_learn_Process(void);

#define GET_HALL_LEARN_STATE()          (struHallProcess.bHallLearnFlg)     /* �õ�HALLѧϰ״̬ */
#define HALL_LEARN_FINISH               3                                   /* HALL��ѧϰ���� */

#endif /* __HALL_H */


/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
