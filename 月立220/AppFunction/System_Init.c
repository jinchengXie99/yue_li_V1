/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� Time_Process.c
 * �ļ���ʶ��
 * ����ժҪ�� ��ʱ��غ���
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

/* Includes ------------------------------------------------------------------*/
#include "Global_Variable.h"
#include "hardware_config.h"
#include "function_config.h"
#include "fault_detection.h"
#include "MC_parameter.h"
#include "math.h"
#include "segger_rtt.h"

void CurrentOffsetCalibration(void);
void FluxObserveInit(void);
extern stru_FluxOBS_Def m_FluxOBSPara;               /* �۲������� */

/*******************************************************************************
 �������ƣ�    int sys_init(void)
 ����������    ϵͳ������ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void sys_init(void)
{
    CurrentOffsetCalibration();               /* ��ȡ��������ͨ��ƫ�� */

    FluxObserveInit();                        /* �۲���������ʼ�� */

    #if ((ROTOR_SENSOR_TYPE == ROTOR_HALL_SENSOR) || (ROTOR_SENSOR_TYPE == ROTOR_HALL2SENSORLESS))
    HALL_InitHallMeasure(&struHallProcess);   /* HALL��������ʼ�� */
    #endif

    FaultInit();                              /* ���ϼ���ʼ�� */

    FaultRecoverInit();                       /* ���ϻָ���ʼ�� */

    struTaskScheduler.sVersion = &sVersion[0];/* ��ʼ���汾�� */

    #if (RTT_FUNCTION == FUNCTION_ON)
    /* JScope RTTģʽ��ʼ�� */
    SEGGER_RTT_ConfigUpBuffer(1, "JScope_i2i4", bRttBuf, 200, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    #endif

    #if (DEBUG_PWM_OUTPUT == TEST_ON)
    DebugPWM_OutputFunction(); /* ���Ե�ʱ�����25%��PWM���� */
    #endif
}


/*******************************************************************************
 �������ƣ�    void CurrentOffsetCalibration(void)
 ����������    ������Offsetֵ
 ���������    stru_FOC_CtrProcDef *this  �ṹ��ָ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void CurrentOffsetCalibration(void)
{
    u16 CalibCnt = 0;
    volatile u32 t_delay;
    stru_OffsetDef  struOffset;

    __disable_irq();


    ADC_SOFTWARE_TRIG_ONLY();

    ADC_STATE_RESET();

    ADC_NormalModeCFG();

    struFOC_CurrLoop.mVoltUVW_PWM.nPhaseU = 0;
    struFOC_CurrLoop.mVoltUVW_PWM.nPhaseV = 0;
    struFOC_CurrLoop.mVoltUVW_PWM.nPhaseW = 0;
    MCPWM0_RegUpdate();
    PWMOutputs(MCPWM0, ENABLE);
    for(t_delay = 0; t_delay < 0x7ffff; t_delay++);

    struOffset.IPhAFilt    = 0;
    struOffset.IPhBFilt    = 0;
    struOffset.UBusFilt    = 0;
    struOffset.IBusFilt    = 0;

    for(CalibCnt = 0; CalibCnt < (1 << ADC_GET_OFFSET_SAMPLES); CalibCnt++)
    {
        /* Clear the ADC0 JEOC pending flag */
        ADC_SWT = 0x00005AA5;

        while(!(ADC_IF & BIT0));

        ADC_IF |= BIT1 | BIT0;
        ADC_STATE_RESET();
        struOffset.IPhAFilt +=  (s16)((ADC_DAT0));
        struOffset.IPhBFilt +=  (s16)((ADC_DAT3));
        struOffset.IPhCFilt += (s16)((ADC_DAT5));
        struOffset.IBusFilt += (s16)(ADC_DAT3);
    }

    ADC_init();

    ADC_NormalModeCFG();

    __enable_irq();


    struFOC_CurrLoop.nPhaseAOffset = (s16)(struOffset.IPhAFilt >> ADC_GET_OFFSET_SAMPLES);
    struFOC_CurrLoop.nPhaseBOffset = (s16)(struOffset.IPhBFilt >> ADC_GET_OFFSET_SAMPLES);
    struFOC_CurrLoop.nPhaseCOffset = (s16)(struOffset.IPhCFilt >> ADC_GET_OFFSET_SAMPLES);

    if((ABS(struFOC_CurrLoop.nPhaseAOffset) > ADC_OFFSET_ERROR) ||
            (ABS(struFOC_CurrLoop.nPhaseBOffset) > ADC_OFFSET_ERROR) ||
            (ABS(struFOC_CurrLoop.nPhaseCOffset) > ADC_OFFSET_ERROR))
    {
        stru_Faults.B.OffsetError = 1;
    }




}

/*******************************************************************************
 �������ƣ�    void FluxObserveInit(void)
 ����������    �۲���������ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void FluxObserveInit(void)
{
    struFluxOB_Param.nSTO_KP = PLL_KP_GAIN;              /* �۲���PLL KP ���� */
    struFluxOB_Param.nSTO_KI = PLL_KI_GAIN;              /* �۲���PLL KI ���� */

    struFluxOB_Param.nSTO_KI_H = PLL_KI_GAIN_L;
    struFluxOB_Param.nSTO_KP_H = PLL_KP_GAIN_L;

    struFluxOB_Param.nCurrLoopIniKP = P_CURRENT_KP;      /* ��������ʼKP */
    struFluxOB_Param.nCurrLoopIniKI = P_CURRENT_KI;      /* ��������ʼKI */


    struFluxOB_Param.nCurrLoopIniKPL = P_CURRENT_KP_H;      /* ��������ʼKP */
    struFluxOB_Param.nCurrLoopIniKIL = P_CURRENT_KI;      /* ��������ʼKI */

    struFluxOB_Param.pStruMotorParame = &struMotorParam; /* ��������ṹ�� */
    struFluxOB_Param.pStruBoardParame = &struBoardParam; /* ���ʰ�Ӳ����·���Բ��� */


    m_FluxOBSPara.nMinOutFreq = OBS_MIN_OUT_SPEED;       /* �۲�����С���Ƶ�� */
    m_FluxOBSPara.nMaxOutFreq = OBS_MAX_OUT_SPEED;       /* �۲���������Ƶ�� */

    FluxObserveParaCalc();                               /* �۲����������� */

    FOC_InitstruParam();                                 /* FOC ��ؿ��Ʊ������ṹ���ʼ�� */

//    m_FluxOBSPara.nObsFactor = 65;    //3
//    m_FluxOBSPara.bDivisor = 3;     //4
//    m_FluxOBSPara.nObsCoef = FRAC16(0.25);                /* �۲����˲�ϵ�� */

    m_FluxOBSPara.nObsFactor = START_OBSFACTOR;
    m_FluxOBSPara.bDivisor = START_DIVISOR;

    m_FluxOBSPara.nObsCoef = OBS_COEF;                /* �۲����˲�ϵ�� */

}



/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
