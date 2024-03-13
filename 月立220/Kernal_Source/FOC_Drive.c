/*******************************************************************************
 * ��Ȩ���� (C)2016, LINKO Semiconduct
 *
 * �ļ����ƣ� FOC.Drive.c
 * �ļ���ʶ��
 * ����ժҪ�� ���ʸ�����Ƶ����ڻ����ؼ������㷨
 * ����˵���� ��
 * ��ǰ�汾�� V 2.0
 * ��    �ߣ� WhenCai Zhao Howlet Li
 * ������ڣ� 2020��9��10��
 *
 * �޸ļ�¼2��
 *    �޸����ڣ�2020��9��10��
 *    �� �� �ţ�V2.0
 *    �� �� �ˣ�Howlet Li
 *    �޸����ݣ���ʽ����
 *
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "FOC_Drive.h"
#include "global_variable.h"
#include "hardware_config.h"
#include "mc_math.h"
#include "MC_Parameter.h"

extern s16 gPmWeakId,gPmWeakIq;
extern u16 g_FluxObsMode;
/*******************************************************************************
 �������ƣ�    void AdcSampleCal(stru_FOC_CtrProcDef *this)
 ����������    ��������
 ���������    stru_FOC_CtrProcDef *this �ṹ��ָ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����    1.�����������ɼ�AB�������
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0            Wencai.Zhao Howlet Li          ����
 *******************************************************************************/
  s16 Mag_nCurr;
void AdcSampleCal(stru_FOC_CurrLoopDef *this)
{
    stru_CurrPhaseUVW t_StatCurrUV;
	
    #if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
    t_StatCurrUV = SVPWM_1ShuntGetPhaseCurrent(this); /* ��ȡADC�������������ֵ */
    #else

    #if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
    t_StatCurrUV = SVPWM_2ShuntGetPhaseCurrent(this); /* ��ȡADC�������������ֵ */
    #else
    #if ((CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT)||(CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
    t_StatCurrUV = SVPWM_3ShuntGetPhaseCurrent(this); /* ��ȡADC�������������ֵ */
    #endif
    #endif
    #endif

    /* �������ۻ����� */
    this->mStatCurrUVW.nPhaseU = _IQ15mpy(t_StatCurrUV.nPhaseU, struFluxOB_Param.CurrentPerUnitGAIN);
    this->mStatCurrUVW.nPhaseV = _IQ15mpy(t_StatCurrUV.nPhaseV, struFluxOB_Param.CurrentPerUnitGAIN);
    this->mStatCurrUVW.nPhaseW = -(this->mStatCurrUVW.nPhaseU + this->mStatCurrUVW.nPhaseV);

    /* clark�任 */
    this->mStatCurrAlfaBeta.nBeta = _IQ15mpy(18919, (this->mStatCurrUVW.nPhaseV - this->mStatCurrUVW.nPhaseW));
    this->mStatCurrAlfaBeta.nAlph = this->mStatCurrUVW.nPhaseU;
		
		Mag_nCurr = CurrentMagCalc(this->mStatCurrAlfaBeta.nAlph,this->mStatCurrAlfaBeta.nBeta);
		
		
}

/*******************************************************************************
 �������ƣ�    void FOC_Model(stru_FOC_CtrProcDef *this)
 ����������    PMSM���Q��ת�ؿ��ƣ�D��������ƴ���ʵ��FOC���Ƶĺ��ĵ�����
 ���������    stru_FOC_CtrProcDef *this �ṹ��ָ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0       Wencai.Zhao Howlet Li    ����
 *******************************************************************************/
extern stru_CurrPhaseUVW struBemfVolt;
volatile u16 openloopAngle, openloopAngleStep;
void FOC_Model(stru_FOC_CtrProcDef *pCtrProc)
{

    stru_FOC_CurrLoopDef *this;

    //struFOC_CurrLoop.mStatVoltDQ.nAxisQ

    this = &struFOC_CurrLoop;
 
    AdcSampleCal(this);          /* �������� */
    
    switch(pCtrProc->eSysState)
    {
        case OPEN_RUN:
        case ALIGN:
        case RUN:
        {
            u16 tElectAngle;

            /* Park�任����qIalpha��qIbeta�任����ת��dq����ϵ��  */
            this->mStatCurrDQ.nAxisQ  = _IQ15mpy(this->mStatCurrAlfaBeta.nBeta, this->mTrigSinCos.hCos) -
                                        _IQ15mpy(this->mStatCurrAlfaBeta.nAlph, this->mTrigSinCos.hSin);
            this->mStatCurrDQ.nAxisD  = _IQ15mpy(this->mStatCurrAlfaBeta.nAlph, this->mTrigSinCos.hCos) +
                                        _IQ15mpy(this->mStatCurrAlfaBeta.nBeta, this->mTrigSinCos.hSin);

            CurrentLoopReg(this);   /* D Q�������PI����*/

            //out :this->mStatVoltAlfaBeta.nAlph ,this->mStatVoltAlfaBeta.nBeta

            #if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_1SHUNT)
            SVPWM_1ShuntGEN(this);     /*Valpha Vbeta ����SVPWM�ռ�ʸ������ �����跽ʽ */
            #else
            #if (CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_2SHUNT)
            SVPWM_2ShuntGEN(this);     /*Valpha Vbeta ����SVPWM�ռ�ʸ������ ˫���跽ʽ */
            #else
            #if ((CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_3SHUNT)||(CURRENT_SAMPLE_TYPE == CURRENT_SAMPLE_MOSFET))
            SVPWM_3ShuntGEN(this);     /*Valpha Vbeta ����SVPWM�ռ�ʸ������ �����跽ʽ*/
            #endif
            #endif
            #endif
            this->MCPWMx_RegUpdate();  /* ����MCPWMģ��ռ�ձ�ֵ������MCPWMģ��ADC������Ĵ���ֵ */

//            g_FluxObsMode = 0;    /* �������л� */
            
            #if (ROTOR_SENSOR_TYPE == ROTOR_SENSORLESS)
            PmsmFluxObserve();         /* �۲����Ƕȹ��� */
            #endif

            OpenCloseAngleSwitch();  /* �����Ƕȣ��ջ��Ƕ��л� */

            GET_ELECT_ANGLE();

            /* ����ģʽ��ǿ������, ǿ�Ƹ���ѹ */
            #if (OPEN_LOOP_FORCE_TEST == TEST_ON)
            openloopAngle += openloopAngleStep;
            struFluxOB_Param.wElectAngle = openloopAngle << 16;
            struFluxOB_Param.wElectAngleOpen = struFluxOB_Param.wElectAngle;
            tElectAngle = openloopAngle;
            #endif

            this->mTrigSinCos = Trig_Functions(tElectAngle); /* sincos��� */
            break;
        }
        case BRAKE:
        {
            /* ֱ���ƶ� */
       
            this->mVoltUVW_PWM.nPhaseU = 0;
            this->mVoltUVW_PWM.nPhaseV = 0; /* ����ȫ�� ɲ�� */
            this->mVoltUVW_PWM.nPhaseW = 0;

            this->mVoltUVW_NegPWM.nPhaseU = 0;
            this->mVoltUVW_NegPWM.nPhaseV = 0; /* ����ȫ�� ɲ�� */
            this->mVoltUVW_NegPWM.nPhaseW = 0;

            this->MCPWMx_RegUpdate();       /* ����MCPWMģ��ռ�ձ�ֵ������MCPWMģ��ADC������Ĵ���ֵ */

            break;
        }
        default:
            break;
    }


}

/*******************************************************************************
 �������ƣ�    CurrentLoopReg(stru_FOC_CurrLoopDef *this)
 ����������    D��Q��������㣬���Vd��Vq
 ���������    stru_FOC_CurrLoopDef *this  �ṹ��ָ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
s16 watchiq;
void CurrentLoopReg(stru_FOC_CurrLoopDef *this)
{

    //  this->nQCur_Reference = 1000;

    this->mPI_Torque.wInError = this->nQCur_Reference - this->mStatCurrDQ.nAxisQ; /* Q��������� */
    this->mPI_Flux.wInError   = this->nDCur_Reference - this->mStatCurrDQ.nAxisD; /* D��������� */
watchiq = this->mStatCurrDQ.nAxisQ;
    /* ����޷����ɸ���ʵ�ʸ��ص�����Ӱ��PI��Ӧ�Ŀ��� */
    // this->mPI_Torque.wInError = sat(this->mPI_Torque.wInError, -0x800, 0x800);
    // this->mPI_Flux.wInError   = sat(this->mPI_Flux.wInError, -0x800, 0x800);

    this->mStatVoltDQ.nAxisQ = CurrentPIRegulator(&this->mPI_Torque);   /* Q�����PI���㣬���Vq */
    this->mStatVoltDQ.nAxisD = CurrentPIRegulator(&this->mPI_Flux);     /* D�����PI���㣬���Vd */

    /* ����ģʽ��ǿ������, ǿ�Ƹ���ѹ */
    #if (OPEN_LOOP_FORCE_TEST == TEST_ON)
    this->mStatVoltDQ.nAxisQ = 400;
    this->mStatVoltDQ.nAxisD = 0;
    #endif

    ModuCircle_Limitation();  /* ��ѹ����Բ���� */
}


/*******************************************************************************
 �������ƣ�    void SpeedLoopReg(MechanicalQuantity *this)
 ����������    �ٶȻ�PI����
 ���������    MechanicalQuantity *this  �ṹ��ָ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/10/8      V1.0           Howlet Li          ����
 *******************************************************************************/
void SpeedLoopReg(MechanicalQuantity *this)
{
    #if (ROTOR_SENSOR_TYPE == ROTOR_HALL_SENSOR)
    this->mSpeedPI.wInError =  this->wSpeedRef - struHallProcess.wSpeedfbkHall;
    #else
    this->mSpeedPI.wInError =  this->wSpeedRef - this->wSpeedfbk;

    #endif
    this->mSpeedPI.wInError = sat(this->mSpeedPI.wInError, -32767, 32767);

    //struFOC_CurrLoop.nQCurrentSet = SpeedPIRegulator(&this->mSpeedPI);
	struFOC_CurrLoop.nQCur_Reference = SpeedPIRegulator(&this->mSpeedPI)  +0*gPmWeakIq;
}

/*******************************************************************************
 �������ƣ�    void FOC_InitstruParam(void)
 ����������    FOC ��ؿ��Ʊ������ṹ���ʼ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void FOC_InitstruParam(void)
{
    struFOC_CurrLoop.pStruFOC_ConstPar = &struFOC_ConstParam;

    struFOC_CurrLoop.mPI_Torque.KP = struFluxOB_Param.nQ_CurrLoopKP;
    struFOC_CurrLoop.mPI_Torque.KI = struFluxOB_Param.nQ_CurrLoopKI;
    /* Lower Limit for Output limitation */
    struFOC_CurrLoop.mPI_Torque.wLowerLimitOutput = (s16)VQMIN * 65536;
    /* Upper Limit for Output limitation */
    struFOC_CurrLoop.mPI_Torque.wUpperLimitOutput = (s16)VQMAX * 65536;
    struFOC_CurrLoop.mPI_Torque.wIntegral = 0;

    struFOC_CurrLoop.mPI_Flux.KP = struFluxOB_Param.nD_CurrLoopKP;
    struFOC_CurrLoop.mPI_Flux.KI = struFluxOB_Param.nD_CurrLoopKI;
    /* Lower Limit for Output limitation */
    struFOC_CurrLoop.mPI_Flux.wLowerLimitOutput = VDMIN * 65536;
    /* Upper Limit for Output limitation */
    struFOC_CurrLoop.mPI_Flux.wUpperLimitOutput = VDMAX * 65536;
    struFOC_CurrLoop.mPI_Flux.wIntegral = 0;

    struFOC_CurrLoop.MCPWMx_RegUpdate = &MCPWM0_RegUpdate;   /* ��ʼ��MCPWM�Ĵ������º���ָ�� */

    struFOC_CurrLoop.bSVPWM_Type = 1;                       /* 7��ʽSVPWM���Ʒ�ʽ */

    struFOC_CurrLoop.nVoltageCircleLim = 4000;              /* ��ѹ����Բ����ֵ Q12��ʽ�����ֵ4096*/

    struFOC_CurrLoop.nWeakFieldLim = AUTO_FW_LIM;           /* �Զ�����D��������� */

    struFOC_CurrLoop.mPI_Torque_BRAKE.KP = BRAKE_P_CURRENT_KP;
    struFOC_CurrLoop.mPI_Torque_BRAKE.KI = BRAKE_P_CURRENT_KI;
		
    struFOC_CurrLoop.mPI_Flux_BRAKE.KP = BRAKE_P_CURRENT_KP;
    struFOC_CurrLoop.mPI_Flux_BRAKE.KI = BRAKE_P_CURRENT_KI;
		
		
		


//    struFOC_CurrLoop.mPI_Torque_H.KP = (struFluxOB_Param.nQ_CurrLoopKP*3)/2;
//    struFOC_CurrLoop.mPI_Torque_H.KI = (struFluxOB_Param.nQ_CurrLoopKI*3)/2;
//		
//    struFOC_CurrLoop.mPI_Flux_H.KP = (struFluxOB_Param.nD_CurrLoopKP*3)/2;
//    struFOC_CurrLoop.mPI_Flux_H.KI = (struFluxOB_Param.nD_CurrLoopKP*3)/2;
}

/*******************************************************************************
 �������ƣ�    void StopMotorImmdly(stru_FOC_CtrProcDef *this)
 ����������    �����ͣ����
 ���������    stru_FOC_CtrProcDef *this  �ṹ��ָ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void StopMotorImmdly(stru_FOC_CtrProcDef *this)
{


}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

