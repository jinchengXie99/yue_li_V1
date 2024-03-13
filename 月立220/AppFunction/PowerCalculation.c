/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� PowerCalculation.c
 * �ļ���ʶ��
 * ����ժҪ�� ���ʻ�����
 * ����˵���� ��
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2020��10��9��
 *
 *******************************************************************************/
#include "MC_Parameter.h"
#include "PowerCalculation.h"
#include "FOC_Drive.h"
#include "Global_Variable.h"
#include "pmsmFluxObserve.h"


stru_PowerBasic_t struPower;
stru_Ramp_t PowerRamp;

/*****************************************************************************
 * ������   : void PowerLimitCalc(MechanicalQuantity *pstruMotorSpeed, s32 PowerValue)
 * ˵��     : �޹��ʼ���
 * ���˼· ��1.���ʵ�ʹ��ʳ�������ֵ��������ٶȻ����ٶȲο�ֵ��һ�������ٶȻ���ʱ���޹��ʴ���
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 * �޸�ʱ�� : 2020.10.09
 * �޸����� �����������ṹ
 *****************************************************************************/
void PowerLimitCalc(MechanicalQuantity *pstruMotorSpeed, s32 PowerValue)
{
    if(++pstruMotorSpeed->nPowerLimitCntr >= POWER_LIMIT_TIME)                      //�޹��ʼ�������
    {
        pstruMotorSpeed->nPowerLimitCntr  = 0;
        if(PowerValue > pstruMotorSpeed->wPowerLimitValue)                           //ʵ�ʹ��ʴ������ƹ������С�ٶȸ���ֵ
        {
            if(pstruMotorSpeed->wPowerLimitSpeedSet < pstruMotorSpeed->wPowerLimitSpeedRef)    //���ʳ������ƺ������޹������
            {
                pstruMotorSpeed->wPowerLimitSpeedSet += pstruMotorSpeed->wSpeedRampACCStep;
            }
            else
            {
                pstruMotorSpeed->wPowerLimitSpeedSet = pstruMotorSpeed->wPowerLimitSpeedRef;
            }
        }
        else                                                                  //���ʻָ������󣬼�С�޹������
        {
            if(pstruMotorSpeed->wPowerLimitSpeedSet > 0)
            {
                pstruMotorSpeed->wPowerLimitSpeedSet -= pstruMotorSpeed->wSpeedRampDECStep;
            }
            else
            {
                pstruMotorSpeed->wPowerLimitSpeedSet = 0;
            }
        }
    }
}

/*****************************************************************************
 * ������   :void SpeedLimitCalc(s32 wSpeedFbk, stru_PowerBasic_t *pstruPower)
 * ˵��     : ��ת�ټ���
 * ���˼· ��1.���ʵ��ת�ٳ�������ֵ������͹��ʻ��Ĺ��ʲο�ֵ             \
 * ����     ��s32 wSpeedFbk, stru_PowerBasic_t *pstruPower
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 * �޸�ʱ�� ��2020.10.09
 *****************************************************************************/
void SpeedLimitCalc(s32 wSpeedFbk, stru_PowerBasic_t *pstruPower)
{
    if(++pstruPower->nSpeedLimitCntr >= SPEED_LIMIT_TIME)                     //��ת�ټ�������
    {
        pstruPower->nSpeedLimitCntr  = 0;
        if(wSpeedFbk >= pstruPower->wSpeedLimitValue)                        //ʵ��ת�ٴ�������ת�����С���ʸ���ֵ
        {
            if(pstruPower->wPowerSet < pstruPower->wSpeedLimitPowerRef)      //ת�ٳ������ƺ������������
            {
                pstruPower->wPowerSet += pstruPower->struPowerRamp.nACCStep;
            }
            else
            {
                pstruPower->wPowerSet = pstruPower->wSpeedLimitPowerRef;
            }
        }
        else                                                                  //ת�ٻָ������󣬼�С��ת�����
        {
            if(pstruPower->wPowerSet > 0)
            {
                pstruPower->wPowerSet -= pstruPower->struPowerRamp.nDECStep;
            }
            else
            {
                pstruPower->wPowerSet = 0;
            }
        }
    }
}



/*****************************************************************************
 * ������   : void PowerCalc(stru_CurrVoctorDQ *pstruCurrDQ, stru_VoltVoctorDQ *pstruVoltDQ , stru_PowerBasic_t *pstruPower)
 * ˵��     : ���ʼ���
* ���˼·  : 1.P = ABS(Id*Ud) + ABS(Iq*Uq)/Ubus ,��������Ĺ���ֵ��ʵ�ʹ��ʽ���������ϣ����б�ʺͲ�ֵ \
 *          : 2.�����õ���Ϊ׼ȷ�Ĺ��ʣ���Ҫ��ĸ�ߵ�������RC�˲������;P = UBus*IBus.
 * ����     ��stru_CurrVoctorDQ *pstruCurrDQ, stru_VoltVoctorDQ *pstruVoltDQ , stru_PowerBasic_t *pstruPower
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 * �޸�ʱ�� ��2020.10.09
 * �޸����� ��������������
 *****************************************************************************/
void PowerCalc(stru_CurrVoctorDQ *pstruCurrDQ, stru_VoltVoctorDQ *pstruVoltDQ,stru_PowerBasic_t *pstruPower)
{
	pstruPower->BUSCURR = GET_ADC_BUS_CURR_RESULT- struFOC_CurrLoop.nPhaseCOffset;
    //pstruPower->wTemp = (s32)(ABS(pstruCurrDQ->nAxisD * pstruVoltDQ->nAxisD)) + (s32)(pstruCurrDQ->nAxisQ * pstruVoltDQ->nAxisQ);  //���ʼ���
    pstruPower->wTemp =pstruPower->BUSCURR*struFOC_CurrLoop.nBusVoltage;
	
    pstruPower->struPowerLowPass.wInput = (pstruPower->wTemp >> 12);
    pstruPower->wPowerValueLowPass = LowPassControl(&pstruPower->struPowerLowPass);          //��ͨ�˲�
	pstruPower->wPowerValue = (pstruPower->wPowerValueLowPass*POWER_CHECK)>>9;
	
	
	pstruPower->BUSpower = (pstruPower->wTemp*POWER_CHECK1)>>22;//(pstruPower->BUSCURR*struFOC_CurrLoop.nBusVoltage*POWER_CHECK1)>>22;
//    pstruPower->wPowerValue >>= POWER_SHIFT;
}

/*****************************************************************************
 * ������   : void LowPassControl(stru_LowPass_t *LowPass)
 * ˵��     : ��ͨ�˲�
 * ���˼· : 1. ��ͨ�˲�    ��ֹƵ�� f = 1/((2^K1 -1)*Ts*2PI ,����TsΪ��ͨ��������
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
//s32 LowPassControl(stru_LowPass_t *pstruLowPass)
//{
//    pstruLowPass->lTemp += pstruLowPass->wInput - pstruLowPass->wOutput;
//    pstruLowPass->wOutput = pstruLowPass->lTemp >> pstruLowPass->nK1;
//    return (pstruLowPass->wOutput);
//}

s32 LowPassControl(stru_LowPass_t *pstruLowPass)
{
    pstruLowPass->lTemp += (pstruLowPass->wInput * 65536 - pstruLowPass->lTemp) >> pstruLowPass->nK1;
    pstruLowPass->wOutput = pstruLowPass->lTemp >> 16;
    return (pstruLowPass->wOutput);
}

/*****************************************************************************
 * ������   : s32 RampControl(stru_Ramp_t *pstruRamp)
 * ˵��     : ���º���
 * ���˼· : 1.���¼���
 * ����     ��
 * ����ֵ   ��pstruRamp->wRef
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
s32 RampControl(stru_Ramp_t *pstruRamp)
{
    if(pstruRamp->wRef < pstruRamp->wSet)
    {
        if((pstruRamp->wRef + pstruRamp->nACCStep) <= pstruRamp->wSet)  //����ֵС���趨ֵ������
        {
            pstruRamp->wRef += pstruRamp->nACCStep;
        }
        else
        {
            pstruRamp->wRef = pstruRamp->wSet;
        }
    }
    else if(pstruRamp->wRef > pstruRamp->wSet)                         //����ֵ�����趨ֵ���С
    {
        if((pstruRamp->wRef - pstruRamp->nDECStep) >= pstruRamp->wSet)
        {
            pstruRamp->wRef -= pstruRamp->nDECStep;
        }
        else
        {
            pstruRamp->wRef = pstruRamp->wSet;
        }
    }

    return(pstruRamp->wRef);
}

/*******************************************************************************
�������ƣ�    s16 PowerPIRegulator(stru_PIRegulator *Regulator)
����������    ���ʻ�PI  ����ʽPI
���������    stru_PIRegulator *Regulator �ṹ��ָ��
���������    PI���ڽ��
�� �� ֵ��    ��
����˵����
�޸�����      �汾��          �޸���            �޸�����
-----------------------------------------------------------------------------
2020/8/5      V1.0           WenCai Zhao          ����
*******************************************************************************/
s16 PowerPIRegulator(stru_PIRegulator *Reg)
{
    long    ACC;
    int AX;

    ACC = (long)(Reg->wInError - Reg->wLastError) * Reg->KP;   /* ��������� */
    ACC = (ACC << 0) + (long)(Reg->wInError) * Reg->KI;        /* ��������� */
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
�������ƣ�    PowerLoopReg(stru_PowerBasic_t *this)
����������    ���ʻ�����
���������    stru_PowerBasic_t *this  �ṹ��ָ��
���������    ��
�� �� ֵ��    ��
����˵����
�޸�����      �汾��          �޸���            �޸�����
-----------------------------------------------------------------------------
2020/8/5      V1.0           Howlet Li          ����
*******************************************************************************/
void PowerLoopReg(stru_PowerBasic_t *this)
{
    this->struPowerPI.wInError = this->wPowerRef - this->wPowerValue;

    /* ����޷����ɸ���ʵ�ʸ��ص�����Ӱ��PI��Ӧ�Ŀ��� */
    this->struPowerPI.wInError = sat(this->struPowerPI.wInError, -500, 500);

    struFOC_CurrLoop.nQCurrentSet = PowerPIRegulator(&this->struPowerPI);
}

/*****************************************************************************
 * ������   : void PowerLoopInit(void)
 * ˵��     : ���ʼ��������ʼ��
 * ���˼· : 1.������ʼ��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.10.09
 *****************************************************************************/
void PowerLoopInit(void)
{
    /*���ʻ�PI������ʼ��*/
    struPower.struPowerPI.KP = POWER_KP;                  //���ʻ� Kp
    struPower.struPowerPI.KI = POWER_KI;                  //���ʻ� Ki
    struPower.struPowerPI.wInError = 0;
    struPower.struPowerPI.wIntegral = 0;
    struPower.struPowerPI.wLastError = 0;
    struPower.struPowerPI.wUpperLimitOutput = (App2CoreCurTrans(User2AppCurTrans(POWER_IQMAX)) << 16); //���ʻ�������ֵ
    struPower.struPowerPI.wLowerLimitOutput = (App2CoreCurTrans(User2AppCurTrans(POWER_IQMIN)) << 16); //���ʻ������Сֵ

    /*���ʻ����������ʼ��*/
    struPower.nSpeedLimitCntr = 0;
    struPower.wPowerRef = 0;
    struPower.wPowerSet = 0;
    struPower.wSpeedLimitPowerRef = POWER_CALC(SPEED_LIMIT_POWER_VALUE);
    struPower.wSpeedLimitPowerSet = 0;
    struPower.wSpeedLimitValue = App2CoreFreqTrans(User2AppFreqTrans(SPEED_LIMIT_VALUE));

    /*���ʼ��������ʼ��*/
    struPower.wPowerValue = 0;
    struPower.wTemp = 0;

    struPower.struPowerLowPass.lTemp = 0;
    struPower.struPowerLowPass.nK1 = POWER_LOWPASS_SCALE;
    struPower.struPowerLowPass.wInput = 0;
    struPower.struPowerLowPass.wOutput = 0;

    /*�������²�����ʼ��*/
    struPower.struPowerRamp.nACCStep = POWER_CALC(POWER_RUN_ACC);
    struPower.struPowerRamp.nDECStep = POWER_CALC(POWER_RUN_DEC);
    struPower.struPowerRamp.wRef = 0;
    struPower.struPowerRamp.wSet = 0;
    
    struAppCommData.nPowerFistFlag = 0;
}

