/*******************************************************************************
 * ��Ȩ���� (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� PWMScan.c
 * �ļ���ʶ��
 * ����ժҪ�� PWMScan
 * ����˵���� PWM chenk , motor direction
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2019��12��27��
 *
 *******************************************************************************/
#include "PWMScan.h"
#include "lks32mc08x.h"
#include "lks32mc08x_tim.h"
#include "parameter.h"
#include "state_machine.h"
#include "MC_type.h"
#include "power.h"
#include "fault_detection.h"

stru_PWMTime_t  PWMTime;

extern u16 g_nMotorRunDir;
/*****************************************************************************
 * ������   : PWMScan(void)
 * ˵��     : �������PWM��ռ�ձȺ����ڣ���������ռ�ձȵ��ٺ�Ƶ�ʵ���
 * ���˼· ��1.��Timer CH0��CH1���ӵ�һ��CH0������׽�ߵ�ƽ������CH1������׽���ڼ���; \
 *          : 2.��Բ���PWM���������Χ��·Ҫ��������������
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void PWMScan(void)
{
  if(UTIMER_IF & BIT5)//T1_CH1_IF Timer1 CH1�½����ж���1
  {
    PWMTime.nFlagPeriod = 1;
    PWMTime.nHighLevelTime = (s16)UTIMER_UNT1_CMP1 - (INT16)UTIMER_UNT1_CMP0; //��׽�ߵ�ƽ����ֵ
    PWMTime.nPeriod = (s16)UTIMER_UNT1_CMP1 - PWMTime.nPreTime;                //��׽���ڼ���ֵ
    PWMTime.nPreTime = (s16)UTIMER_UNT1_CMP1;

    PWMTime.nFlagReset = 0;
  }

  if(UTIMER_IF & BIT3)                     //Timer1�����ж���1
  {
    if(!PWMTime.nFlagPeriod)              //�����ⲻ��CH1�½��ر仯����ΪPWM���ӵ����
    {
      if(PWM_HIGH)                     //����IO�ڵ�ƽΪ�ߣ�ռ�ձ�Ϊ100%
      {
        PWMTime.nDuty  = DUTY_FULL;//100%
        PWMTime.nPeriod = PWM_TIME_PERIOD; //10k

        PWMTime.nFlagReset = 0;
      }
      else                             //����IO�ڵ�ƽΪ�ͣ�ռ�ձ�Ϊ0%
      {
        PWMTime.nDuty  = DUTY_ZERO;//0%
        PWMTime.nPeriod = PWM_TIME_PERIOD;
//                PWMTime.FlagReset = 1;
      }
    }

    PWMTime.nFlagPeriod = 0;

  }
}

/*****************************************************************************
 * ������   : PWMnDutyScan(void)
 * ˵��     : PWMռ�ձȼ���
 * ���˼· ��1.���ݸߵ�ƽ�����ڵļ���������ռ�ձȣ�����DSP���㣬���̼���ʱ��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void PWMnDutyScan(void)
{
  if(PWMTime.nFlagPeriod)
  {
    __disable_irq();/*�ر��жϣ��ж��ܿ���*/

    DSP_DID = ((INT32)PWMTime.nHighLevelTime * 1000 + ((INT32)PWMTime.nPeriod >> 1));  //����DSP����ռ�ձ�
    DSP_DIS =  (INT32)PWMTime.nPeriod;
    delay(15);
    PWMTime.nDuty = DSP_QUO;

    __enable_irq();/*�����ж�*/
  }

  /***********оƬ��λ����*******************/
//    if(PWMTime.FlagReset)
//    {
//        PWMTime.ResetCntr ++;
//        if(PWMTime.ResetCntr > RESET_TIME)
//        {
//            PWMOutputs(DISABLE);
//            __disable_irq();/*�ر��жϣ��ж��ܿ���*/
//            FG_LOW;
//            PWMTime.ResetCntr = 0;
//            REG32(0xE000ED0C) = 0x05FA0004; //MCU ��λ
//        }
//    }
//    else
//    {
//        PWMTime.ResetCntr = 0;
//    }


}

/*****************************************************************************
 * ������   : PWMScanInit(void)
 * ˵��     : PWMռ�ձȼ���ʼ��
 * ���˼· ��1.������ʼ��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void PWMScanInit(void)
{
  PWMTime.nDuty = 0;
  PWMTime.nFlagPeriod = 0;
  PWMTime.nHighLevelTime = 0;
  PWMTime.nLowLevelTime = 0;
  PWMTime.nPeriod = 0;

  PWMTime.nFlagReset = 0;
  PWMTime.nResetCntr = 0;
}

/*****************************************************************************
 * ������   : SpeedScan(void)
 * ˵��     : �����ٶȼ���
 * ���˼· ��1.���ݼ�⵽��ռ�ձȼ����Ӧת��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void SpeedScan(void)
{
  if(PWMTime.nDuty < PWM_MIN)
  {
    AppCommData.wSpeedRef = SPEED_MIN;
  }
  else if(PWMTime.nDuty < PWM_MAX)
  {
    AppCommData.wSpeedRef = (SPEED_SCALE * (PWMTime.nDuty - PWM_MIN)) + SPEED_MIN;
  }
  else
  {
    AppCommData.wSpeedRef = SPEED_MAX;
  }
}

/*****************************************************************************
 * ������   : MotorDirScan(void)
 * ˵��     : ������з���ת��
 * ���˼· ��1.���ݷ�����IO�ĵ�ƽ��ȷ�����ת��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void MotorDirScan(void)
{
  static s16 s16DirectionCnt;

  if(DIR_CW)                              //IO��ƽΪ��ʱΪCW
  {
    s16DirectionCnt ++;
    if(s16DirectionCnt >= DIR_FLITER)   //����
    {
      s16DirectionCnt = DIR_FLITER;
      g_nMotorRunDir = CW;
    }
  }
  else                                     //IO��ƽΪ��ʱΪCCW
  {
    s16DirectionCnt --;
    if(s16DirectionCnt <= -DIR_FLITER)
    {
      s16DirectionCnt = -DIR_FLITER;
      g_nMotorRunDir = CCW;
    }
  }

  if(AppCommData.nMotorRunDirPre != g_nMotorRunDir)   //��⵽��ת��͵�ǰת��һ�£����л���ilde״̬����
  {
    ControlState = idle;
  }

  AppCommData.nMotorRunDirPre = g_nMotorRunDir;
}

/*****************************************************************************
 * Function:     void FGScan(void)
 * Description:  FG  ��еһȦ6�����壬2�Լ� == һ��������3������
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
/*****************************************************************************
 * ������   : FGScan(void)
 * ˵��     : ���ת������ź�
 * ���˼· ��1.��еһȦ6�����壬2�Լ� == һ��������3�����壬��Ҫ����ʵ��������     \
 *          : 2.����FGƵ�ʺ͵����Ƶ������������ϵ�Ĳ������еķ�������ͨ���Ƕ�����תIO; \
 *          : 3.��������Ƶ�ʵ�FG���������Timer����ͨ��ת��������Timer������ֵ��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void FGScan(u16 nTheta)
{
  if((nTheta > 0) && (nTheta <= 10922))          //0<Theta<60
  {
    FG_HIGH;
  }
  else if((nTheta > 10922) && (nTheta <= 21844)) //60<Theta<120
  {
    FG_LOW;
  }
  else if((nTheta > 21844) && (nTheta <= 32766)) //120<Theta<180
  {
    FG_HIGH;
  }
  else if((nTheta > 32766) && (nTheta <= 43688)) //180<Theta<240
  {
    FG_LOW;
  }
  else if((nTheta > 43688) && (nTheta <= 54610)) //240<Theta<300
  {
    FG_HIGH;
  }
  else if((nTheta > 54610) && (nTheta <= 65535)) //300<Theta<360
  {
    FG_LOW;
  }

}

/*****************************************************************************
 * ������   : PowerScan(void)
 * ˵��     : �������ʼ���
 * ���˼· ��1.���ݼ�⵽�ĵ�λ����ѹ�����Ӧת��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void PowerScan(void)
{
  if(PWMTime.nVSPValue < VSP_MIN)
  {
    if(Faults.R == 0)
    {
      g_bMC_RunFlg = 0;
    }
  }
  else if(PWMTime.nVSPValue < VSP_MAX)
  {
    PWMTime.wPowerValue = (POWER_SCALE * (PWMTime.nVSPValue - VSP_MIN)) + POWER_MIN;
    if((g_bMC_RunFlg == 0) && (Faults.R == 0))
    {
      g_bMC_RunFlg = 1;
    }
  }
  else
  {
    PWMTime.wPowerValue = POWER_MAX;
    if((g_bMC_RunFlg == 0) && (Faults.R == 0))
    {
      g_bMC_RunFlg = 1;
    }
  }

}

