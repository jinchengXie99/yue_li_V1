/*******************************************************************************
 * ��Ȩ���� (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� SpeedScan.c
 * �ļ���ʶ��
 * ����ժҪ�� ��λ�����٣�PWM���ٴ������
 * ����˵���� 
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2019��12��27��
 *
 *******************************************************************************/
#include "SpeedScan.h"
#include "lks32mc03x.h"
#include "lks32mc03x_timer.h"
#include "state_machine.h"
#include "MC_type.h"
#include "fault_detection.h"
#include "Global_Variable.h"
#include "PowerCalculation.h"

stru_PWMTime_t  struPWMTime;

/*****************************************************************************
 * ������   : PWMScan(void)
 * ˵��     : �������PWM��ռ�ձȺ����ڣ���������ռ�ձȵ��ٺ�Ƶ�ʵ���
 * ���˼· ��1.��Timer0 ���PWMռ�ձȣ�CH1������׽�ߵ�ƽ������CH0������׽���ڼ���; \
 *          : 2.��Բ���PWM���������Χ��·Ҫ��������������
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void PWMScan(void)
{
    if(UTIMER0_IF & BIT0) 
    {
        UTIMER0_IF = BIT0;
        
        struPWMTime.nFlagPeriod = 1;
        struPWMTime.wHighLevelTime = UTIMER0_CMP1; //��׽�ߵ�ƽ����ֵ
        struPWMTime.wPeriod = UTIMER0_CMP0;           //��׽���ڼ���ֵ

        struPWMTime.nHighCntr = 0;
        struPWMTime.nLowCntr = 0;
    }

    if(UTIMER0_IF & BIT2)             
    {
        UTIMER0_IF = BIT2;
        
        if(struPWMTime.nFlagPeriod == 0)     //�����ⲻ��CH1�½��ر仯����ΪPWM���ӵ����
        {
            if(PWM_HIGH)                          //����IO�ڵ�ƽΪ�ߣ�ռ�ձ�Ϊ100%
            {

                struPWMTime.nHighCntr ++;
                struPWMTime.nLowCntr = 0;
            }
            else                             //����IO�ڵ�ƽΪ�ͣ�ռ�ձ�Ϊ0%
            {
                struPWMTime.nLowCntr ++;
                 struPWMTime.nHighCntr = 0;
            }
        } 
        else
        {                                          
            struPWMTime.nHighCntr = 0;
            struPWMTime.nLowCntr = 0;
        }

        struPWMTime.nFlagPeriod = 0;
    }
}

/*****************************************************************************
 * ������   : PWMDutyScan(void)
 * ˵��     : PWMռ�ձȼ���
 * ���˼· ��1.���ݸߵ�ƽ�����ڵļ���������ռ�ձȣ�����DSP���㣬���̼���ʱ��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void PWMDutyScan(void)
{
    if((struPWMTime.wPeriod > 0)&&(struPWMTime.wHighLevelTime > 0))
    {
        struPWMTime.nDuty = (struPWMTime.wHighLevelTime * 1000 + ((s32)struPWMTime.wPeriod * 5))/struPWMTime.wPeriod;

        if((struPWMTime.nHighCntr > 5)||(struPWMTime.nLowCntr > 5))
        {
            if(struPWMTime.nHighCntr > struPWMTime.nLowCntr)
            {
                struPWMTime.nDuty  = DUTY_FULL;//100%
                struPWMTime.wPeriod = PWM_TIME_PERIOD; 
                
                struPWMTime.nHighCntr = 0;
                struPWMTime.nLowCntr = 0;
                struPWMTime.wHighLevelTime = 0;
            }
            else
            {
                struPWMTime.nDuty  = DUTY_ZERO;//0%
                struPWMTime.wPeriod = PWM_TIME_PERIOD;
                
                struPWMTime.nHighCntr = 0;
                struPWMTime.nLowCntr = 0;
                struPWMTime.wHighLevelTime = 0;
            }
        }
    }
    else
    {
        if((struPWMTime.nHighCntr > 5)||(struPWMTime.nLowCntr > 5))
        {
            if(struPWMTime.nHighCntr > struPWMTime.nLowCntr)
            {
                struPWMTime.nDuty  = DUTY_FULL;//100%
                struPWMTime.wPeriod = PWM_TIME_PERIOD; 
                
                struPWMTime.nHighCntr = 0;
                struPWMTime.nLowCntr = 0;
                
                struPWMTime.wHighLevelTime = 0;
            }
            else
            {
                struPWMTime.nDuty  = DUTY_ZERO;//0%
                struPWMTime.wPeriod = PWM_TIME_PERIOD;
                
                struPWMTime.nHighCntr = 0;
                struPWMTime.nLowCntr = 0;
                
                struPWMTime.wHighLevelTime = 0;
            }
        }
    }
}

//void PWMDutyScan(void)
//{
//    if(struPWMTime.nFlagPeriod == 1)
//    {
//        struPWMTime.nDuty = (struPWMTime.wHighLevelTime * 1000 + ((s32)struPWMTime.wPeriod * 5))/struPWMTime.wPeriod;
//    }
//    else
//    {
//        if((struPWMTime.nHighCntr > 5)||(struPWMTime.nLowCntr > 5))
//        {
//            if(struPWMTime.nHighCntr > struPWMTime.nLowCntr)
//            {
//                struPWMTime.nDuty  = DUTY_FULL;//100%
//                struPWMTime.wPeriod = PWM_TIME_PERIOD; 
//                
//                struPWMTime.nHighCntr = 0;
//                struPWMTime.nLowCntr = 0;
//                
//                struPWMTime.wHighLevelTime = 0;
//            }
//            else
//            {
//                struPWMTime.nDuty  = DUTY_ZERO;//0%
//                struPWMTime.wPeriod = PWM_TIME_PERIOD;
//                
//                struPWMTime.nHighCntr = 0;
//                struPWMTime.nLowCntr = 0;
//                
//                struPWMTime.wHighLevelTime = 0;
//            }
//        }
//    }
//}

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
    struPWMTime.nDuty = 0;
    struPWMTime.nFlagPeriod = 0;
    struPWMTime.wHighLevelTime = 0;
    struPWMTime.wLowLevelTime = 0;
    struPWMTime.wPeriod = 0;
    struPWMTime.nHighCntr = 0;
    struPWMTime.nLowCntr = 0;
}

/*****************************************************************************
 * ������   : PWMSpeedScan(void)
 * ˵��     : 
 * ���˼· ��1.���ݼ�⵽��ռ�ձȼ����Ӧת��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
//void PWMSpeedScan(void)
//{
//    if(struPWMTime.nDuty < PWM_OFF)
//    {
//        struFOC_CtrProc.bMC_RunFlg = 0;   //�ر�PWM
//        struAppCommData.wPWMPowerSet = PWM_POWER_OFF;
//    }
//    else if(struPWMTime.nDuty < PWM_MIN)
//    {
//        struAppCommData.wPWMPowerSet = PWM_POWER_MIN;
//    }
//    else if(struPWMTime.nDuty < PWM_MAX)
//    {
//        struAppCommData.wPWMPowerSet = (PWM_POWER_SCALE * (struPWMTime.nDuty - PWM_MIN)) + PWM_POWER_MIN;
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else
//    {
//        struAppCommData.wPWMPowerSet = PWM_POWER_MAX;
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//}

void PWMSpeedScan(void) //���ʻ�
{
    if(struPWMTime.nDuty < PWM_OFF)
    {
        struFOC_CtrProc.bMC_RunFlg = 0;   //�ر�PWM
        struAppCommData.wPWMPowerSet = PWM_POWER_OFF;
    }
    else if(struPWMTime.nDuty < PWM_MIN)
    {
        struAppCommData.wPWMPowerSet = PWM_POWER_MIN;

    }
    else if(struPWMTime.nDuty < PWM_MAX)
    {
        struAppCommData.wPWMPowerSet = (PWM_POWER_SCALE * (struPWMTime.nDuty - PWM_MIN)) + PWM_POWER_MIN;
        
        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
        {
            struFOC_CtrProc.bMC_RunFlg = 1;
        }
    }
    else
    {
        struAppCommData.wPWMPowerSet = PWM_POWER_MAX;
        
        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
        {
            struFOC_CtrProc.bMC_RunFlg = 1;
        }
    }
}




//void PWMSpeedScan(void)
//{
//    if(struPWMTime.nDuty < PWM_OFF)
//    {
//        struFOC_CtrProc.bMC_RunFlg = 0;   //�ر�PWM
//    }
//    else if(struPWMTime.nDuty < PWM_MIN)
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_FIRST); 
//        
//    }
//    else if(struPWMTime.nDuty <= PWM_FIRST)    /* 1�� */
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_FIRST);          

//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else if(struPWMTime.nDuty < PWM_SECOND)    /* 2�� */
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_SECOND);        
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else if(struPWMTime.nDuty < PWM_THREE)    /* 3�� */
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_THREE);      
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else
//    {
//        struAppCommData.wPWMPowerSet = POWER_CALC(PWM_POWER_FOUR);
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//}

/*****************************************************************************
 * ������   : VspSpeedScan(void)
 * ˵��     : 
 * ���˼· ��1.���ݼ�⵽��ѹ�����Ӧת��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void VspSpeedScan(void)
{
    if(struAppCommData.nVspADC < VSP_MIN)
    {
        struAppCommData.wVSPSpeedSet = VSP_SPEED_MIN;
        
        struFOC_CtrProc.bMC_RunFlg = 0;
    }
    else if(struAppCommData.nVspADC  < VSP_MAX)
    {
        struAppCommData.wVSPSpeedSet = (VSP_SPEED_SCALE * (struAppCommData.nVspADC - VSP_MIN)) + VSP_SPEED_MIN;
        
        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
        {
            struFOC_CtrProc.bMC_RunFlg = 1;
        }
        
    }
    else
    {
        struAppCommData.wVSPSpeedSet = VSP_SPEED_MAX;

        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
        {
            struFOC_CtrProc.bMC_RunFlg = 1;
        }
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
//void MotorDirScan(void)
//{
//    static s16 s16DirectionCnt;

//    if(DIR_CW)                              //IO��ƽΪ��ʱΪCW
//    {
//        s16DirectionCnt ++;
//        if(s16DirectionCnt >= DIR_FLITER)   //����
//        {
//            s16DirectionCnt = DIR_FLITER;
//            struFOC_CtrProc.bMotorDirtionCtrl = CW;
//        }
//    }
//    else                                     //IO��ƽΪ��ʱΪCCW
//    {
//        s16DirectionCnt --;
//        if(s16DirectionCnt <= -DIR_FLITER)
//        {
//            s16DirectionCnt = -DIR_FLITER;
//            struFOC_CtrProc.bMotorDirtionCtrl = CCW;
//        }
//    }

//    if(struAppCommData.nMotorRunDirPre != struFOC_CtrProc.bMotorDirtionCtrl)   //��⵽��ת��͵�ǰת��һ�£����л���ilde״̬����
//    {
//        struFOC_CtrProc.eSysState = IDLE;
//    }

//    struAppCommData.nMotorRunDirPre = struFOC_CtrProc.bMotorDirtionCtrl;
//}

/*****************************************************************************
 * ������   : void FGScan(u16 nTheta)
 * ˵��     : ���ת������ź�
 * ���˼· ��1.��еһȦ6�����壬6�Լ� == һ��������1�����壬��Ҫ����ʵ��������     \
 *          : 2.����FGƵ�ʺ͵����Ƶ������������ϵ�Ĳ������еķ�������ͨ���Ƕ�����תIO; \
 *          : 3.��������Ƶ�ʵ�FG���������Timer����ͨ��ת��������Timer������ֵ��
 * ����     ����
 * ����ֵ   ����
 * �޸�ʱ�� ��2020.08.17
 *****************************************************************************/
void FGScan(u16 nTheta)
{
    if((nTheta > 0) && (nTheta <= 32766))          //0<Theta<180
    {
        FG_HIGH;
    }        
    else if((nTheta > 32766) && (nTheta <= 65535)) //190<Theta<360
    {
        FG_LOW;
    }
}

///*****************************************************************************
// * ������   : PowerScan(void)
// * ˵��     : �������ʼ���
// * ���˼· ��1.���ݰ����������ʣ�1��60W��2��120W
// * ����     ����
// * ����ֵ   ����
// * �޸�ʱ�� ��2021.04.21
// *****************************************************************************/
//void PowerScan(void)
//{
//    
//    if(struPWMTime.nVSPValue < VSP_MIN)
//    {
//        if(stru_Faults.R == 0)
//        {
//            struFOC_CtrProc.bMC_RunFlg = 0;
//        }
//    }
//    else if(struPWMTime.nVSPValue < VSP_MAX)
//    {
//        struPWMTime.wPowerValue = (POWER_SCALE * (struPWMTime.nVSPValue - VSP_MIN)) + POWER_MIN;
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }
//    else
//    {
//        struPWMTime.wPowerValue = POWER_MAX;
//        
//        if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//        {
//            struFOC_CtrProc.bMC_RunFlg = 1;
//        }
//    }

//}


