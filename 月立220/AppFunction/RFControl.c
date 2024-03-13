/*******************************************************************************
 * ��Ȩ���� (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� RFControl.c
 * �ļ���ʶ��
 * ����ժҪ�� RFControl
 * ����˵����
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2020��2��21��
 *
 *******************************************************************************/
#include "RFControl.h"
#include "lks32mc08x_flash.h"
#include "lks32mc08x_tim.h"
#include "mc_declaration.h"
#include "parameter.h"
#include "fault_detection.h"
#include "lks32mc08x_tim.h"

RFSystem_t RFState;
RFProcess_t RFHandle;

/*****************************************************************************
 * Function:     void RFReceive(void)
 * Description:  RFReceive
 * Parameter:    data receive  period 62.5us
 * Return:       no
 *****************************************************************************/
//INT16  DataTestCntr;
//UINT32 DataTest[6];
void RFReceive(void)
{
  if(RFLEVEL_HIGH)//level 1
  {
    if(RFState.PeriodFlag)
    {
      RFState.PeriodFlag = 0;
      RFState.Period = RFState.HighLevelCntr + RFState.LowLevelCntr;
      if((RFState.Period > CODE_PERIOD_TIME_MIN)&&(RFState.Period < CODE_PERIOD_TIME_MAX))
      {
        if((RFState.HighLevelCntr > CODE0_HIGH_TIME_MIN)&&(RFState.HighLevelCntr < CODE0_HIGH_TIME_MAX))//data 0
        {
          if(RFState.DataCntr < DATA_LENGTH)
          {
            RFState.DataCntr++;
          }
          else
          {
//              DataTest[DataTestCntr] = RFState.DataBuffer.R;
//              DataTestCntr++;
//              if(DataTestCntr > 6)
//              {
//                DataTestCntr = 0;
//              }

            if(RFState.DataBufferTime == 0)
            {
              RFState.DataBufferCheck.R = RFState.DataBuffer.R;
              RFState.DataBufferTime = 1;
            }
            else  //�ڶ�������
            {
              if(RFState.DataBufferCheck.R == RFState.DataBuffer.R)
              {
                RFState.DataBufferTime = 0;
//                RFState.RXSuccessFlag  = 1;
                RFState.DataBufferCheck.R = 0;
//                RFState.DataBufferCntr = 0;
                if(RFState.Data.R != RFState.DataBuffer.R)
                {
                  RFState.Data.R = RFState.DataBuffer.R;
                  if(RFState.DataDelayCntr == 0)
                  {
                    RFState.RXSuccessFlag  = 1;
                    RFState.DataDelayCntr = DATA_DELAY_TIME;
                  }
//                  RFState.RXSuccessFlag  = 1;
                }
              }
              else
              {
                RFState.DataBufferCheck.R = RFState.DataBuffer.R;
              }
            }

            RFState.DataCntr = 0;
            RFState.DataBuffer.R = 0;
          }
        }
        else if((RFState.HighLevelCntr > CODE1_HIGH_TIME_MIN)&&(RFState.HighLevelCntr < CODE1_HIGH_TIME_MAX))//data 1
        {
          if(RFState.DataCntr < DATA_LENGTH)
          {
            RFState.DataBuffer.R += (1 << (DATA_LENGTH-RFState.DataCntr));//MSB
//            RFState.DataBuffer.R += (1 << RFState.DataCntr);  //LSB
            RFState.DataCntr++;
          }
          else
          {
            RFState.DataBuffer.R += 1;//MSB
//            RFState.DataBuffer.R += (1<< RFState.DataCntr); //LSB

//              DataTest[DataTestCntr] = RFState.DataBuffer.R;
//              DataTestCntr++;
//              if(DataTestCntr > 6)
//              {
//                DataTestCntr = 0;
//              }
//
            if(RFState.DataBufferTime == 0) //��һ������
            {
              RFState.DataBufferCheck.R = RFState.DataBuffer.R;
              RFState.DataBufferTime = 1;
            }
            else     ////��ȡ���� & У�� 2��������ͬ
            {
              if(RFState.DataBufferCheck.R == RFState.DataBuffer.R)
              {
                RFState.DataBufferTime = 0;
//                RFState.RXSuccessFlag  = 1;
                RFState.DataBufferCheck.R = 0;
//                RFState.DataBufferCntr = 0;
                if(RFState.Data.R != RFState.DataBuffer.R)
                {
                  RFState.Data.R = RFState.DataBuffer.R;

                  if(RFState.DataDelayCntr == 0)
                  {
                    RFState.RXSuccessFlag = 1;
                    RFState.DataDelayCntr = DATA_DELAY_TIME;
                  }
//                  RFState.RXSuccessFlag  = 1;
                }
              }
              else
              {
                RFState.DataBufferCheck.R = RFState.DataBuffer.R;
              }

            }

            RFState.DataCntr = 0;
            RFState.DataBuffer.R = 0;
          }
        }
        else //noise
        {
          RFState.DataCntr = 0;
          RFState.DataBuffer.R = 0;
        }
      }
      else //noise
      {
        RFState.DataCntr = 0;
        RFState.DataBuffer.R = 0;
      }

      RFState.HighLevelCntr = 0;
      RFState.LowLevelCntr  = 0;
    }

    if(RFState.HighLevelCntr < CODE1_HIGH_TIME_MAX)
    {
      RFState.HighLevelCntr ++;
    }
    else //noise
    {
      RFState.DataCntr = 0;
      RFState.DataBuffer.R = 0;
//      RFState.DataBufferCheck.R = 0;
    }

  }
  else //level 0
  {

    if(RFState.LowLevelCntr < CODE_LOW_TIME_MAX)
    {
      RFState.LowLevelCntr++;
      RFState.PeriodFlag = 1;
    }
    else  //noise
    {
      RFState.DataCntr = 0;
      RFState.DataBuffer.R = 0;
      RFState.HighLevelCntr = 0;
    }

  }

  if(RFState.DataBufferTime) //300ms ���ܽ��յ�2֡��ȷ���������½���
  {
    if(RFState.DataBufferCntr < DATA_CHECK_TIME)
    {
      RFState.DataBufferCntr ++;
    }
    else
    {
      RFState.DataBufferCntr = 0;
      RFState.DataBufferTime = 0;
      RFState.DataBufferCheck.R = 0;
    }
  }
  else
  {
    RFState.DataBufferCntr = 0;
  }

}


/*****************************************************************************
 * Function:     void RFCommand(void)
 * Description:  RF Command
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void RFCommand(void)
{
  if(RFState.RXSuccessFlag)
  {
    RFState.RXSuccessFlag = 0;
    RFState.DataBufferCntr = 0;
//    RFState.DataDelayCntr = DATA_DELAY_TIME;
    //У��
    RFState.CheckData = (RFState.Data.B.FunctionCode0<<3) + RFState.Data.B.PID;
    RFState.CheckData = RFState.CheckData ^ RFState.Data.B.CustomerCode0 ^ RFState.Data.B.CustomerCode1 ^ RFState.Data.B.CustomerCode2 ^
                        RFState.Data.B.CustomerCode3 ^ RFState.Data.B.CustomerCode4 ^ RFState.Data.B.FunctionCode1 ^ 0xA;
    if(RFState.CheckData == RFState.Data.B.Check) //У����ȷ
    {
      RFState.KeyCode = ((UINT16)RFState.Data.B.FunctionCode1 << 1) + (UINT16)RFState.Data.B.FunctionCode0;//ȡ������

      RFState.CustomerIDBuffer = (RFState.Data.R & 0xFFFFF000); //��ȡID

      if(RFState.IDMatchCntr < ID_MATCH_TIME) //5s�ڶ���
      {
        if((RFState.KeyCode == ID_MATCH)&&(RFState.IDMatchFlag == 0)) //����
        {
          if((RFState.CustomerIDBuffer != RFState.CustomerID[0])&&(RFState.CustomerIDBuffer != RFState.CustomerID[1]))//�洢ID
          {
            RFState.CustomerID[0] = RFState.CustomerID[1];
            RFState.CustomerID[1] = RFState.CustomerIDBuffer;

            __disable_irq();

            EraseSector(ID_ADDRESS);
            ProgramPage(ID_ADDRESS,8,(unsigned char*)RFState.CustomerID);

            __enable_irq();
            RFState.IDMatchFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
        }
      }

      if((RFState.CustomerIDBuffer == RFState.CustomerID[0])||(RFState.CustomerIDBuffer == RFState.CustomerID[1])) //ID match
      {
//        RFState.BZTime = BZ_ON_TIME;

        switch(RFState.KeyCode)
        {
        case IR_LED: //P0.11    �ƿ�
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            if(RFHandle.LedFlag == 0)
            {
              UTIMER_UNT3_CMP0 = 48000;
              RFHandle.LedFlag = 1;
            }
            else if(RFHandle.LedFlag == 1)
            {
              UTIMER_UNT3_CMP0 = 96001;
              RFHandle.LedFlag = 0;
            }

            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_ON_OFF:         //����
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.OnOffFlag = FAN_OFF;
            TIM_TimerCmd(TIMER3, DISABLE);//�ص�
            RFInit();
            g_bMC_RunFlg = 0;
            ControlState = idle;
          }
          else if(RFHandle.OnOffFlag == FAN_OFF)
          {
            if(Faults.R == 0)
            {
              RFHandle.OnOffFlag = FAN_ON;
            }
//              g_bMC_RunFlg = 1;
          }

          RFState.BZTime = BZ_ON_TIME;

          break;
        }
        case IR_SPEED1:   //1��
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;
            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED1);

            RFState.BZTime = BZ_ON_TIME;
            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_SPEED2:    //2��
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;

            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED2);

            RFState.BZTime = BZ_ON_TIME;

            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_SPEED3:   //3��
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;

            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED3);

            RFState.BZTime = BZ_ON_TIME;

            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_SPEED4:     //4��
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;

            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED4);

            RFState.BZTime = BZ_ON_TIME;

            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_SPEED5:     //5��
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            g_bMC_RunFlg = 1;

            AppCommData.wSpeedRef = User2AppFreqTrans(SPEED5);

            RFState.BZTime = BZ_ON_TIME;

            RFHandle.NatureWindFlag = 0;
          }
          break;
        }
        case IR_NATUREWIND:   //��Ȼ��
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {

            RFHandle.NatureWindFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_AUTOPOWER_1H:       ////��ʱ1h
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.PowerOnCntr = AUTOPOWER_1H;
            RFHandle.PowerDelayFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_AUTOPOWER_2H: ////��ʱ2h
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.PowerOnCntr = AUTOPOWER_2H;
            RFHandle.PowerDelayFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_AUTOPOWER_3H://��ʱ3h
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.PowerOnCntr = AUTOPOWER_3H;
            RFHandle.PowerDelayFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_AUTOPOWER_8H:    //��ʱ8h
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            RFHandle.PowerOnCntr = AUTOPOWER_8H;
            RFHandle.PowerDelayFlag = 1;
            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case IR_DIR: //�����л�
        {
          if(RFHandle.OnOffFlag == FAN_ON)
          {
            if(RFHandle.FanDirection == CW)
            {
              RFHandle.FanDirection = CCW;
              g_nMotorRunDir = RFHandle.FanDirection;
            }
            else if(RFHandle.FanDirection == CCW)
            {
              RFHandle.FanDirection = CW;
              g_nMotorRunDir = RFHandle.FanDirection;
            }

            RFHandle.DirectionCntr = DIR_DELAY_TIME;
            RFHandle.DirectionFlag = 1;
            ControlState = idle;//����idle״̬����ʱDIR_DELAY_TIME ����
            g_bMC_RunFlg = 0;

            RFState.BZTime = BZ_ON_TIME;
          }
          break;
        }
        case ID_MATCH:
        {
//            if(RFState.IDMatchFlag == 0)
//            {
//              RFState.BZTime = BZ_ON_TIME;
//            }
          break;
        }
        default:
          break;
        }
      }
    }
  }

  if(RFState.IDMatchCntr < ID_MATCH_TIME)//5s�ڲ��ܶ��룬�����ϵ硣
  {
    RFState.IDMatchCntr ++;
  }


  if(RFState.BZTime > 0) //������ʱ��
  {
    RFState.BZTime --;
  }

  if(RFHandle.DirectionFlag)
  {
    if(RFHandle.DirectionCntr > 0) //�����л�ʱ��
    {
      RFHandle.DirectionCntr--;
    }
    else
    {
      if(AppCommData.wSpeedRef != 0)
      {
        g_bMC_RunFlg = 1;
      }
      RFHandle.DirectionFlag = 0;
    }
  }

  if(RFState.DataDelayCntr > 0)//���յ���ȷ���ݺ����ʱ
  {
    RFState.DataDelayCntr--;
  }

  NatureWindControl(); //��Ȼ��

  FanRunTime();//��ʱ

}

/*****************************************************************************
 * Function:     void NatureWindContor(void)
 * Description:  Nature Wind 1 3 5 3 5 4����20s
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void NatureWindControl(void)
{
  if(RFHandle.NatureWindFlag)
  {
    if(RFHandle.NatureWindCntr < NATUREWIND_TIME)
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED1);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*2))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED3);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*3))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED5);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*4))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED2);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*5))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED5);

    }
    else if(RFHandle.NatureWindCntr < (NATUREWIND_TIME*6))
    {
      AppCommData.wSpeedRef = User2AppFreqTrans(SPEED4);

    }
    else
    {
      RFHandle.NatureWindCntr = 0;
    }

    RFHandle.NatureWindCntr ++;
  }
  else
  {
    RFHandle.NatureWindCntr = 0;
  }
}

/*****************************************************************************
 * Function:     void FanRunTime(void)
 * Description:  �������ж�ʱ  1H 2H 3H 8H
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void FanRunTime(void)
{
  if(RFHandle.PowerDelayFlag)
  {
    if(RFHandle.PowerOnCntr > 0)
    {
      RFHandle.PowerOnCntr--;
    }
    else
    {
      RFHandle.OnOffFlag = FAN_OFF;
      RFHandle.PowerDelayFlag = 0;
      RFInit();
      g_bMC_RunFlg = 0;
    }
  }
}
/*****************************************************************************
 * Function:     void BZContor(void)
 * Description:   Contor BZ  3k hz
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
UINT8 BZCntr = 0;
void BZControl(void)
{
  if(RFState.BZTime > 0)
  {
    if(BZCntr < 2)
    {
      BZ_ON;
    }
    else if(BZCntr < 4)
    {
      BZ_OFF;
    }
    else
    {
      BZCntr = 0;
    }

    BZCntr++;
  }
  else
  {
    BZ_ON;
    BZCntr = 0;
  }
}

/*****************************************************************************
 * Function:     void RFInit(void)
 * Description:  RF  Init
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void RFInit(void)
{
  RFState.CheckData = 0;
  RFState.Data.R = 0;
  RFState.DataBuffer.R = 0;
  RFState.DataBufferCheck.R = 0;
  RFState.DataBufferCntr = 0;
  RFState.DataBufferTime = 0;
  RFState.DataCntr = 0;
  RFState.PeriodFlag = 0;
  RFState.RXSuccessFlag = 0;
  RFState.HighLevelCntr = 0;
  RFState.LowLevelCntr = 0;
  RFState.Period = 0;
  RFState.KeyCode = 0;

//RF Handle
  RFHandle.PowerOnCntr = 0;
  RFHandle.NatureWindCntr = 0;
  RFHandle.NatureWindFlag = 0;
  RFHandle.PowerDelayFlag = 0;
  RFHandle.DirectionCntr = 0;
  RFHandle.LedFlag = 0;
}
