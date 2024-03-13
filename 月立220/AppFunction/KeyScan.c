/*******************************************************************************
 * ��Ȩ���� (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� KeyScan.c
 * �ļ���ʶ��
 * ����ժҪ�� KeyScan
 * ����˵���� ��
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2019��12��19��
 *
 *******************************************************************************/
#include "KeyScan.h"

KeyControl_t key_state;

/*****************************************************************************
 * Function:     void KeyScan(void)
 * Description:  key value  stop or run
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void KeyScan(KeyControl_t *pKeyState)
{
  if(KEY0)
  {
    pKeyState->nKey0PressCnt ++;
    if(pKeyState->nKey0PressCnt >= KEY_FLITER)
    {
      pKeyState->nKey0PressCnt = KEY_FLITER;
      if(pKeyState->nFlagKey0Press == 1)
      {
        pKeyState->nFlagKey0Press = 0;
        if(pKeyState->nKey0Value == START)
        {
          pKeyState->nKey0Value = STOP;
        }
        else
        {
          pKeyState->nKey0Value = START;
        }
      }
    }
  }
  else
  {
    pKeyState->nKey0PressCnt --;
    if(pKeyState->nKey0PressCnt <= -KEY_FLITER)
    {
      pKeyState->nKey0PressCnt = -KEY_FLITER;
      pKeyState->nFlagKey0Press = 1;
    }
  }
}

/*****************************************************************************
 * Function:     void KeyInit(void)
 * Description:  init
 * Parameter:    no
 * Return:       no
 *****************************************************************************/
void KeyInit(void)
{
  key_state.nFlagKey0Press = 0;
  key_state.nKey0PressCnt  = 0;
  key_state.nKey0Value     = STOP;
  LED_OFF;
}
