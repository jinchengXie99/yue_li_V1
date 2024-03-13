/*******************************************************************************
 * ��Ȩ���� (C)2019, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� KeyScan.h
 * �ļ���ʶ��
 * ����ժҪ�� KeyScan  & Led
 * ����˵���� ��
 * ��ǰ�汾�� V1.0
 * ��    �ߣ� andrew kong
 * ������ڣ� 2019��12��19��
 *
 *******************************************************************************/
#ifndef __KEYSCAN_H
#define __KEYSCAN_H

#include "basic.h"


#define   KEY0        (GPIO2_PDI & 0x0020) //P2.5

#define   LED_ON      (GPIO0_PDO &= 0x7FFF) //LED3-P0.15
#define   LED_OFF     (GPIO0_PDO |= 0x8000)

#define   KEY_FLITER  10//ms

#define   KEY_START       0 
#define   KEY STOP        1

typedef struct
{
//key0
  s16 nKey0Value;
  s16 nKey0PressCnt;
  s16 nFlagKey0Press;  
} KeyControl_t;


extern KeyControl_t key_state;

extern void KeyScan(KeyControl_t *pKeyState);
extern void KeyInit(void);
#endif

