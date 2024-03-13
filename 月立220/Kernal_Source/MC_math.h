/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� MC_math.h
 * �ļ���ʶ��
 * ����ժҪ�� ������������ѧ���㺯��
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
 /*------------------------------prevent recursive inclusion -------------------*/
#ifndef __MC_MATCH_H
#define __MC_MATCH_H

#include "Global_Variable.h"

#define DivSQRT_3               (s16)0x49E6     /* 1/sqrt(3) in q1.15 format=0.5773315*/
#define SIN_MASK                0x0300
#define U0_90                   0x0200
#define U90_180                 0x0300
#define U180_270                0x0000
#define U270_360                0x0100

UINT16 CurrentMagCalc(INT16 cx,INT16 cy);
s16 wGet_Atan(s32 x, s32 y);

#endif
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
