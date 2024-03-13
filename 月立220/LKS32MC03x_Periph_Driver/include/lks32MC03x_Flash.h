/*******************************************************************************
 * ��Ȩ���� (C)2021, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� LKS32MC_03x_flash.h
 * �ļ���ʶ��
 * ����ժҪ�� flash������������
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ�   HMG
 * ������ڣ� 2021��10��14��
 *
 *
 *******************************************************************************/
#ifndef __LKS32MC03x_FLASH__
#define __LKS32MC03x_FLASH__
#include "lks32mc03x.h"


extern volatile u32 erase_flag;
extern volatile u32 progm_flag;

void EraseSector(u32 adr, u16 nvr, u32 erase_flag);
//void ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf);
int ProgramPage(u32 adr, u32 sz, u8 *buf, u16 nvr, u32 progm_flag);
void Read_Flash(uint32_t adr, u32 *buf, unsigned long sz, u16 nvr);
extern u32  NVR_UserRoomRead(u32 addr);
extern void Nvr_UserRoomWrite(UINT8 addr,UINT32 data);
extern void Nvr_UserRoomErase(UINT8 addr);
#endif

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
