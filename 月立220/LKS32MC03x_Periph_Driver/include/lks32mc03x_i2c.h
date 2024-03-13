/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� lks32mc03x_I2C.h
 * �ļ���ʶ��
 * ����ժҪ�� I2C������������ͷ�ļ�
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� YangZJ
 * ������ڣ� 2021��11��09��
 *
 *******************************************************************************/
#ifndef __lks32mc03x_I2C_H
#define __lks32mc03x_I2C_H
/* Includes ------------------------------------------------------------------*/
#include "lks32mc03x.h"
#include "basic.h"

//I2C_BUS_STATE   I2C����״̬
#define I2C_BUS_STATE_STT_ERR     BIT7   // ���ߴ���
#define I2C_BUS_STATE_LOST_ARB    BIT6   // �����ٲö�ʧ
#define I2C_BUS_STATE_STOP_EVT    BIT5   // STOP�¼���¼

#define I2C_BUS_STATE_ADDR_DATA   BIT3   // ���ڴ����ַ����

#define I2C_BUS_STATE_RX_ACK      BIT1   // ���յ�ACK��Ӧ
#define I2C_BUS_STATE_Done        BIT0   // �������

typedef struct
{
    __IO uint32_t ADDR;   // I2C��ַ�Ĵ���
    __IO uint32_t CFG;    // I2C���üĴ���
    __IO uint32_t SCR;    // I2C״̬�Ĵ���
    __IO uint32_t DATA;   // I2C���ݼĴ���
    __IO uint32_t MSCR;   // I2C��ģʽ�Ĵ���
    __IO uint32_t BCR;    // I2C������ƼĴ���
    __IO uint32_t BSIZE;  // I2C���䳤�ȼĴ���
} I2C_TypeDef;

typedef struct
{
    u32 ADRCMP          ;   /**<   I2C Ӳ����ַ�Ƚ�ʹ�ܿ��أ�ֻ���� DMA ģʽ�¿�������Ч*/
    u32 MST_MODE        ;   /**<   I2C ��ģʽʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
    u32 SLV_MODE        ;   /**<   I2C ��ģʽʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
    u32 DMA             ;   /**<   I2C DMA����ʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
    u32 BaudRate        ;   /**<   I2C ������*/
    u32 IE              ;   /**<   I2C �ж�ʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
    u32 TC_IE           ;   /**<   I2C ���ݴ�������ж�ʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
    u32 BUS_ERR_IE      ;   /**<   I2C ���ߴ����¼��ж�ʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
    u32 STOP_IE         ;   /**<   I2C STOP �¼��ж�ʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
    u32 BURST_NACK      ;   /**<   I2C ���䣬NACK �¼��ж�ʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
    u32 BURST_ADDR_CMP  ;   /**<   I2C ���䣬Ӳ����ַƥ���ж�ʹ�ܣ�ENABLE:ʹ�ܣ�DISABLE:ʧ��*/
}I2C_InitTypeDef;

void I2C_Init(I2C_InitTypeDef *);        // I2C��ʼ��
void I2C_StructInit(I2C_InitTypeDef *);   // I2C���ýṹ���ʼ��

u8 Read_I2c_Bus_State(u16);            // ��I2C����״̬
void Clear_I2c_Bus_State(u16);          // I2C����״̬��λ

// i2c&dma����&����
void i2c_dma_rx(u8 addr, u8 *data, u8 len);
void i2c_dma_tx(u8 addr, u8 *data, u8 len);
void i2c_dma_init(void);
void i2c_dma_state_over(void);
#endif /*__lks32mc03x_I2C_H */
/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
