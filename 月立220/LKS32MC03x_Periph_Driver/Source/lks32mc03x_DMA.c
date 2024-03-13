
 /**
 * @file 
 * @copyright (C)2015, LINKO SEMICONDUCTOR Co.ltd
 * @brief �ļ����ƣ� LKS32MC03x_DMA.c\n
 * �ļ���ʶ�� �� \n
 * ����ժҪ�� DMA������������ \n
 * ����˵���� �� \n
 *@par �޸���־:
 * <table>
 * <tr><th>Date	        <th>Version  <th>Author  <th>Description
 * <tr><td>2021��10��14�� <td>1.0     <td>Yangzj      <td>����
 * </table>
 */
#include "lks32mc03x_DMA.h"
#include "string.h"

 /**
 *@brief @b ��������:   void DMA_StructInit(DMA_InitTypeDef *DMAInitStruct)
 *@brief @b ��������:   DMA�ṹ���ʼ��
 *@see���������ݣ�       ��
 *@param���������       DMA_InitTypeDef
 *@param���������       ��
 *@return�� �� ֵ��      ��
 *@note����˵����        ��
 *@warning              ��   
 *@par ʾ�����룺
 *@code    
           DMA_InitTypeDef DMA_InitStructure;
		   DMA_StructInit(&DMA_InitStructure); //��ʼ���ṹ��
  @endcode    
 *@par �޸���־:
 * <table>
 * <tr><th>Date	        <th>Version  <th>Author  <th>Description
 * <tr><td>2021��10��14�� <td>1.0     <td>Yangzj     <td>����
 * </table>
 */
void DMA_StructInit(DMA_InitTypeDef *DMAInitStruct)
{
    memset(DMAInitStruct, 0, sizeof(DMA_InitTypeDef));
}

 /**
 *@brief @b ��������:   void DMA_Init(DMA_RegTypeDef *DMAx, DMA_InitTypeDef *DMAInitStruct)
 *@brief @b ��������:   DMA��ʼ������
 *@see���������ݣ�       ��
 *@param���������       DMAx , ADC_InitTypeDef
 *@param���������       ��
 *@return�� �� ֵ��      ��
 *@note����˵����        ��
 *@warning              ��
 *@par ʾ�����룺
 *@code    
    DMA_StructInit(&DMA_InitStruct);
    DMA_InitStruct.DMA_IRQ_EN = DISABLE ;             // DMA ��������ж�ʹ��
    DMA_InitStruct.DMA_CIRC = DISABLE;                // DMA����ģʽ��ѭ��ģʽ������Ч 
    DMA_InitStruct.DMA_SINC = ENABLE;                 // Դ��ַ����,  ����Ч,��ַ���� SBTW ��Ӧ��С���� 1/2/4
    DMA_InitStruct.DMA_DINC = DISABLE;                // Ŀ�ĵ�ַ����,����Ч,��ַ���� DBTW ��Ӧ��С���� 1/2/4
    DMA_InitStruct.DMA_SBTW = DMA_BYTE_TRANS;         // Դ����λ�� 0:byte, 1:half-word, 2:word 
    DMA_InitStruct.DMA_DBTW = DMA_BYTE_TRANS;         // Ŀ�ķ���λ�� 0:byte, 1:half-word, 2:word 
    DMA_InitStruct.DMA_REQ_EN = DMA_SPI_TX_REQ_EN;    // ͨ�� UART_TX DMA����ʹ�ܣ�����Ч 
    DMA_InitStruct.DMA_RMODE = ENABLE;                // 0:���ִ��䣬һ������������ �� 1:���֣�ÿ�ֽ���һ�����ݴ��� 
    DMA_InitStruct.DMA_SADR = (u32)Txaddr;            // DMA ͨ�� x Դ��ַ 
    DMA_InitStruct.DMA_DADR = (u32)&SPI_TX_DATA;      // DMA ͨ�� x Ŀ�ĵ�ַ 
    DMA_Init(DMA_CH0, &DMA_InitStruct);

  @endcode   
 *@par �޸���־:   
 * <table>
 * <tr><th>Date	        <th>Version    <th>Author      <th>Description
 * <tr><td>2021��10��14�� <td>1.0     <td>Yangzj       <td>����
 * </table>
 */
void DMA_Init(DMA_RegTypeDef *DMAx, DMA_InitTypeDef *DMAInitStruct)
{
    /* ͨ�����üĴ��� ��ֵ*/
    DMAx->DMA_CCR  = 0;
    DMAx->DMA_SADR = DMAInitStruct->DMA_SADR;   /* �����ַ�Ĵ��� */
    DMAx->DMA_DADR = DMAInitStruct->DMA_DADR;   /* �ڴ��ַ�Ĵ��� */
    DMAx->DMA_REN  = DMAInitStruct->DMA_REQ_EN;
    DMAx->DMA_CCR  = (DMAInitStruct -> DMA_SBTW  << 10) | (DMAInitStruct -> DMA_DBTW  <<8) |
                     (DMAInitStruct -> DMA_SINC  <<  6) | (DMAInitStruct -> DMA_DINC  <<4) |
                     (DMAInitStruct -> DMA_CIRC  <<  3) | (DMAInitStruct -> DMA_RMODE <<1);
      
    if(DMAInitStruct->DMA_IRQ_EN)
    switch((u32)DMAx)
    {
        case (u32)DMA_CH0:
            DMA_IE |= BIT0;
            break;
        case (u32)DMA_CH1:
            DMA_IE |= BIT1;
            break;
        case (u32)DMA_CH2:
            DMA_IE |= BIT2;
            break;
        case (u32)DMA_CH3:
            DMA_IE |= BIT3;
            break;
    }
    DMA_CTRL  = 0x0001;  /*DMA����ʹ��*/
}  

/**
 *@brief @b ��������:   void DMA_CHx_EN(DMA_RegTypeDef *DMAx,u32 Channel_EN,u32 set)
 *@brief @b ��������:   ʹ��DMAͨ��
 *@see���������ݣ�       DMAx��ѡ�� DMA_CH0 �� DMA_CH1 �� DMA_CH2 �� DMA_CH3
 *@param���������       DMAx��DMAͨ��ѡ�� \n 
                        Channel_EN��ENABLE��ʹ��DMAͨ����DISABLE���ر�DMAͨ��ʹ�� \n
                        set: DMAͨ�� x ���ݰ������������1~255 \n         
 *@param���������       ��
 *@return�� �� ֵ��      ��
 *@note����˵����        ��
 *@warning              ��   
 *@par ʾ�����룺
 *@code    
           DMA_CHx_EN(DMA_CH0,ENABLE,1);//ʹ��DMAͨ��0
  @endcode    
 *@par �޸���־:
 * <table>
 * <tr><th>Date	        <th>Version  <th>Author  <th>Description
 * <tr><td>2022��4��13�� <td>1.1       <td>HuangMG    <td>����
 * </table>
 */
void DMA_CHx_EN(DMA_RegTypeDef *DMAx,u32 Channel_EN,u32 set)
{
	 DMAx->DMA_CCR  &= ~BIT0; /*�ر�ͨ��ʹ��*/
	 DMAx->DMA_CTMS = set;    /*����DMA�������������*/
   if(Channel_EN)
	 {
	   DMAx->DMA_CCR  |= BIT0;/*ʹ��DMA����*/
	 }
}


/**
 *@brief @b ��������:   uint32_t DMA_GetIRQFlag(u32 timer_if)
 *@brief @b ��������:   ��ȡDMA�жϱ�־
 *@see���������ݣ�       ��
 *@param���������      timer_if������ѡ�� 
 * <table>              <tr><td> �궨��          <td>˵��
                        <tr><th> CH3_FIF    <td>DMAͨ�� 3 ����жϱ�־
 *					    <tr><th> CH2_FIF    <td>DMAͨ�� 2 ����жϱ�־
 *						<tr><th> CH1_FIF	<td>DMAͨ�� 1 ����жϱ�־
 *						<tr><th> CH0_FIF    <td>DMAͨ�� 0 ����жϱ�־
 * </table>  
 *@see 
 *@param���������       ��
 *@return�� �� ֵ��      ��
 *@note����˵����        ��
 *@warning              ֻ�ж�Ӧ�ж�ʹ�ܺ󣬸�Ϊ���ܶ�ȡ�������Ӧ�ж�δʹ�ܣ���ȡ���һֱΪ0   
 *@par ʾ�����룺
 *@code    
           if(DMA_GetIRQFlag(CH0_FIF))//��ȡDMAͨ��0����жϱ�־
		   {	
		   }
  @endcode    
 *@par �޸���־:
 * <table>
 * <tr><th>Date	        <th>Version  <th>Author  <th>Description
 * <tr><td>2022��4��13�� <td>1.0      <td>HuangMG    <td>�޸�
 * </table>
 */
uint32_t DMA_GetIRQFlag(u32 timer_if)
{
   if(DMA_IF & timer_if && DMA_IE)
	 {
	   return 1;
	 }
	 return 0;
}



/**
 *@brief @b ��������:   void DMA_ClearIRQFlag(uint32_t tempFlag)
 *@brief @b ��������:   ���DMA�жϱ�־
 *@see ���������ݣ�     
 *@param ���������      DMAx��DMAͨ��ѡ��  \n 
                        tempFlag������ѡ�� 
 * <table>              <tr><td> �궨��          <td>˵��
                       <tr><th> CH3_FIF    <td>DMAͨ�� 3 ����жϱ�־
 *					    <tr><th> CH2_FIF    <td>DMAͨ�� 2 ����жϱ�־
 *						<tr><th> CH1_FIF	<td>DMAͨ�� 1 ����жϱ�־
 *						<tr><th> CH0_FIF    <td>DMAͨ�� 0 ����жϱ�־
 * </table>   
 * 
 *@param ���������   ��
 *@return �� �� ֵ��  ��
 *@note ����˵����    ��
 *@warning           ��
 *@par ʾ�����룺
 *@code    
           if(DMA_GetIRQFlag(CH0_FIF))//��ȡDMAͨ��0����жϱ�־
		   {	
			  DMA_ClearIRQFlag(CH0_FIF)//���DMAͨ��0����жϱ�־
		   }
  @endcode 
 *@par �޸���־:
 * <table>
 * <tr><th>Date	        <th>Version    <th>Author    <th>Description
 * <tr><td>2022��4��13��   <td>1.0      <td>HuangMG      <td>����
 * </table>
 */
void DMA_ClearIRQFlag(uint32_t tempFlag)
{
	  DMA_IF = tempFlag;
}



