/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� lks32mc03x_timer.c
 * �ļ���ʶ��
 * ����ժҪ�� Timer���ó���
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� YangZJ
 * ������ڣ� 2021/11/10
 *
 *******************************************************************************/
#include "lks32mc03x.h"
#include "lks32mc03x_timer.h"
#include "lks32mc03x_sys.h"

/*******************************************************************************
 �������ƣ�    void TIM_TimerInit(TIM_TimerTypeDef *TIMERx, TIM_TimerInitTypeDef *this)
 ����������    Timer��ʼ��
 �����ı�    ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              ����
 *******************************************************************************/
void TIM_TimerInit(TIM_TimerTypeDef *TIMERx, TIM_TimerInitTypeDef *this)
{
    if(TIMERx == TIMER0)
    {
        SYS_ModuleClockCmd(SYS_Module_TIMER0,ENABLE);     //��Timerʱ��
    }
    if(TIMERx == TIMER1)
    {
        SYS_ModuleClockCmd(SYS_Module_TIMER1,ENABLE);     //��Timerʱ��
    }
    TIMERx -> CFG  = (this->EN            << 31) | (this->CAP1_CLR_EN   << 27) |
                     (this->CAP0_CLR_EN   << 26) | (this->ETON          << 24) |
                     (this->CLK_DIV       << 20) | (this->CLK_SRC       << 16) |
                     (this->SRC1          << 12) | (this->CH1_POL       << 11) |
                     (this->CH1_MODE      << 10) | (this->CH1_FE_CAP_EN <<  9) |
                     (this->CH1_RE_CAP_EN <<  8) | (this->SRC0          <<  4) |
                     (this->CH0_POL       <<  3) | (this->CH0_MODE      <<  2) |
                     (this->CH0_FE_CAP_EN <<  1) | (this->CH0_RE_CAP_EN);
    TIMERx -> TH    = this->TH;
    TIMERx -> CNT   = this->CNT;
    TIMERx -> CMPT0 = this->CMP0;
    TIMERx -> CMPT1 = this->CMP1;
    TIMERx -> EVT   = this->EVT_SRC;
    TIMERx -> FLT   = this->FLT;
    TIMERx -> IE    = this->IE;
    TIMERx -> IF    = 0x7;
}
/*******************************************************************************
 �������ƣ�    void TIM_TimerStrutInit(TIM_TimerInitTypeDef *TIM_TimerInitStruct)
 ����������    Timer�ṹ���ʼ��
 �����ı�    ��
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2021/11/10    V1.0           YangZJ              ����
 *******************************************************************************/
#define DISABLE 0
void TIM_TimerStrutInit(TIM_TimerInitTypeDef *this)
{
    this -> EN            = DISABLE;               // Timer ģ��ʹ��
    this -> CAP1_CLR_EN   = DISABLE;      // ������ CAP1 �����¼�ʱ������ Timer ������������Ч
    this -> CAP0_CLR_EN   = DISABLE;      // ������ CAP0 �����¼�ʱ������ Timer ������������Ч
    this -> ETON          = DISABLE;             // Timer ����������ʹ������ 0: �Զ����� 1: �ȴ��ⲿ�¼���������
    this -> CLK_DIV       = 0;                // Timer ��������Ƶ
    this -> CLK_SRC       = TIM_Clk_SRC_MCLK; // Timer ʱ��Դ
    this -> TH            = 0;                     // Timer �������������ޡ��������� 0 ������ TH ֵ���ٴλ� 0 ��ʼ����
            
    this -> SRC1          = TIM_SRC1_1;       // Timer ͨ�� 1 ����ģʽ�ź���Դ
    this -> CH1_POL       = 1;             // Timer ͨ�� 1 �ڱȽ�ģʽ�µ�������Կ��ƣ���������0������ֵ
    this -> CH1_MODE      = 0;            // Timer ͨ�� 1 ����ģʽѡ��0���Ƚ�ģʽ��1������ģʽ
    this -> CH1_FE_CAP_EN = DISABLE; // Timer ͨ�� 1 �½��ز����¼�ʹ�ܡ�1:ʹ�ܣ�0:�ر�
    this -> CH1_RE_CAP_EN = DISABLE; // Timer ͨ�� 1 �����ز����¼�ʹ�ܡ�1:ʹ�ܣ�0:�ر�
    this -> CMP1          = 0;    // Timer ͨ�� 1 �Ƚ�����
            
    this -> SRC0          = TIM_SRC1_0;       // Timer ͨ�� 0 ����ģʽ�ź���Դ
    this -> CH0_POL       = 1;             // Timer ͨ�� 0 �ڱȽ�ģʽ�µ�������Կ��ƣ���������0������ֵ
    this -> CH0_MODE      = 0;            // Timer ͨ�� 0 ����ģʽѡ��0���Ƚ�ģʽ��1������ģʽ
    this -> CH0_FE_CAP_EN = DISABLE; // Timer ͨ�� 0 �½��ز����¼�ʹ�ܡ�1:ʹ�ܣ�0:�ر�
    this -> CH0_RE_CAP_EN = DISABLE; // Timer ͨ�� 0 �����ز����¼�ʹ�ܡ�1:ʹ�ܣ�0:�ر�
    this -> CMP0          = 0;    // Timer ͨ�� 0 �Ƚ�����
            
    this -> CNT           = 0;     // Timer ��������ǰ����ֵ��д��������д���µļ���ֵ��
    this -> EVT_SRC       = 0; // Timer ����ʹ�ܿ�ʼ���ⲿ�¼�ѡ��
    this -> FLT           = 0;     // ͨ�� 0/1 �ź��˲����ѡ��0-255
    this -> IE            = 0;      // Timer �ж�ʹ��
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR *****END OF FILE****/
