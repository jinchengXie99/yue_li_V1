/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� function_config.h
 * �ļ���ʶ��
 * ����ժҪ�� ���Ʋ������ͷ�ļ�
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� Howlet
 * ������ڣ� 2020��8��5��
 *
 * �޸ļ�¼1��
 * �޸����ڣ� 2020��8��5��
 * �� �� �ţ� V 1.0
 * �� �� �ˣ� Howlet
 * �޸����ݣ� ����
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __FUNCTION_CFG_PARAM_H
#define __FUNCTION_CFG_PARAM_H

#include "hardware_config.h"

/* -------------------------------ADC ��ƫ�ô�������---------------------------- */
#define ADC_GET_OFFSET_SAMPLES         (9)           /* ƫ�ò�������2��9�η�, 512�� */
#define ADC_OFFSET_ERROR               (3500)        /* ADC��ƫ�ô��� */

/* -------------------------------�洢��ַ��ض���---------------------------- */ 
#define HALL_LEARN_ADDR                0x7600       /* Hallѧϰ�洢���ַ */


/* ------------------------------FOC������ض���------------------------------ */
#define MAX_MODULE_VALUE               18919    /* PWM���Ʊ� 1/sqrt(3)*32767 */ 

/* ---------------------------------������ض���------------------------------ */
#define TEST_ON                        1
#define TEST_OFF                       0

#define DEBUG_PWM_OUTPUT               TEST_OFF       /* PWM�ڵ��Ե�ʱ�����һ��ռ�ձ� */

#define ALTERNATE_OUT_PWM              TEST_OFF      /* PWM������ԣ��������Ch012 CH3 */

#define OPEN_LOOP_FORCE_TEST           TEST_OFF       // ǿ�Ƶ�ѹ���


#define FUNCTION_ON                    1
#define FUNCTION_OFF                   0

#define RTT_FUNCTION                   FUNCTION_OFF   /* RTT ���Թ��� */


#endif /* __FUNCTION_CFG_PARAM_H */

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
