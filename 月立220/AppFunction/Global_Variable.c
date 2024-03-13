/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� Global_Variable.c
 * �ļ���ʶ��
 * ����ժҪ�� ȫ�ֱ��������ļ�
 * ����˵���� ��
 * ��ǰ�汾�� V 1.0
 * ��    �ߣ� Howlet
 * ������ڣ� 2020��8��5��
 *
 * �޸ļ�¼1��
 * �޸����ڣ�2020��8��5��
 * �� �� �ţ�V 1.0
 * �� �� �ˣ�Howlet
 * �޸����ݣ�����
 *
 *******************************************************************************/

#include "Global_Variable.h"
#include "time_process.h"
#include "MC_Parameter.h"

const char sVersion[10] = "Ver_3.9";        /* ����汾 */

stru_TaskSchedulerDef struTaskScheduler;    /* ������Ƚṹ�� */

stru_FOC_CtrProcDef struFOC_CtrProc;        /* FOC����������̽ṹ�� */

stru_FOC_CurrLoopDef struFOC_CurrLoop;      /* �����ڻ��ṹ�� */

stru_FluxOB_ParameterDef struFluxOB_Param;  /* �۲��������ṹ�� */

stru_CommData struAppCommData;              /* �������̿��ƽṹ�� */

MechanicalQuantity struMotorSpeed;          /* ת��ʸ���ṹ�壬core */

Stru_OnTheFlyDetect mOnTheFlyDetect;        /* ˳��������ṹ�� */

stru_HallProcessDef struHallProcess;

stru_PIRegulator mSpeedPI;                  /* �ٶȻ� */

stru_OpenForceRunDef mOpenForceRun;         /* ǿ�����нṹ�� */

stru_TransCoef struUser2App;                /* ���ۻ����� */
stru_TransCoef struCore2App, struApp2Core;  /* ���ۻ����� */

//stru_HallProcessDef  struHallProcess;       /* Hall �źŴ���ṹ�� */

stru_IPD_CtrProcDef mIPD_CtrProc;           /* IPD ת��λ�ù��ƽṹ�� */

/* FOC ���� �������� */
Stru_FOC_ConstParame struFOC_ConstParam = FOC_CONST_PARAMETER;

/* ��������ṹ�� */
stru_MotorParameterDef struMotorParam = MOTOR_PARAMETER_TAB;
/* ���ʰ�Ӳ����·���Բ��� */
stru_BoardParameterDef struBoardParam = BOARD_PARAMETER_TAB;




/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
