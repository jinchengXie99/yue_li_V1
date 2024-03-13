/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� Global_Variable.h
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

#ifndef __GLOBAL_VARIABLE__
#define __GLOBAL_VARIABLE__

#include "basic.h"
#include "function_config.h"
#include "foc_drive.h" 
#include "state_machine.h"
#include "pmsmFluxObserve.h"
#include "MC_IPD.h"
#include "mc_hall.h"

extern const char sVersion[10];                      /* ����汾 */
                                                  
extern stru_TaskSchedulerDef struTaskScheduler;      /* ������Ƚṹ�� */

extern stru_FOC_CtrProcDef struFOC_CtrProc;          /* FOC�����ڻ��ṹ�� */
extern stru_FOC_CurrLoopDef struFOC_CurrLoop;        /* �����ڻ��ṹ�� */

extern stru_CommData struAppCommData;                /* �������̿��ƽṹ�� */

extern stru_FluxOB_ParameterDef struFluxOB_Param;    /* �۲��������ṹ�� */

extern stru_MotorParameterDef struMotorParam;        /* ��������ṹ�� */

extern stru_OpenForceRunDef mOpenForceRun;           /* ǿ�����нṹ�� */
 
extern stru_BoardParameterDef struBoardParam;        /* ���ʰ�Ӳ����·���Բ��� */ 

extern stru_HallProcessDef struHallProcess;

extern stru_TransCoef struUser2Core, struUser2App;   /* ���ۻ����� */
extern stru_TransCoef struCore2App, struApp2Core;    /* ���ۻ����� */

extern stru_IPD_CtrProcDef mIPD_CtrProc;             /* IPD ת��λ�ù��ƽṹ�� */
extern stru_HallProcessDef  struHallProcess;         /* Hall �źŴ���ṹ�� */

extern Stru_FOC_ConstParame struFOC_ConstParam;      /* FOC ���� �������� */
extern MechanicalQuantity struMotorSpeed;            /* ת��ʸ���ṹ�壬core */

extern void DebugPWM_OutputFunction(void);
extern stru_FluxOBS_Def m_FluxOBSPara;               /* �۲������� */

#endif

/* ********************** (C) COPYRIGHT LINKO SEMICONDUCTOR ******************** */
/* ------------------------------END OF FILE------------------------------------ */
