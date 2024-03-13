/*******************************************************************************
 * ��Ȩ���� (C)2015, LINKO SEMICONDUCTOR Co.ltd
 *
 * �ļ����ƣ� MC_hall.c
 * �ļ���ʶ��
 * ����ժҪ�� Hall�źŴ���
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

/* Includes ------------------------------------------------------------------*/
#include "Global_Variable.h"
#include "MC_hall.h"
#include "hardware_config.h"
#include "mc_type.h"
#include "time_process.h"

Stru_HallParaConstDef struHallParamaConst =
{   /* Hall���ٶȼ��㣬��Ƕ������س������� */
    ROTOR_SPEED_FACTOR,
    (u32)(ROTOR_SPEED_FACTOR / 3),
    ROTOR_SPEED_FACTOR_SAT,
    ROTOR_SPEED_FACTOR_PER_UNIT,
    ROTOR_SPEED_FACTOR_PER_UNIT1,
    ROTOR_FACTOR_REDUCE_PER_UNIT,
    ROTOR_FACTOR_ECC_UNIT,

    LOW_CLOSE_SPEED_THD,
    LOW_CLOSE_SPEED_THD1,
    LOW_CLOSE_SPEED_THD2,

    PWM_PERIOD,

    PWM_TIME_500uS,
    PWM_TIME_1MS,
    PWM_TIME_2MS,
    PWM_TIME_1MS,
    PWM_TIME_20MS,

    HALL_LEARN_ADDR

};

void hall_comm_VariableInit(stru_HallProcessDef *this);

/*******************************************************************************
 �������ƣ�    void HALL_Init_Electrical_Angle(stru_HallProcessDef *this)
 ����������    Hall�仯ʱ����ʼ����ǰ��Ƕ�
 ���������    stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void HALL_Init_Electrical_Angle(stru_HallProcessDef *this)
{

    if(this->bHallRunFlg & HALL_COM_ERR)
    {   /* �����ת����180413 HL */
        this->bCloseLoopCnt = 0;
        this->bCloseLoopAngleFlg = 0;
        this->nHallChangeCnt = 0;

        this->nPhaseShift = this->nPhaseShiftOffset;
//      PID_Flux.hKi_Gain = PID_FLUX_KI_DEFAULT;
    }


    if((this->wMotorSpeedAvgCnt < this->wCloseLoopPeriodThd))
    {   /* �ٶȴ��ڱջ����ޣ���ͨ�ջ����� */
        if(this->bCloseLoopCnt < this->bOpenStartUpCntThd)
        {
            this->bCloseLoopCnt++;
        }
        else
        {
            if(this->bCloseLoopCnt < 20)
            {
                this->bCloseLoopCnt ++;
            }
            //��ͨ�ջ�����
            this->bCloseLoopAngleFlg = 1;

            if(this->bHallRunFlg & HALL_DIR_FLG)
            {
                this->nPhaseShift = this->nPhaseShiftOffset - S16_30_PHASE_SHIFT;

                if(this->bMotorDirtionCtrl)
                {
                    this->bCloseLoopCnt = 0;
                    this->bCloseLoopAngleFlg = 0;
                    this->bMotorDirtionCtrl = 0;
                }

            }
            else
            {   /* �����ת���� */
                this->nPhaseShift = this->nPhaseShiftOffset + S16_30_PHASE_SHIFT;
                if(this->bMotorDirtionCtrl == 0)
                {
                    this->bCloseLoopCnt = 0;
                    this->bCloseLoopAngleFlg = 0;
                    this->bMotorDirtionCtrl = 1;
                }


            }

        }
    }
    else
    {



        /* �ٶ�С�ڿ������ޣ���ͨ�������� */
        if(this->bCloseLoopAngleFlg)
        {
            this->nHallChangeCnt = 0;
        }
        this->bCloseLoopAngleFlg = 0;
        this->bCloseLoopCnt = 0;

        this->nPhaseShift = this->nPhaseShiftOffset;


    }

    /* ���õ���ǰHallλ���µĽǶ�ֵ */
    GetHall_edgeAngle(this);

    //hall״̬�����ı�
    if(this->bOldHallState != this->bHallState)
    {   /* ������з����־��0:��ת 1:��ת */
        if(this->bHallRunFlg & HALL_DIR_FLG)
        {
            if(this->nHallQEP_Cnt < 15)
            {
                this->nHallQEP_Cnt ++;
            }
            //�������Ƶĵ�����з���
            if(this->bMotorDirtionCtrl == 0)
            {
                this->nHallChangeCnt ++;
            }
            else
            {
                this->nHallChangeCnt = 0;
            }
            //�����ٶȷ��� �ٶȵķ��ű�־λ��1;
            this-> bHallSpeedDirFlag = 1;
        }
        else
        {
            if(this->nHallQEP_Cnt > -15)
            {
                this->nHallQEP_Cnt --;
            }

            if(this->bMotorDirtionCtrl)
            {
                this->nHallChangeCnt ++;
            }
            else
            {
                this->nHallChangeCnt = 0;
            }
            //�����ٶȷ��� �ٶȵķ��ű�־λ��-1;
            this->bHallSpeedDirFlag = (-1);
        }
        this->bOldHallState = this->bHallState;


    }

    Angle_Init_Process(this);

    //��ͨ�˲�
    this->nRotorFreqDppFir = lowPass_filter(&this->stru_HallDppRC, this->nRotorFreqDpp);

    //ת�����ٶȱջ������������� ԭʼ����ʵ���ٶȱ�
    //hall�������� Sensor less���� 0.194161*1.25051=0.2428012871536524    *����λ
    this->wSpeedfbkHall = App2CoreFreqTrans(User2AppFreqTrans(this->nRotorFreqDppFir * 0.2428)) * (this->bHallSpeedDirFlag);
    //    hall_change_flg = 1;
}




/*******************************************************************************
 �������ƣ�    void HALL_Init_Electrical_Angle(stru_HallProcessDef *this)
 ����������    Hall��ر�����ʼ��
 ���������    stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void HALL_InitHallMeasure(stru_HallProcessDef *this)
{
    u8 t_i;

    this->nReduceToqueDpp = 0;

    this->bSpeedFIFO_Index = HALL_SPEED_FIFO_SIZE - 1;

    this->bHallRunFlg |= HALL_COM_ERR;

    for(t_i = 0; t_i < HALL_SPEED_FIFO_SIZE; t_i++)
    {
        this->wSensorPeriod[this->bSpeedFIFO_Index] = ROTOR_SPEED_FACTOR / 3;
    }

    this->wMotorSpeedAvgCnt = ROTOR_SPEED_FACTOR / 3;
    HALL_Init_Electrical_Angle(this);
    this->bHallRunFlg &= ~HALL_COM_ERR;

    //������hall�����ͺͽǶ�
    this->nHallOffsetArry[0] = S16_60_PHASE_SHIFT;
    this->nHallOffsetArry[1] = S16_120_PHASE_SHIFT;
    this->nHallOffsetArry[2] = S16_180_PHASE_SHIFT;
    this->nHallOffsetArry[3] = S16_240_PHASE_SHIFT;
    this->nHallOffsetArry[4] = S16_300_PHASE_SHIFT;
    this->nHallOffsetArry[5] = 0;


    this->bHallCommTab[0] = 3; //--1(30��)
    this->bHallCommTab[1] = 1; //--5(90��)
    this->bHallCommTab[2] = 5; //--4(150��)
    this->bHallCommTab[3] = 4; //--6(210��)
    this->bHallCommTab[4] = 6; //--2(270��)
    this->bHallCommTab[5] = 2; //--3(330��)

    this->bHallCommTab[0] = 3; //--1(30��)
    this->bHallCommTab[1] = 2; //--5(90��)
    this->bHallCommTab[2] = 6; //--4(150��)
    this->bHallCommTab[3] = 4; //--6(210��)
    this->bHallCommTab[4] = 5; //--2(270��)
    this->bHallCommTab[5] = 1; //--3(330��)
    
    this->bHallCommTab[0] = 6; //--1(30��)
    this->bHallCommTab[1] = 2; //--5(90��)
    this->bHallCommTab[2] = 3; //--4(150��)
    this->bHallCommTab[3] = 1; //--6(210��)
    this->bHallCommTab[4] = 5; //--2(270��)
    this->bHallCommTab[5] = 4; //--3(330��)
    
    
    this->bHallCommTab[0] = 4; //--1(30��)
    this->bHallCommTab[1] = 6; //--5(90��)
    this->bHallCommTab[2] = 2; //--4(150��)
    this->bHallCommTab[3] = 3; //--6(210��)
    this->bHallCommTab[4] = 1; //--2(270��)
    this->bHallCommTab[5] = 5; //--3(330��)


    hall_comm_VariableInit(this);
    this->pStru_HallParama = &struHallParamaConst;      /* �ṹ��ָ���ʼ�� */
    this->bHallType = 120;                               /* Hall����Ϊ120��Hall; 120 */
    this->nPhaseShiftOffset = S16_270_PHASE_SHIFT;      //S16_315_PHASE_SHIFT      /* ��ǰHall�Ƕ�ƫ��ֵ���� */
    this->bOpenStartUpCntThd = 6;                        /* ����״̬����ת�ջ���������ֵ */
    this->wCloseLoopPeriodThd = HALL_PLL_OPEN2CLOSE_THD; /* HALL PLL �����бջ��ٶ�����ֵ */
    this->wOpenLoopPeriodThd  = HALL_PLL_CLOSE2OPEN_THD; /* HALL PLL �ջ��п����ٶ�����ֵ */
    this->stru_HallDppRC.coef = FRAC16(0.2);             /* Hall���ٶȵ�ͨ�˲�FRAC16(0.2) */
    this->stru_HallDppRC.yk_1=0;                         /* Hall���ٶȵ�ͨ�˲�FRAC16(0.2) */
}



/*******************************************************************************
 �������ƣ�    void HALL_GetElectricalAngle(stru_HallProcessDef *this)
 ����������    Hall�仯ʱ����ʼ����ǰ��Ƕ�
 ���������    stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��    ���ص�ǰת�ӵ����Ƕ�λ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
__inline s16 HALL_GetElectricalAngle(stru_HallProcessDef *this)
{
    return(this->nElectrical_Angle);
}


/*******************************************************************************
 �������ƣ�    void HALL_IRQProcess()
 ����������    Hall�жϴ�����
 ���������    HALL_TypeDef *HALLx : Hallģ��HALL0��HALL1
               stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void HALL_IRQProcess(HALL_TypeDef *HALLx, stru_HallProcessDef *this)
{
    unsigned char temp_state, t_FuncState;
    volatile u8 t_i, t_cnt;

    if(HALLx->INFO & BIT16)
    {   /* Hall �仯�ж� */
        this->wHallCapCnt += HALLx->WIDTH;  /* �ۼƲ�׽����HALL����ֵ */
        HALLx->INFO = BIT16;

        temp_state = ReadHallCapState(HALLx);

        t_FuncState = check_hall_state(temp_state, this);

        if(t_FuncState)
        {
            if (this->bSpeedFIFO_Index != HALL_SPEED_FIFO_SIZE - 1)
            {   /* FIFO ָ����� */
                this->bSpeedFIFO_Index++;
            }
            else
            {
                this->bSpeedFIFO_Index = 0;
            }

            if(this->bFstHallSigFlg)
            {
                this->bFstHallSigFlg = 0;
            }
            else
            {
                u32 t_CapCnt;

                if(this->stru_HallRepare.bBrokenHallFlg)
                {   /* Hall������ʱ����Ĵ�������������PWM���� */
                    t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;

                    if(t_CapCnt < MAX_SPEED_DPP)
                    {
                        t_CapCnt = MAX_SPEED_DPP;
                    }

                    this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;
                }
                else
                {


                    if(this->wHallCapCnt < MAX_SPEED_DPP)
                    {
                        this->wHallCapCnt = MAX_SPEED_DPP;
                    }

                    this->wSensorPeriod[this->bSpeedFIFO_Index] = this->wHallCapCnt;
                }
            }

            this->wHallCapCnt = 0;
            this->wHallPWMTimerCnt = 0;

            this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(this);

            if(this->bHallLearnFlg == 0)
            {   /* ѧϰ״̬�����½Ƕ� */
                HALL_Init_Electrical_Angle(this);
            }
        }

    }

    if(HALLx->INFO & BIT17)
    {   /* Hall��ʱ�ж� */

        HALLx->INFO = BIT17;

        this->bHallRunFlg |= HALL_COM_ERR;
        this->bMotorDirtionCtrl = 0;

        this->bSpeedFIFO_Index = 0;

        for(t_i = 0; t_i < HALL_SPEED_FIFO_SIZE; t_i++)
        {   /* Hall��ʱ����ʼ��FIFOֵΪ�������ֵ */
            this->wSensorPeriod[this->bSpeedFIFO_Index] = ROTOR_SPEED_FACTOR / 3;
        }
        this->wMotorSpeedAvgCnt = ROTOR_SPEED_FACTOR / 3;

        temp_state = ReadHallState(HALLx);

        t_FuncState = check_hall_state(temp_state, this);

        if(this->bHallLearnFlg == 0)
        {   /* ѧϰ״̬�����½Ƕ� */
            HALL_Init_Electrical_Angle(this);
        }
    }

}

/*******************************************************************************
 �������ƣ�    void GetAvrgHallPeriod()
 ����������    ����Hallƽ�����ڱ仯ֵ
 ���������    stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��    Hallƽ�����ڱ仯ֵ����λֵΪHALLģ��������ʱ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
static __inline u32 GetAvrgHallPeriod(stru_HallProcessDef *this)
{
    u32 wAvrgBuffer;
    u8 wIndex;


    wAvrgBuffer = 0;

    if(this->bCloseLoopCnt >= 6)
    {

        for ( wIndex = 0; wIndex < HALL_SPEED_FIFO_SIZE; wIndex++ )
        {
            wAvrgBuffer += this->wSensorPeriod[wIndex];
        }

        wAvrgBuffer /= HALL_SPEED_FIFO_SIZE;  /* Average value */
    }
    else
    {
        wAvrgBuffer = (this->wOldSensorPeriod + this->wSensorPeriod[this->bSpeedFIFO_Index]) >> 1;
        if(wAvrgBuffer < ROTOR_SPEED_FACTOR / 2000)
        {
            wAvrgBuffer = ROTOR_SPEED_FACTOR / 3;
        }
    }
    this->wOldSensorPeriod = this->wSensorPeriod[this->bSpeedFIFO_Index];


    return (wAvrgBuffer);
}

/*******************************************************************************
 �������ƣ�    void Angle_Init_Process(stru_HallProcessDef *this)
 ����������    ��Ƕȳ�ʼ����Pll����ʱ��ֱ�Ӳ��ȵ���ʼ�Ƕȡ�Reduce toqueģʽ��
               �����Ƕ�dppֵ
 ���������    stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
static __inline void Angle_Init_Process(stru_HallProcessDef *this)
{
    if(this->bCloseLoopAngleFlg)
    {
        if(this->wMotorSpeedAvgCnt > this->wOpenLoopPeriodThd)
        {
            this->nEndElectricalAngle = (s16)(this->nTabElectrical_Angle - this->nElectrical_Angle);
            this->bFastAddElecAngleFlg = 1;
        }

        closeLoopAnglePLLInit(this->nTabElectrical_Angle, this);
        this->nMaxIncAngle = 0;
    }
    else
    {
        u16 t_RotorFreq_dpp;

        this->nRotorFreqDpp = ROTOR_SPEED_FACTOR / this->wMotorSpeedAvgCnt;

//�˴������˷������ת��
//        if(this->nRotorFreqDpp > 100)
//        {
//            this->nRotorFreqDpp = 100;
//        }

        if(this->bHallRunFlg & HALL_COM_ERR)
        {   /* �������ֱ�Ӹ����Ƕ� */

            if(this->bHallRunFlg & HALL_DIR_FLG)
            {
                this->nEndElectricalAngle = (s16)(this->nTabElectrical_Angle - this->nElectrical_Angle);
                this->bFastAddElecAngleFlg = 1;
            }
            else
            {
                this->nElectrical_Angle = this->nTabElectrical_Angle;
            }

            this->bReduceToqueFlg = 0;



        }
        else
        {   /* ��������ʱ���������ջ���� */
            calc_first_ElectAngle(this->nTabElectrical_Angle, this);
            
            t_RotorFreq_dpp = (u32)(ROTOR_FACTOR_REDUCE_PER_UNIT * this->nMaxReduceIncAngle)
                                      / this->wSensorPeriod[this->bSpeedFIFO_Index];

            if(t_RotorFreq_dpp < 10)
            {
                this->nReduceToqueDpp = 10;
            }

            else
            {
                this->nReduceToqueDpp = t_RotorFreq_dpp;
                if(this->nReduceToqueDpp > 200)
                {
                    this->nReduceToqueDpp = 200;
                }
            }
        }
    }
}

/*******************************************************************************
 �������ƣ�    void Verify_Hall_State(stru_HallProcessDef *this)
 ����������    ��֤��ǰHall�Ƿ���Ч����Hall�жϼ�⵽����ֵʱ���������Hallֵ
               ����һ��Hall��ʱ��ִ��Hall���޸�
 ���������    HALL_TypeDef *HALLx : Hallģ��HALL0��HALL1
               stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��
 ���������:   ���漰�ֲ���̬�����������뺯��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Howlet Li          ����
 *******************************************************************************/
void Verify_Hall_State(HALL_TypeDef *HALLx, stru_HallProcessDef *this)
{
    u8 temp_state1, temp_state;
    u8 t_cnt, t_i;
    u8 t_funcState;
    u32 t_CapCnt;
  

    //��ȡ��ǰhall״̬
    temp_state = ReadHallState(HALLx);

    if(this->bCloseLoopAngleFlg == 0)
    {   /* �����ǰHall ���໷����ģʽ */
        if(this->eMotorState == RUN)
        {
            if(this->stru_HallRepare.nFocusCommCnt < PWM_TIME_20MS * 2)
            {
                this->stru_HallRepare.nFocusCommCnt ++;
            }

            if(this->stru_HallRepare.nFocusCommCnt == PWM_TIME_20MS)
            {   /* 20mS��ⲻ��Hall�仯��ǿ�Ƴ�ʼ���Ƕ���Ϣ */
                this->stru_HallRepare.bHallState_org = temp_state;

                this->bHallState = this->bHallPossableTab[this->bHallState];

                if (this->bSpeedFIFO_Index < (HALL_SPEED_FIFO_SIZE - 1))
                {
                    this->bSpeedFIFO_Index++;
                }
                else
                {
                    this->bSpeedFIFO_Index = 0;
                }

                t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;

                this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;

                //
                this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(this);

                this->wHallPWMTimerCnt = 0;

                //�����Ƕȳ�ʼ��
                HALL_Init_Electrical_Angle(this);

            }
        }
    }

    if(this->stru_HallRepare.bNeed_DelayComm)
    {   /* Hall�޸�״̬�������Ҫ��ʱ���� */
        if(this->wHallPWMTimerCnt > this->stru_HallRepare.bDelay_timeCnt)
        {   /* ��ʱ����Ҫ�����ʱ�̣���ʼ��Hallֵ������ʼ���Ƕ���Ϣ��*/
            this->stru_HallRepare.bNeed_DelayComm = 0;

            this->bHallState = this->bHallPossableTab[this->bHallState];

            if (this->bSpeedFIFO_Index < (HALL_SPEED_FIFO_SIZE - 1))
            {
                this->bSpeedFIFO_Index++;
            }
            else
            {
                this->bSpeedFIFO_Index = 0;
            }

            t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;

            this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;

            this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(this);

            this->wHallPWMTimerCnt = 0;
            //��ʼ���Ƕ���Ϣ
            HALL_Init_Electrical_Angle(this);

        }
    }
    else
    {   //����Ҫ��ʱ����
        t_cnt = 0;
        if(temp_state != this->stru_HallRepare.bHallState_org)
        {   /* ��⵽Hall״̬�͵�ǰ״̬��һ��, ������Hall״̬ */
            for (t_i = 0; t_i < 4; t_i++)
            {
                temp_state1 = ReadHallState(HALLx); /* ������״̬ */

                if (temp_state == temp_state1)
                {
                    t_cnt++;
                }
            }

            if (t_cnt > 2)
            {
                //�ر�hall�ж�
                NVIC_DisableIRQ (HALL_IRQn);

                t_funcState = check_hall_state(temp_state, this); /* ��ѯ����״̬���� */
                if (t_funcState)
                {
                    if (this->bSpeedFIFO_Index < (HALL_SPEED_FIFO_SIZE - 1))
                    {
                        this->bSpeedFIFO_Index++;
                    }
                    else
                    {
                        this->bSpeedFIFO_Index = 0;
                    }

                    t_CapCnt = this->wHallPWMTimerCnt * ROTOR_SPEED_FACTOR_PER_UNIT;

                    this->wSensorPeriod[this->bSpeedFIFO_Index] = t_CapCnt;

                    this->wMotorSpeedAvgCnt = GetAvrgHallPeriod(this);
                    this->wHallPWMTimerCnt = 0;
                    if(this->bHallLearnFlg == 0)
                    {
                    HALL_Init_Electrical_Angle(this);
                    }
                }
                //ʹ��hall�ж�
                NVIC_EnableIRQ (HALL_IRQn);
            }
        }
    }

  
}

/*******************************************************************************
 �������ƣ�    hall_comm_VariableInit(void)
 ����������    HALL������ʼ��, ����Hall���ܵĻ����
 ���������    stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/15      V1.0           Howlet Li          ����
 *******************************************************************************/
void hall_comm_VariableInit(stru_HallProcessDef *this)
{
    u8 t_i, t_j;
    if (this->bHallType == 60)
    {
        for (t_i = 0; t_i < 8; t_i++)
        {
            this->bHallPossableTab[t_i] = 0;
            for (t_j = 0; t_j < 8; t_j++)
            {
                if (this->bHallCommTab[t_j] == t_i)
                {
                    if (t_j != 5)
                    {
                        this->bHallPossableTab[t_i] = this->bHallCommTab[t_j + 1];
                    }
                    else
                    {
                        this->bHallPossableTab[t_i] = this->bHallCommTab[0];
                    }
                    break;
                }
            }

            for (t_j = 0; t_j < 8; t_j++)
            {
                this->bHallPossableTab2[t_i] = 0;
                if (this->bHallCommTab[t_j] == t_i)
                {
                    if (t_j == 0)
                    {
                        this->bHallPossableTab2[t_i] = this->bHallCommTab[5];
                    }
                    else
                    {
                        this->bHallPossableTab2[t_i] = this->bHallCommTab[t_j - 1];
                    }
                    break;
                }
            }
        }
    }
    else
    {
        for (t_i = 1; t_i < 7; t_i++)
        {
            for (t_j = 0; t_j < 6; t_j++)
            {
                if (this->bHallCommTab[t_j] == t_i)
                {
                    if (t_j != 5)
                    {
                        this->bHallPossableTab[t_i] = this->bHallCommTab[t_j + 1];
                    }
                    else
                    {
                        this->bHallPossableTab[t_i] = this->bHallCommTab[0];
                    }
                    break;
                }
            }

            for (t_j = 0; t_j < 6; t_j++)
            {
                if (this->bHallCommTab[t_j] == t_i)
                {
                    if (t_j == 0)
                    {
                        this->bHallPossableTab2[t_i] = this->bHallCommTab[5];
                    }
                    else
                    {
                        this->bHallPossableTab2[t_i] = this->bHallCommTab[t_j - 1];
                    }
                    break;
                }
            }
        }
    }

    /* �ҵ�Hall 1λ�ã��ж�Hall �����������Ϊ��Hall�޸����� 171008 HL*/
    for (t_i = 0; t_i < 6; t_i++)
    {
        if(this->bHallCommTab[t_i] == 1)
        {
            break;
        }
    }

    if(t_i == 5)
    {
        if(this->bHallCommTab[0] == 3)
        {
            this->stru_HallRepare.bRepareDirect = 0;
        }
        else
        {
            this->stru_HallRepare.bRepareDirect = 1;
        }
    }
    else
    {
        if(this->bHallCommTab[t_i + 1] == 3)
        {
            this->stru_HallRepare.bRepareDirect = 0;
        }
        else
        {
            this->stru_HallRepare.bRepareDirect = 1;
        }
    }
}







/*******************************************************************************
 �������ƣ�    Hall_ElecAnglePWM_Process(void)
 ����������    Hall�Ƕ�PWM�ж����ڴ���
               HALL�쳣�����쳣����ʱ�������ۼӵ�Ƕ�ֵ
 ���������    stru_HallProcessDef *this : Hall����ṹ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/15      V1.0           Howlet Li          ����
 *******************************************************************************/
void Hall_ElecAnglePWM_Process(stru_HallProcessDef *this)
{
    this->wHallPWMTimerCnt++;       /* Hall PWMʱ������ */
    if(this->bFastAddElecAngleFlg)
    {
        u16 t_addAngleStep;

        t_addAngleStep = HALL_FAST_ADD_DPP;

        if(this->stru_HallRepare.bBrokenHallFlg)
        {
            t_addAngleStep = HALL_FAST_ADD_DPP / 2;
        }

        if(this->nEndElectricalAngle > 0)
        {
            if(this->nEndElectricalAngle > t_addAngleStep)
            {
                this->nEndElectricalAngle -= t_addAngleStep;
                this->nElectrical_Angle += t_addAngleStep;
            }
            else
            {
                this->bFastAddElecAngleFlg = 0;
            }
        }
        else
        {
            if(this->nEndElectricalAngle < -t_addAngleStep)
            {
                this->nEndElectricalAngle += t_addAngleStep;
                this->nElectrical_Angle -= t_addAngleStep;
            }
            else
            {
                this->bFastAddElecAngleFlg = 0;
            }
        }
    }

    //hall �Ƕȵ����ִ���
    if (this->bCloseLoopAngleFlg)
    {
        closeLoopAnglePLL(this);
    }
    else
    {
        reduceToqueAnglePll(this);
    }

}

/*******************************************************************************
 �������ƣ�    Hall_learn_Process(void)
 ����������    Hall�Ƕ���ѧϰ
 ���������    ��
 ���������    ��
 �� �� ֵ��    ��
 ����˵����
 �޸�����      �汾��          �޸���            �޸�����
 -----------------------------------------------------------------------------
 2015/11/15      V1.0           Howlet Li          ����
 *******************************************************************************/
void Hall_learn_Process(void)
{
    volatile u8 t_i;


    if (struHallProcess.bHallLearnFlg == 2)
    {
        struHallProcess.bHallLearnFlg = 1;
        struHallProcess.bHallLearnStateFlg = 0;
        struHallProcess.nLearnHallStep = 0;
        struHallProcess.nLearnHallTimeCnt = 0;
        struHallProcess.nElectrical_Angle = 0;
        struHallProcess.bHallCommTab[6] = 0;

        for(t_i = 0; t_i < 6; t_i++)
        {
            struHallProcess.bHallCommTab[t_i] = 0;
        }

        struFOC_CtrProc.bMC_RunFlg = 1;
        struFOC_CurrLoop.nQCur_Reference = App2CoreCurTrans(User2AppCurTrans(HALL_LEARN_CURRENT_SETTINT));;
        struFOC_CurrLoop.nDCur_Reference = 0;
    }

    if (struHallProcess.bHallLearnFlg == 1)
    {
        struFOC_CurrLoop.nQCur_Reference = App2CoreCurTrans(User2AppCurTrans(HALL_LEARN_CURRENT_SETTINT));; /* ѧϰ������ ֵԽ�����Խ�� */
        struFOC_CurrLoop.nDCur_Reference = 0;
        learn_hall_proc(&struHallProcess);
    }
}

/************************ (C) COPYRIGHT LINKO SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
