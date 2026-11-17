/*
Author: skywalker&codeeartist
TIME:2024_3_30

*exp:
*   Motor1 = new MT_Motor(MF9025, CAN2, 0x203);  //����motor
*****************************************************
		Motor_Send(); ���ͺ���
		Motor_Receive(CanRxMsg *_CanRxMsg, u8 ifsetzero=0) ���պ���
		���඼��motorһ��
*****************************************************ʹ�õ�����÷���ֵ
		Motor1->Auto_ctrlSpeed(50);
		Motor1->Auto_ctrlPosition(30);
		
		�Լ�һ������
*/

#include "mt_motor.h"
#include "dbus.h"

#define MAX_NUM 10  //������󲻻���10��lkmotorͬʱ������
uint8_t vallid_mtNum=0;   //һ�������lk�������
MT_Motor* mtList[MAX_MTNUM]={0};

enum _CMD
{
	ERROR_CLEAR_FB = 0X9A,
  ERROR_CLEAR = 0X9B,
  MOTOR_CLOSE = 0X80,
  MOTOR_STOP = 0X81,
  MOTOR_RUN = 0X88,
  CTRL_IQ = 0XA1,    //��������ģʽ
  CTRL_SPEED = 0XA2, //�ٶ�ģʽ
	MUL_POSITION=0XA4, //��Ȧλ��ģʽ
	MUL_POSI_READ=0X90 //����Ȧλ��
};

// ע���ǽ��յ�id ���Ƿ���id ��0x240��ʼ����������������غϵĿ���֡��ҪС��
MT_Motor::MT_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _REC_ID):Virtual_Motor(motorType, _Canx,_REC_ID-0x100,_REC_ID)
{
	Send_Current_Agreement=&MT_Motor::SendCurrentAgreement;
	
	mtList[vallid_mtNum]=this;
	vallid_mtNum++;
}

void MT_Motor::Motor_Send()
{
		if(readcnt++>=5)	
		{
		 motorRead_totalAngle();
		 readcnt=0;
		 return;
		} //5֡��һ�ζ�Ȧ�Ƕȣ�һ�������Բ��ã�
		
		if(use_my_send)
			motorSend(canTxBuff);//�Զ��巢�ͺ���
		else
			motorSend();//���ͺ���
}

/**
 * @brief ����ٶȿ���
 * @param ���Ƶ��ٶ�  dps
*/
void MT_Motor::Auto_ctrlSpeed(float ctrlspeed)
{
	int32_t speed=ctrlspeed*100;
    /* ��������� */
    memset(canTxBuff,0,8);
    /* ������� */
    canTxBuff[0] = CTRL_SPEED;
    /* ������� */
	canTxBuff[4] = (uint8_t)((uint32_t)speed&0XFF);
	canTxBuff[5] = (uint8_t)((uint32_t)speed>>8&0XFF);
	canTxBuff[6] = (uint8_t)((uint32_t)speed>>16&0XFF);
	canTxBuff[7] = (uint8_t)((uint32_t)speed>>24&0XFF);
	
}

/**
 * @brief �����Ȧ�Ƕȿ���
 * @param ���ƵĽǶ� Ϊ 1degree/LSB
maxspeedΪ����ٶ� 1 dps/LSB ���Ծ����ٶȺ���
*/
void MT_Motor::Auto_ctrlPosition(float ctrlposition,float maxspeed)  //ʹ���������ʱ��Ҫע�����Ƿ��м��ٱ�
{
	if(maxspeed<0)
		maxspeed=10;
	int32_t position=ctrlposition*100;

  canTxBuff[0] = MUL_POSITION;
  /* ������� */
	canTxBuff[2] = (uint8_t)((uint32_t)maxspeed&0XFF);
	canTxBuff[3] = (uint8_t)((uint32_t)maxspeed>>8&0XFF);
	
	canTxBuff[4] = (uint8_t)((uint32_t)position&0XFF);
	canTxBuff[5] = (uint8_t)((uint32_t)position>>8&0XFF);
	canTxBuff[6] = (uint8_t)((uint32_t)position>>16&0XFF);
	canTxBuff[7] = (uint8_t)((uint32_t)position>>24&0XFF);
}



/**
 * @brief ��ȡ��Ȧ�Ƕ�
*/

void MT_Motor::motorRead_totalAngle(void)
{
    u8 canTxBuf[8];
	  memset(canTxBuf,0,8);
    /* ������� */
    canTxBuf[0] =MUL_POSI_READ;
		canTx(canTxBuf,motorInfo.can_x == 0 ? CAN1 : CAN2,motorInfo.Send_ID);
}


void MT_Motor::GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg)
{
	
	
	 /* ��������ID������Ӧ���� */
    switch(_CanRxMsg->Data[0])
    {
        case CTRL_IQ: 
        case CTRL_SPEED:
				case MUL_POSITION:
            Info->temperature = _CanRxMsg->Data[1];
            Info->trueCurrent  = (int16_t)((uint16_t)(_CanRxMsg->Data[3]<<8)|_CanRxMsg->Data[2]);    //ת�ص���
            Info->speed = (int16_t)((uint16_t)(_CanRxMsg->Data[5]<<8)|_CanRxMsg->Data[4]);
            break;
			case MUL_POSI_READ:
					Info->encoder = (uint16_t)((uint16_t)(_CanRxMsg->Data[5]<<8)|_CanRxMsg->Data[4]); //������λ��
				break;
        default:
            break;
    }	
}

void MT_Motor::SendCurrentAgreement(uint8_t canbuff[8],float canResult)
{
	memset(canbuff,0,8);
	canbuff[0] = CTRL_IQ;
	int16_t current =(int16_t)(canResult);
	canbuff[4] = current & 0xFF;
	canbuff[5] = (current >> 8) & 0xFF;
}


void MTList_Send()
{
	if(vallid_mtNum<=0)
		return;
	for(int i=0;i<vallid_mtNum;i++)
	{
		mtList[i]->Motor_Send();
	}
}

void MTList_Receive(u8 can_x,CanRxMsg *_CanRxMsg)
{
	if(vallid_mtNum<=0)
		return;
	
	for(int i=0;i<vallid_mtNum;i++)
	{
		mtList[i]->motorReceive(_CanRxMsg,can_x,0);
	}
}


