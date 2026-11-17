/*
Author: skywalker&codeeartist
TIME:2024_3_30

*exp:
*   Motor1 = new DM_Motor(MF9025, CAN2, 0x203);  //创建Virtual_Motor
*****************************************************
		Motor_Send(); 发送函数
		Motor_Receive(CanRxMsg *_CanRxMsg, u8 ifsetzero=0) 接收函数
		其余都和Virtual_Motor一样
*****************************************************使用电机内置反馈值
		Motor1->Auto_ctrlSpeed(50);
		Motor1->Auto_ctrlPosition(30);

		以及一堆命令
*/

#include "dm_Motor.h"
#include "dbus.h"


uint8_t valid_dmmotorNum = 0; // 一共定义的lk电机对象
DM_Motor *dmList[MAX_DMNUM]={0}; // 电机列表

int float_to_uint(float x, float x_min, float x_max, int bits)
  {
      /// Converts a float to an unsigned int, given range and number of bits ///
      float span = x_max - x_min;
      float offset = x_min;
      return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
  }

  float uint_to_float(int x_int, float x_min, float x_max, int bits)
  {
      /// converts unsigned int to float, given range and number of bits ///
      float span = x_max - x_min;
      float offset = x_min;
      return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
  }

DM_Motor::DM_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t SendId,uint32_t REC_id) : Virtual_Motor(motorType, _Canx,  SendId,REC_id)
{
	Send_Current_Agreement = &DM_Motor::SendCurrentAgreement;//发送函数传递
	motor_Calculate.GetOriginData=&GetOriginData; //接受函数传递

	dmList[valid_dmmotorNum] = this;
	valid_dmmotorNum++;
}

void DM_Motor::Motor_Send()
{
	if (use_my_send)
		motorSend(canTxBuff); // 自定义发送函数
	else
		motorSend(); // 发送函数
}


void DM_Motor::GetOriginData(CanInfo *Info, CanRxMsg *_CanRxMsg)
{
    Info->errCode = (_CanRxMsg->Data[0]) >> 4;
    Info->encoder =  uint_to_float(int((_CanRxMsg->Data[1] << 8) | _CanRxMsg->Data[2]), P_MIN, P_MAX, 16);         // (-12.5,12.5)
    Info->speed = uint_to_float(int((_CanRxMsg->Data[3] << 4) | (_CanRxMsg->Data[4] >> 4)), V_MIN, V_MAX, 12);         // (-45.0,45.0)
    Info->trueCurrent = uint_to_float(int(((_CanRxMsg->Data[4] & 0xF) << 8) | _CanRxMsg->Data[5]), T_MIN, T_MAX, 12); // (-18.0,18.0)
    Info->temperature = (float)(_CanRxMsg->Data[7]);
}

/// @brief MIT控制模式
/// @param p_des,v_des,t_ff：范围可由调试助手进行设定，
/// @param kp,kd：Kp的范围为[0,500]，Kd的范围为[0,5]。
/// @note  对位置进行控制时， kd 不能赋 0， 否则会造成电机震荡， 甚至失控。
void DM_Motor::mitCtrl(float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(p_des, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(v_des, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);	
    tor_tmp = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    
    canTxBuff[0] = (pos_tmp >> 8);
    canTxBuff[1] = pos_tmp;
    canTxBuff[2] = (vel_tmp >> 4);
    canTxBuff[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    canTxBuff[4] = kp_tmp;
    canTxBuff[5] = (kd_tmp >> 4);
    canTxBuff[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    canTxBuff[7] = tor_tmp;
    
}

/// @brief 进入电机
void DM_Motor::motorEnable()
{
    u8 txData[8]={0};
    for (uint8_t i = 0; i < 7; i++)
    {
       txData[i] = 0xff;
    }
    txData[7] = 0xfc;
    canTx(txData, motorInfo.can_x == 0 ? CAN1 : CAN2, motorInfo.Send_ID);
    
}

/// @brief 退出电机
void DM_Motor::motorDisable()
{
    u8 txData[8]={0};
    for (uint8_t i = 0; i < 7; i++)
    {
        txData[i] = 0xff;
    }
    txData[7] = 0xfd;
    canTx(txData, motorInfo.can_x == 0 ? CAN1 : CAN2, motorInfo.Send_ID);
    
}


/// @brief 保存位置零点
void DM_Motor::saveZero()
{
    u8 txData[8]={0};
    for (uint8_t i = 0; i < 7; i++)
    {
        txData[i] = 0xff;
    }
    txData[7] = 0xfe;
    
    canTx(txData,motorInfo.can_x == 0 ? CAN1 : CAN2, motorInfo.Send_ID);
}


void DM_Motor::SendCurrentAgreement(uint8_t canbuff[8], float canResult)
{
	memset(canbuff, 0, 8);
	
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  pos_tmp = float_to_uint(0, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(0, V_MIN, V_MAX, 12);
  kp_tmp = float_to_uint(0, KP_MIN, KP_MAX, 12);
  kd_tmp = float_to_uint(0, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(canResult, T_MIN, T_MAX, 12);

    
   canbuff[0] = (pos_tmp >> 8);
   canbuff[1] = pos_tmp;
   canbuff[2] = (vel_tmp >> 4);
   canbuff[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
   canbuff[4] = kp_tmp;
   canbuff[5] = (kd_tmp >> 4);
   canbuff[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
   canbuff[7] = tor_tmp;
	
}


void DMList_Send()
{
	if (valid_dmmotorNum <= 0)
		return;
	for (int i = 0; i < valid_dmmotorNum; i++)
	{
		dmList[i]->Motor_Send();
	}
}

void DMList_Receive(u8 can_x, CanRxMsg *_CanRxMsg)
{
	if (valid_dmmotorNum <= 0)
		return;

	for (int i = 0; i < valid_dmmotorNum; i++)
	{
		dmList[i]->motorReceive(_CanRxMsg,can_x,0);
	}
}
