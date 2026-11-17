
/**
	????log???????????????§Ö???¦Ë???????¨¢?
	??????¨°??? Log_task::init()
	??????????????link???????»Ç
	link(0,(u8*)&aaa,sizeof(aaa));   ????aaa??id?0??
	??????????????????????????
	bool full_error=0;           //????????
	bool repeatId=0;             //?????id

	autolink ???????›¥???§Ö?Virtual_Motor??imu??rc??power??????????????›¥??


	Author:codeartist
	Date: 2024_12_06
*/
#include "log.h"
#include "usbd_cdc_core.h"
#include "usbd_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "crc.h"
#include "vision.h"

extern CDC_IF_Prop_TypeDef VCP_fops;
Log_task log_task;

#pragma pack(1)
struct LogStruct
{
	float vision_yaw;
	float vision_pitch;
	float imu_yaw;
	float imu_pitch;
	uint8_t beat;
	bool offlineFlag;
}logStruct;
#pragma pack()

Log_task::Log_task()
{
	memset(Tx_Struct.Tx_Buf, 0, sizeof(Tx_Struct.Tx_Buf));
	current_offset = 0;
}

void Log_task::init() // ???????log??????,????id??¦¶ 0-50
{
	 link(0,(u8*)&logStruct.vision_yaw,sizeof(logStruct.vision_yaw));
	 link(1,(u8*)&logStruct.vision_pitch,sizeof(logStruct.vision_pitch));
	 link(2,(u8*)&logStruct.imu_yaw,sizeof(logStruct.imu_yaw));
	 link(3,(u8*)&logStruct.imu_pitch,sizeof(logStruct.imu_pitch));
	 link(4,(u8*)&logStruct.beat,sizeof(logStruct.beat));
	
	 link(5,(u8*)&logStruct.offlineFlag,sizeof(logStruct.offlineFlag));
}

uint32_t aaa;

void Log_task::run()
{
	logStruct.vision_yaw = vision_RecvStruct.yaw;
	logStruct.vision_pitch = vision_RecvStruct.pitch;
	logStruct.imu_yaw = boardImu->FusionAngle.yaw;
	logStruct.imu_pitch = boardImu->FusionAngle.pitch;
	logStruct.beat = vision_RecvStruct.beat;
	#if USE_USB_VISION
	logStruct.offlineFlag = !vision.USB_offline_Flag;
	#endif
	#if USE_VISION
	logStruct.offlineFlag = !vision.offlineFlag;
	#endif
	
	Tx_Struct.time_stamp = imuTimeMatch.GetImu_VisionTime(boardImu->readTime, sizeof(vision_SendStruct));
	
	logData();
	Append_CRC16_Check_Sum((u8 *)&Tx_Struct, sizeof(Tx_Struct)); // ????CRC16§µ????
	
	#if USE_USB_VISION
	VCP_fops.pIf_DataTx((uint8_t *)&Tx_Struct, sizeof(Tx_Struct));
	#endif
	#if USE_VISION
	vision.SendMessage((uint8_t *)&Tx_Struct, sizeof(Tx_Struct));
	#endif
	
	aaa = sizeof(Tx_Struct);
}

void Log_task::logData()
{
	current_offset = 0; // ?????????

	for (int i = 0; i < data_list.size(); i++)
	{
		Tx_Struct.Tx_Buf[current_offset++] = 0x55;			  
		Tx_Struct.Tx_Buf[current_offset++] = data_list[i].id; 
		Tx_Struct.Tx_Buf[current_offset++] = data_list[i].size; 
		memcpy((u8 *)&Tx_Struct.Tx_Buf[current_offset], data_list[i].data, data_list[i].size);
		current_offset += data_list[i].size; // ???????????
	}

}

void Log_task::link(uint8_t id, const uint8_t *data, uint8_t size)
{
	DataItem new_item = {id, data, size};
	data_list.push_back(new_item);
}

