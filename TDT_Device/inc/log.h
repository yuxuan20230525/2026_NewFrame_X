#ifndef _LOG_H_
#define _LOG_H_

#include "board.h"
#include "time.h"
#include "task_virtual.h"
#include "motor.h"
#include "imu_task.h"
#include "bmi088.h"
#include "dbus.h"
//       ? »…     id    ? ? ß≥
struct DataItem
{
	uint8_t id = 0;			 //    ?  ID
	const uint8_t *data = nullptr; //    ?? ?
	uint8_t size = 0;		 //    ?? ß≥
};

class Log_task
{
public:
	Log_task();

	void run();

	void init();

	void link(uint8_t id, const uint8_t *data, uint8_t size);

	void logData();
private:
	myvector<DataItem, 80> data_list; //      ß“  ,   80    ¶À

#pragma pack(1)
	struct
	{
		uint8_t header = 0xA5; //   ?  ?  0XA5
		uint8_t type = 3;
		uint8_t Tx_Buf[60] = {0}; // 600   ???  ?     
		uint64_t time_stamp = 0;
		uint16_t CRC16CheckSum = 0; // ßµ    
	} Tx_Struct;

#pragma pack()
	size_t current_offset = 0; //   ??    
};

extern Log_task log_task;

#endif