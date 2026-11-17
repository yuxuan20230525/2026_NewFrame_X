/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-01-25 19:09:02
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-01-25 19:37:32
 * @FilePath: \2025_NewFrame\2025_NewFram\Frame Base\Frame2025\TDT_Device\inc\scha634_03.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SCHA634_03_H
#define __SCHA634_03_H

#include "board.h"
#include "imu.h"
#include "spi.h"

class Scha634_03 : public ImuCalc
{
public:
	Spi *spiDUE = nullptr;
	Spi *spiUNO = nullptr;
	union communFrame
	{
		uint8_t u8data[4];
		uint16_t u16data[2];
		uint32_t u32data;
		struct
		{
			uint32_t Checksum : 8;	   // Checksum
			uint32_t data : 16;		   // Return status
			uint32_t returnStatus : 2; // Return status
			uint32_t RW_Address : 5;   // Register address
			uint32_t RW_Operation : 1; // R=0 W=1
		} infoData;
	} sendFrame, recvFrame;

	Scha634_03(SPI_TypeDef *spix, int baud, CsPin pinDUE, CsPin pinUNO);
	

	void init() override;
	virtual void GetOriginData() override;
	//	void get6AxisRawData(void);
	//	void get3AxisAccRawData(void);
	//	void get3AxisGyroRawData(void);
	//	void getTempRawData(void);
	//	void GetOriginData()override;

	uint32_t spi_send_byte32(uint32_t data);
	void sendRecvMsg();

	void DUE_changeData();
	void UNO_changeData();

	void commInit();
	int startup_attempt = 0;
	int crc8WrongNum = 0;
	ImuCalc *imuView = nullptr;
};

#endif
