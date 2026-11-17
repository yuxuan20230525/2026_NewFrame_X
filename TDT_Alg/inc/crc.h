/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-09-21 11:41:15
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-01-25 17:13:45
 * @FilePath: \2025_NewFrame\2025_NewFram\Frame Base\Frame2025\TDT_Alg\inc\crc.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*****************************************************************************
File name: TDT_Alg\src\crc.h
Description: CRC校验算法
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _CRC_H
#define _CRC_H

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/
#include "stm32f4xx.h"

#define NULL 0 /* see <stddef.h> */

	extern const unsigned char CRC8_TAB[256];
	extern const uint16_t wCRC_Table[256];

	/**
	 * @addtogroup TDT_CRC
	 * @{
	 */

	/// 获取CRC8校验位
	unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
	/// 对数据进行CRC8校验
	unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
	/// 对数据追加CRC8数据位
	void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
	/// 获取CRC16校验位
	uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
	/// 对数据进行CRC16校验
	uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
	/// 对数据追加CRC16数据位
	void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

	/** @} */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif
