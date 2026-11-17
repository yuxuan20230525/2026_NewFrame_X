#ifndef _ICM45686_
#define _ICM45686_

#include "spi.h"
#include "board.h"
#include "imu.h"
#include "filter.h"


class Icm45686 : public Spi, public ImuCalc
{
private:
public:
	uint16_t Error_Status = 0;
	Icm45686(SPI_TypeDef *spix, int baud);
	
	// µÍÍ¨ÂË²¨ÂË³ý¸ßÆµÔëÉù
	Lpf2p gyrolpf[3] ;
	Lpf2p acclpf[3] ;

	Kf accxkf, accykf, acczkf;
	Kf gyroxkf, gyroykf, gyrozkf;
	int ignoretime=0;
	float kfq=0, kfr=0;
	float gyroq=0, gyror=0;
	float watchkf=0;
	virtual void init(void) override;
	void GetOriginData(void) override;
	int si_print_error_if_any(int rc);
	int setup_imu(int use_ln, int accel_en, int gyro_en);
	int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);
	ImuCalc *imuview = nullptr;
};

static int icm45686_read_regs(uint8_t reg, uint8_t* buf, uint32_t len);
static uint8_t io_write_reg(uint8_t reg, uint8_t value);
static int icm45686_write_regs(uint8_t reg, const uint8_t* buf, uint32_t len);
u8 SPI3_ReadWriteByte(u8 TxData);

extern float watchtemp;
#endif

