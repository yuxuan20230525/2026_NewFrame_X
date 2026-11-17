/*
Description: BMI088陀螺仪初始化以及数据读取
Author: liuskywalkerjskd
*/
#include "icm45686.h"
#include "filter.h"
#include "imu_task.h"
#include <stdio.h>
#include "inv_imu_driver.h"


#define ICM_USE_HARD_SPI
//#define ICM_USE_I2C

#define ICM_I2C_ADDR 0x69


#define UI_I2C  0 /**< identifies I2C interface. */
#define UI_SPI4 1 /**< identifies 4-wire SPI interface. */

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define	SPI_IMU_CS PAout(15)  //选中IMU	

#define INV_MSG(level,msg, ...) 	      printf("%d," msg "\r\n", __LINE__, ##__VA_ARGS__)

static inv_imu_device_t  imu_dev; /* Driver structure */

static uint8_t discard_accel_samples; /* Indicates how many accel samples should be discarded */
static uint8_t discard_gyro_samples; /* Indicates how many gyro samples should be discarded */


// 构造函数
Icm45686::Icm45686(SPI_TypeDef *spix, int baud) : Spi(spix, baud)
{
	this->gyroDpsFector = 4000 /32768.f;
	this->accValueFector =  32 /3276.8f;
}



#define SI_CHECK_RC(rc)                                                                            \
	do {                                                                                           \
		if (Icm45686::si_print_error_if_any(rc)) {                                                           \
			INV_MSG(INV_MSG_LEVEL_ERROR, "At %s (line %d)", __FILE__, __LINE__);                   \
			delayMs(100);                                                                   \
			return rc;                                                                             \
		}                                                                                          \
	} while (0)

/*
 * Error codes
 */
int Icm45686::si_print_error_if_any(int rc)
{
	if (rc != 0) {
		switch (rc) {
		case INV_IMU_ERROR:
			printf("Unspecified error (%d)", rc);
			break;
		case INV_IMU_ERROR_TRANSPORT:
			printf("Error occurred at transport level (%d)", rc);
			break;
		case INV_IMU_ERROR_TIMEOUT:
			printf("Action did not complete in the expected time window (%d)",rc);
			break;
		case INV_IMU_ERROR_BAD_ARG:
			printf("Invalid argument provided (%d)", rc);
			break;
		case INV_IMU_ERROR_EDMP_BUF_EMPTY:
			printf("EDMP buffer is empty (%d)", rc);
			break;
		default:
			printf("Unknown error (%d)", rc);
			break;
		}
	}

	return rc;
}
/*******************************************************************************
* 名    称： icm42688_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2024-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
int icm45686_read_regs(uint8_t reg, uint8_t* buf, uint32_t len)
{
#if defined(ICM_USE_HARD_SPI)
    reg |= 0x80;
    SPI_IMU_CS=0;
    /* 写入要读的寄存器地址 */
		SPI3_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    while(len)
	{
		*buf = SPI3_ReadWriteByte(0x00);
		len--;
		buf++;
	}
   SPI_IMU_CS=1;
#elif defined(ICM_USE_I2C)
	IICreadBytes(ICM_I2C_ADDR, reg, len, buf);
#endif
	return 0;
}

uint8_t io_write_reg(uint8_t reg, uint8_t value)
{
#if defined(ICM_USE_HARD_SPI)
    SPI_IMU_CS=0;
    /* 写入要读的寄存器地址 */
    /* 写入要读的寄存器地址 */
    SPI3_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    SPI3_ReadWriteByte(value);
    SPI_IMU_CS=1;
#elif defined(ICM_USE_I2C)
	IICwriteBytes(ICM42688_ADDRESS, reg, 1, &value);
#endif
    return 0;
}

int icm45686_write_regs(uint8_t reg, const uint8_t* buf, uint32_t len)
{
	int rc;

	for (uint32_t i = 0; i < len; i++) 
	{
		rc = io_write_reg(reg + i, buf[i]);
		if (rc)
			return rc;
	}
	return 0;
}


void Icm45686::init(void)
{
	Spi::init();
	if (0x00 == setup_imu(1,1,1))
	{
		acc.origin.data[0] =0;
		acc.origin.data[1] =0;
		acc.origin.data[2] =0;
		gyro.origin.data[0]=0;
		gyro.origin.data[1]=0;
		gyro.origin.data[2]=0;
		return;
	}
	for(int i = 0 ; i<3 ; ++i)
	{
		gyrolpf[i].SetCutoffFreq(500,50) ;
		acclpf[i].SetCutoffFreq(500,50);
	}

	// 初始化滤波器参数
	kfq = 10;
	kfr = 100;
	gyroq = 15; //大概取R的十分之一
	gyror = 120; //IMU陀螺仪数据方差约为这个
	ImuCalc::init(); // 初始化解算
	return;
}

void Icm45686::GetOriginData()
{
		float accgyroval[7];
	ignoretime++;
	float sqrResult_gyro[3];
	float avgResult_gyro[3];
	//读取加速度和陀螺仪的当前ADC
	bsp_IcmGetRawData(accgyroval, &accgyroval[3], &accgyroval[6]);
	if(ignoretime>8)
	{
		acc.origin.data[0] =  accgyroval[0];
		acc.origin.data[1] =  accgyroval[1];
		acc.origin.data[2] =  accgyroval[2];
		gyro.origin.data[0] =  accgyroval[3];
		gyro.origin.data[1] =  accgyroval[4];
		gyro.origin.data[2] =  accgyroval[5];
		watchtemp=  accgyroval[6];
	}
	
//	gyro.origin.data[0] = gyroxkf.KalmanFilter(gyrolpf[0].Apply(gyro.origin.data[0]), gyroq ,gyror);
//	gyro.origin.data[1] = gyroykf.KalmanFilter(gyrolpf[1].Apply(gyro.origin.data[1]), gyroq ,gyror);
//	gyro.origin.data[2] = gyrozkf.KalmanFilter(gyrolpf[2].Apply(gyro.origin.data[2]), gyroq ,gyror);
//	
//	// 陀螺仪低通+卡尔曼滤波，低频低速运动下表现良好，漂移最小，但高频剧烈运动下会有较高的误差
//	
//	
////	gyro.origin.data[2] = gyrolpf.Apply(gyro.origin.data[2]) ;
//	// 陀螺仪低通滤波，低频运动下表现良好，漂移比较小，高频运动下会有少许误差

//	//	gyro.origin.data[2] = gyrokf.KalmanFilter(gyro.origin.data[2],gyroq,gyror) ;
//	// 陀螺仪卡尔曼滤波，低速运动下表现良好，漂移比较小，高速运动下会有滞后和误差

//	acc.origin.data[0] = accxkf.KalmanFilter(acclpf[0].Apply(acc.origin.data[0]),kfq,kfr) ;
//	acc.origin.data[1] = accykf.KalmanFilter(acclpf[1].Apply(acc.origin.data[1]),kfq,kfr) ;
//	acc.origin.data[2] = acczkf.KalmanFilter(acclpf[2].Apply(acc.origin.data[2]),kfq,kfr) ;

}


/* Initializes IMU device and apply configuration. */
int Icm45686::setup_imu(int use_ln, int accel_en, int gyro_en)
{
	int                      rc     = 0;
	uint8_t                  whoami = 0;
	inv_imu_int_pin_config_t int_pin_config;
	inv_imu_int_state_t      int_config;
#if defined(ICM_USE_HARD_SPI)
	/* Init transport layer */
	imu_dev.transport.read_reg   = icm45686_read_regs;
	imu_dev.transport.write_reg  = icm45686_write_regs;
	imu_dev.transport.serif_type = UI_SPI4;
	imu_dev.transport.sleep_us   = delayUs;
	GPIO_InitTypeDef GPIO_Initure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOA时钟
	
	GPIO_Initure.GPIO_Pin = GPIO_Pin_15; // PB5~7复用功能输出
	GPIO_Initure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Initure.GPIO_OType = GPIO_OType_PP;	  // 推挽输出
	GPIO_Initure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_Initure.GPIO_PuPd = GPIO_PuPd_UP;		  // 上拉
	GPIO_Init(GPIOA, &GPIO_Initure);			  // 初始化

	
	
	SPI_IMU_CS=1;			                //SPI FLASH不选中
	//SPI1_Init();		   			        //初始化SPI
	//SPI3_SetSpeed(SPI_BAUDRATEPRESCALER_8); 
			
#endif
	/* Wait 3 ms to ensure device is properly supplied  */
	delayMs(100);

	/* In SPI, configure slew-rate to prevent bus corruption on DK-SMARTMOTION-REVG */
	if (imu_dev.transport.serif_type == UI_SPI3 || imu_dev.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
		rc |= inv_imu_write_reg(&imu_dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
		SI_CHECK_RC(rc);
		delayUs(2); /* Takes effect 1.5 us after the register is programmed */
	}

	/* Check whoami */
	rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
	SI_CHECK_RC(rc);
	if (whoami != INV_IMU_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
		return -1;
	}

	rc |= inv_imu_soft_reset(&imu_dev);
	SI_CHECK_RC(rc);

	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
	SI_CHECK_RC(rc);

	/* Interrupts configuration */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_UI_DRDY = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* Set FSR */
	rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G);
	rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS);
	SI_CHECK_RC(rc);

	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_6400_HZ);
	rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_6400_HZ);
	SI_CHECK_RC(rc);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8);
	rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8);
	SI_CHECK_RC(rc);

	/* Sensor registers are not available in ULP, so select RCOSC clock to use LP mode. */
	rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
	SI_CHECK_RC(rc);

	/* Set power modes */
	if (use_ln) {
		if (accel_en)
			rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
		if (gyro_en)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
	} else {
		if (accel_en)
			rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
		if (gyro_en)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LP);
	}

	/* Discard N samples at 50Hz to ignore samples at sensor enabling time */
	if (accel_en)
		discard_accel_samples = (ACC_STARTUP_TIME_US / 20000) + 1;
	if (gyro_en)
		discard_gyro_samples = (GYR_STARTUP_TIME_US / 20000) + 1;

	SI_CHECK_RC(rc);

	return rc;
}
int Icm45686::bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc)
{
	int rc = 0;
	inv_imu_sensor_data_t d;
	
	rc |= inv_imu_get_register_data(&imu_dev, &d);
	SI_CHECK_RC(rc);
	
	accel_mg[0]=(float)(d.accel_data[0]);
	accel_mg[1]=(float)(d.accel_data[1]);
	accel_mg[2]=(float)(d.accel_data[2]);
	
	gyro_dps[0] = (float)d.gyro_data[0];
	gyro_dps[1] = (float)d.gyro_data[1];
	gyro_dps[2] = (float)d.gyro_data[2];
	
//	accel_mg[0] = (float)((d.accel_data[0] * 4 /* mg */) / 3276.8);
//	accel_mg[1] = (float)((d.accel_data[1] * 4 /* mg */) / 3276.8);
//	accel_mg[2] = (float)((d.accel_data[2] * 4 /* mg */) / 3276.8);
//	gyro_dps[0] = (float)((d.gyro_data[0] * 1000 /* dps */) / 32768.0);
//	gyro_dps[1] = (float)((d.gyro_data[1] * 1000 /* dps */) / 32768.0);
//	gyro_dps[2] = (float)((d.gyro_data[2] * 1000 /* dps */) / 32768.0);
	*temp_degc  = (float)(25 + (d.temp_data / 128.0));
	return 0;
}
	
u8 SPI3_ReadWriteByte(u8 TxData)
{
   for (int i = 0; i < 0xfffffff && SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET; i++)
		;							// 绛夊緟鍙戦�佸尯绌?
	SPI_I2S_SendData(SPI3,TxData); // 閫氳繃澶栬SPIx鍙戦�佷竴涓猙yte  鏁版嵁
	for (int i = 0; i < 0xfffffff && SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET; i++)
		;							  // 绛夊緟鎺ユ敹瀹屼竴涓猙yte
	return SPI_I2S_ReceiveData(SPI3); // 杩斿洖閫氳繃SPIx鏈�杩戞帴鏀剁殑鏁版嵁
}
