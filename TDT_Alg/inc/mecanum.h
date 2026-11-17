/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <board.h>
#include "my_math.h"
#include "motor.h"
#include "filter.h"
#ifndef __MECANUM_H__
#define __MECANUM_H__

/**
 * @addtogroup TDT_ALG_MECANUM
 * @{
 */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.3f // 弧度转角度
#endif

/************************ 底盘参数 ****************************/
#define USE_MAI 0  //麦轮
#define USE_QUAN 1  //全向轮

/** 轮子半径(mm) */
#define RADIUS 70
/** 轮子周长(mm) */
#define PERIMETER 220
/** 轮距(mm) */
#define WHEELTRACK 380
/** 轴距(mm) */
#define WHEELBASE 380

/** 云台相对底盘中心的X轴偏移(mm)，上为正 */
#define ROTATE_X_OFFSET 0
/** 云台相对底盘中心的Y轴偏移(mm)，右为正 */
#define ROTATE_Y_OFFSET 0

//轮子与沿y轴速度的方向线的夹角
#define ANGLE 45

/**电机减速比 */
#define MOTOR_DECELE_RATIO_3508 (1.0f / 15.0f)

/** 单个3508电机最大转速，单位是rpm */
#define MAX_WHEEL_RPM 8500 // 8347rpm = 3500mm/s


/** 底盘最大平移速度，单位是mm/s */
#define MAX_CHASSIS_VX_SPEED 4000 // 8000rpm
/** @copybrief MAX_CHASSIS_VX_SPEED */
#define MAX_CHASSIS_VY_SPEED 4000
/** 底盘最大旋转速度，单位是度/秒 */
#define MAX_CHASSIS_VW_SPEED 1000 // 5000rpm

/** 底盘虚拟电机编码器精度，单位是(圈)^-1 */
#define MOTOR_ENCODER_ACCURACY 8192.0f


/**
 * @copydoc TDT_ALG_MECANUM
 */
class Mecanum
{
private:

	struct mecanum_position
	{
		float v_x_mm = 0;  /**< x方向速度(mm/s) */
		float v_y_mm = 0;  /**< y方向速度(mm/s) */
		float rate_deg = 0; /**< 旋转速率(°/s) */
		float position_x_mm = 0; /**< 当前位置x(mm) */
		float position_y_mm = 0; /**< 当前位置y(mm) */
		float angle_deg = 0; /**< 当前角度(°) */
	};
	

public:
	float rpm_ratio = 0; /**< 轮子转速比 */
	/** 麦轮速度参数 */
	struct mecanum_speed
	{
		float vx = 0; /**< 前进(+)，后退(-) */
		float vy = 0; /**< 向左(-)，向右(+) */
		float vw = 0; /**< 逆时针旋转(+) 顺时针旋转(-) */
	};

	/** 云台偏航角度参数 */
	struct mecanum_gyro
	{
		float yaw_gyro_angle = 0; /**< 偏航角度 */
		float yaw_gyro_rate = 0;  /**< 偏航角速度 */
	};

	struct mecanum
	{
		struct mecanum_speed speed; /**< 麦轮的速度参数 */
		struct mecanum_position position; /**< 麦轮的位置参数 */
		struct mecanum_gyro gyro; /**< 麦轮的偏航角与角速度 */
		float wheel_rpm[4] = {0}; /**< 轮子转速 */
	} mec;

	/**
	 * @brief Mecanum 构造器
	 * @param MotorType 给轮组赋值默认参数，mecanum.h修改宏定义
	 */
	Mecanum(Motor* chassismotor[4],float* YawMotorAngle); // 构造函数

	/**
	 * @brief 麦轮解算。F:前进 B:后退 L:左移 R:右移
	 * @param[in] speedDef : ccy=+vy(mm/s)  ccx=+vx(mm/s)  ccw=+vw(°/s) y方向速度 x方向速度 旋转速度
	 * @note  1=FR 2=FL 3=BL 4=BR  轮组命名为逆时针顺序，轮组安装为O型安装
	 */
	void mecanum_calculate(vec3f speedDef);

	/**
	 * @brief 获取麦轮底盘的里程，使用全局偏航角（需要传感器反馈的偏航角度）
	 * @param[in] wheel_fdb 麦轮电机反馈速度
	 * @warning 根据主动轮编码器计算里程，未进行全面测试！
	 */
	void mecanum_position_measure(Motor* chassismotor[4]);
	
	/**
	* @brief 底盘电机控制;
	 */
	void mecanum_ctrl(vec3f speedset,Motor* chassismotor[4],float YawMotorAngle);
	
	Pid Speedx_encoderPid;             //全局pid，接收编码器解算的速度反馈
	Pid Speedy_encoderPid;							//由于速度精度不高并且有滞后以及消耗电容，只在减速的时候开启
	float encoder_resultx,encoder_resulty;
	
	Lpf2p lpf_encoderx,lpf_encodery;

};

/** @} */

#endif // __MECANUM_H__
