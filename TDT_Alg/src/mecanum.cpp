#include "mecanum.h"

#define USE_ENCODER_PID  0 //是否使用编码器速度环
/**
 * @ingroup TDT_ALG
 * @defgroup TDT_ALG_MECANUM 麦轮解算
 * @brief 该类提供了大疆开源的麦轮算法库的调用，能将车需要运动的速度换算成四个麦轮的速度，还提供了将麦轮作为里程计获取车的里程的功能。
 * @details 本类原理和调用都不难，主要是参数和宏定义都比较繁琐
 */

PidParam Speed_encoderPidParam = {0.5,0,0,1,2000};

Mecanum::Mecanum(Motor* chassismotor[4],float* YawMotorAngle)
{
	
	lpf_encoderx.SetCutoffFreq(500,5);
	lpf_encodery.SetCutoffFreq(500,5);
	
	Speedx_encoderPid.setPlanNum(1);
	Speedx_encoderPid.paramPtr=&Speed_encoderPidParam;
	Speedx_encoderPid.fbValuePtr[0]=&mec.position.v_x_mm;
	
	Speedy_encoderPid.setPlanNum(1);
	Speedy_encoderPid.paramPtr=&Speed_encoderPidParam;
	Speedy_encoderPid.fbValuePtr[0]=&mec.position.v_y_mm;
	
	Speed_encoderPidParam.positiveFBFlag=0;
	
	
}

void Mecanum::mecanum_ctrl(vec3f speedset,Motor* chassismotor[4],float YawMotorAngle)
{
	float mul_differangle=0,mul_differangle_f=0;

	if(YawMotorAngle)
		mul_differangle = normalizeAngle(YawMotorAngle);
	else
		mul_differangle=0;
	
	mul_differangle_f = mul_differangle/360.0f*2*PI;
	
	
	mecanum_position_measure(chassismotor);
	
	vec3f chassis_setspeed;
	chassis_setspeed.data[0] = (+speedset.data[0] * my_cos_rad(mul_differangle_f) -speedset.data[1] * my_sin_rad(mul_differangle_f));
	chassis_setspeed.data[1] = (speedset.data[0] * my_sin_rad(mul_differangle_f) +speedset.data[1] * my_cos_rad(mul_differangle_f));

	#if USE_ENCODER_PID
		if( chassismotor[0]->canInfo.lostFlag==1|| chassismotor[1]->canInfo.lostFlag==1 || chassismotor[2]->canInfo.lostFlag==1 || chassismotor[3]->canInfo.lostFlag==1)                    
		{
			encoder_resultx=0;
			encoder_resulty=0;		
		}
		encoder_resultx=Speedx_encoderPid.Calculate(chassis_setspeed.data[1]);
		encoder_resulty=Speedy_encoderPid.Calculate(chassis_setspeed.data[0]);
	#else
		encoder_resultx=0;
		encoder_resulty=0;		
	#endif
		
	vec3f chassis_inputspeed;
	
	chassis_inputspeed.data[0]=chassis_setspeed.data[0]+encoder_resulty;
	chassis_inputspeed.data[1]=chassis_setspeed.data[1]+encoder_resultx;
	chassis_inputspeed.data[2] = speedset.data[2];

	mecanum_calculate(chassis_inputspeed);
	
	for(int i=0 ;i<4;i++)
	{
		chassismotor[i]->ctrlSpeed(mec.wheel_rpm[i]);
	}
	
}


void Mecanum::mecanum_calculate(vec3f speedDef)
{
		mec.speed.vx = speedDef.data[1]; // 输入 mm/s
    mec.speed.vy = speedDef.data[0];
    mec.speed.vw = speedDef.data[2];                                  // deg/s
    LIMIT(mec.speed.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); // mm/s
    LIMIT(mec.speed.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); // mm/s
    LIMIT(mec.speed.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);

    float rotate_ratio_fr;
    float rotate_ratio_fl;
    float rotate_ratio_bl;
    float rotate_ratio_br; // 每个轮的旋转系数，可修改旋转中心
    float wheel_rpm_ratio;
    float wheel_rpm[4];
    float max = 0;

#if USE_MAI
    rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_X_OFFSET - ROTATE_Y_OFFSET) / RADIAN_COEF;
    rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_X_OFFSET + ROTATE_Y_OFFSET) / RADIAN_COEF;
    rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_X_OFFSET + ROTATE_Y_OFFSET) / RADIAN_COEF;
    rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_X_OFFSET - ROTATE_Y_OFFSET) / RADIAN_COEF;
    wheel_rpm_ratio = 60.0f / (PERIMETER * MOTOR_DECELE_RATIO_3508); // 线速度mm/s转角速度rad/min

    wheel_rpm[0] =  (-mec.speed.vx + mec.speed.vy - mec.speed.vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[1] =  (-mec.speed.vx - mec.speed.vy - mec.speed.vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[2] =  (mec.speed.vx - mec.speed.vy - mec.speed.vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] =  (mec.speed.vx + mec.speed.vy - mec.speed.vw * rotate_ratio_br) * wheel_rpm_ratio;
#endif
#if USE_QUAN
    rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_X_OFFSET - ROTATE_Y_OFFSET) / RADIAN_COEF;
    rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_X_OFFSET + ROTATE_Y_OFFSET) / RADIAN_COEF;
    rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_X_OFFSET + ROTATE_Y_OFFSET) / RADIAN_COEF;
    rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_X_OFFSET - ROTATE_Y_OFFSET) / RADIAN_COEF;
    wheel_rpm_ratio = 60.0f / (PERIMETER * MOTOR_DECELE_RATIO_3508); // 线速度mm/s转角速度rad/min
    rpm_ratio = PERIMETER * MOTOR_DECELE_RATIO_3508 / (4 * 60.0f);
    wheel_rpm[0] =  0.707 * (-mec.speed.vx + mec.speed.vy - mec.speed.vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[1] =  0.707 * (-mec.speed.vx - mec.speed.vy - mec.speed.vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[2] =  0.707 * (mec.speed.vx - mec.speed.vy - mec.speed.vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] =  0.707 * (mec.speed.vx + mec.speed.vy - mec.speed.vw * rotate_ratio_br) * wheel_rpm_ratio;
#endif

    // 找到最大轮速
    for (uint8_t i = 0; i < 4; i++)
    {
        if (ABS(wheel_rpm[i]) > max)
        {
            max = ABS(wheel_rpm[i]);
        }
    }

    // 最大旋转速度超过设定最大旋转速度，平均降速
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_rpm[i] *= rate;
        }
    }
    memcpy(mec.wheel_rpm, wheel_rpm, 4 * sizeof(float));
}

void Mecanum::mecanum_position_measure(Motor* chassismotor[4])
{
    float rotate_ratio_fr;
    float rotate_ratio_fl;
    float rotate_ratio_bl;
    float rotate_ratio_br;
    float ecd_ratio;
    double mecanum_angle;
    static double last_d_x, last_d_y, last_d_w;
    double d_x, d_y, d_w, diff_d_x, diff_d_y, diff_d_w;
    double position_x, position_y, angle_w;
    double v_x, v_y, v_w;
	

    rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_X_OFFSET - ROTATE_Y_OFFSET);
    rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_X_OFFSET + ROTATE_Y_OFFSET);
    rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_X_OFFSET + ROTATE_Y_OFFSET);
    rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_X_OFFSET - ROTATE_Y_OFFSET);

    ecd_ratio = PERIMETER * MOTOR_DECELE_RATIO_3508 / (4 * MOTOR_ENCODER_ACCURACY);

    last_d_x = d_x;
    last_d_y = d_y;
    last_d_w = d_w;
		
    d_x = ecd_ratio * (-chassismotor[0]->canInfo.encoder + chassismotor[1]->canInfo.encoder + chassismotor[2]->canInfo.encoder - chassismotor[3]->canInfo.encoder);
    d_y = -ecd_ratio * (-chassismotor[0]->canInfo.encoder - chassismotor[1]->canInfo.encoder + chassismotor[2]->canInfo.encoder + chassismotor[3]->canInfo.encoder);
    d_w = ecd_ratio * (-chassismotor[0]->canInfo.encoder / rotate_ratio_fr - chassismotor[1]->canInfo.encoder / rotate_ratio_fl - chassismotor[2]->canInfo.encoder / rotate_ratio_bl - chassismotor[3]->canInfo.encoder / rotate_ratio_br);

    diff_d_x = d_x - last_d_x;
    diff_d_y = d_y - last_d_y;
    diff_d_w = d_w - last_d_w;

    /* use glb_chassis gyro angle data */
    mecanum_angle = mec.gyro.yaw_gyro_angle / RADIAN_COEF;

    position_x += diff_d_x * cos(mecanum_angle) - diff_d_y * sin(mecanum_angle);
    position_y += diff_d_x * sin(mecanum_angle) + diff_d_y * cos(mecanum_angle);

    angle_w += diff_d_w;

    mec.position.position_x_mm = position_x;        // mm
    mec.position.position_y_mm = position_y;        // mm
    mec.position.angle_deg = angle_w * RADIAN_COEF; // degree

    v_y = -2*rpm_ratio * (-chassismotor[0]->canInfo.speed + chassismotor[1]->canInfo.speed + chassismotor[2]->canInfo.speed - chassismotor[3]->canInfo.speed)*my_cos_angle(ANGLE);
    v_x = 2*rpm_ratio * (-chassismotor[0]->canInfo.speed - chassismotor[1]->canInfo.speed + chassismotor[2]->canInfo.speed + chassismotor[3]->canInfo.speed)*my_sin_angle(ANGLE);
    v_w = rpm_ratio * (-chassismotor[0]->canInfo.speed / rotate_ratio_fr - chassismotor[1]->canInfo.speed / rotate_ratio_fl - chassismotor[2]->canInfo.speed / rotate_ratio_bl - chassismotor[3]->canInfo.speed / rotate_ratio_br);
		
    mec.position.v_x_mm = v_x;                 // mm/s
    mec.position.v_y_mm = v_y;                 // mm/s
    mec.position.rate_deg = v_w * RADIAN_COEF; // degree/s
		
	if(ABS(mec.position.v_x_mm)<60)
	mec.position.v_x_mm=0;
	if((ABS(mec.position.v_y_mm)<60))
	mec.position.v_y_mm=0;
		
	mec.position.v_x_mm=lpf_encoderx.Apply(mec.position.v_x_mm);
	mec.position.v_y_mm=lpf_encodery.Apply(mec.position.v_y_mm);
}
