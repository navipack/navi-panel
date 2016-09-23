/**
******************************************************************************
* @file    sensor_update.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   IMU 传感器解算
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "sensor_update.h"
#include "fir_filter.h"
#include "math_lib.h"
#include "global_defines.h"
#include "inv_mpu_driver.h"
#include "motion_control.h"
#include <math.h>
#include "tim_user.h"


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
#define FILTER_400_50_TAP       			4                     
#define FILTER_2K_100_TAP       			4 

#define LIMIT(x, m)							if((x) > (m)) (x) = (m); else if((x) < (-m)) (x) = -(m);	//m为正数
#define SENSOR_UPDATE_DT					(4096/IMU_UPDATE_FREQ)

#ifdef _DEBUG
#define SENSOR_DEBUG
#endif

// STOP
#define	PITCH_ANGLE_TOO_LARGE				(1)		//俯仰角过大，表现为爬坡
#define ROLL_ANGLE_TOO_LARGE				(2)		//横滚角过大，表现为爬坡

// MOVE BACK
#define	ACC_FORWARD_TOO_LARGE				(3)		//前向加速度过大，表现为前后碰撞
#define ACC_LEFT_TOO_LARGE					(4)		//左加速度过大，表现为左右碰撞
#define ACC_RIGHT_TOO_LARGE					(5)		//左加速度过大，表现为左右碰撞

// DO NOTHING
#define	ACC_BACKWARD_TOO_LARGE				(6)		//后向加速度过大，表现为前后碰撞

#define ACC_Z_TOO_LARGE						(7)		//垂直加速度过大，表现为抬起
#define LIGHT_COLLISION						(8)		//挂住空转，表现为微小碰撞，但小车主动轮被抬起空转

#define REST_ANGLE_THRESHOLD                ((u32)DEGREE_QCALC*CalibrationFactor)
#define PITCH_ANGLE_THRESHOLD				(10 * DEGREE_QCALC)	 //10度
#define ROLL_ANGLE_THRESHOLD				(10 * DEGREE_QCALC)	 //10度
#define ACC_X_THRESHOLD						(0.6 * DEGREE_QCALC) //0.3g
#define ACC_Y_THRESHOLD						(0.6 * DEGREE_QCALC) //0.3g
#define ACC_Z_THRESHOLD						(-0.5 * DEGREE_QCALC) //0.3g
#define LIGHT_COLLISION_THRESHOLD			(0.5 * DEGREE_QCALC) //0.5m
/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/
IMUFilterInfo	g_SensorSystemStatus = {0};

u32 g_IMUSystermStatus = 0;
/*
*********************************************************************************************************
*                                            LOCAL VARIABLES
*********************************************************************************************************
*/

//FIR滤波器参数
const s32 c_fFactor_400_50_4[FILTER_400_50_TAP] = {(s32)(0.223*DEGREE_QCALC), (s32)(0.277*DEGREE_QCALC), (s32)(0.277*DEGREE_QCALC), (s32)(0.223*DEGREE_QCALC)};
const s32 c_fFactor_2K_100_4[FILTER_2K_100_TAP] = {(s32)(0.2459*DEGREE_QCALC), (s32)(0.2541*DEGREE_QCALC), (s32)(0.2541*DEGREE_QCALC), (s32)(0.2459*DEGREE_QCALC)};

//accel 
static s32 s_fAccxFilter_Delay[FILTER_400_50_TAP];
static s32 s_fAccyFilter_Delay[FILTER_400_50_TAP];
static s32 s_fAcczFilter_Delay[FILTER_400_50_TAP];

static FirFilterIntDef s_tAccxFilter = {s_fAccxFilter_Delay, c_fFactor_400_50_4, FILTER_400_50_TAP};
static FirFilterIntDef s_tAccyFilter = {s_fAccyFilter_Delay, c_fFactor_400_50_4, FILTER_400_50_TAP};
static FirFilterIntDef s_tAcczFilter = {s_fAcczFilter_Delay, c_fFactor_400_50_4, FILTER_400_50_TAP};

//gyro
static s32 s_fGyroxFilter_Delay[FILTER_2K_100_TAP];
static s32 s_fGyroyFilter_Delay[FILTER_2K_100_TAP];
static s32 s_fGyrozFilter_Delay[FILTER_2K_100_TAP];

static FirFilterIntDef s_tGyroxFilter = {s_fGyroxFilter_Delay, c_fFactor_2K_100_4, FILTER_2K_100_TAP};
static FirFilterIntDef s_tGyroyFilter = {s_fGyroyFilter_Delay, c_fFactor_2K_100_4, FILTER_2K_100_TAP};
static FirFilterIntDef s_tGyrozFilter = {s_fGyrozFilter_Delay, c_fFactor_2K_100_4, FILTER_2K_100_TAP};

static IMUFilterInfo	s_IMU_FilterProcessInfo = {0};

		
#ifdef SENSOR_DEBUG
	 IMUDebugInfo 		g_SensorDebug = {0.0};
	 IMURawDebugInfo	g_SensorRawDebug = {0.0};
     s32 TestG[6];
     u8 g_MaxDebug[20];
#endif

/*
*********************************************************************************************************
*                                            STATIC FUNCTIONS
*********************************************************************************************************
*/


/*******************************************************************************
* Function Name  : SensorUpdate_DoubleAxis
* Description    :
* Input          : None
* Output         : None
* Return		 : None
* Notes          : None
*******************************************************************************/
void SensorUpdate_DoubleAxis(IMURawInfo SensorDataInput, IMUFilterInfo *SensorDataOutput)
{
	static bool is_first_update = true;
	s32 fused_kg = 20;
	s32 sin_pitch_angle = GetSin(s_IMU_FilterProcessInfo.fused_pitch_angle);
	s32 cos_pitch_angle = GetCos(s_IMU_FilterProcessInfo.fused_pitch_angle);
	s32 sin_roll_angle = GetSin(s_IMU_FilterProcessInfo.fused_roll_angle);
	s32 cos_roll_angle = GetCos(s_IMU_FilterProcessInfo.fused_roll_angle);
	
	LIMIT(SensorDataInput.accel_pitch, DEGREE_QCALC);
	LIMIT(SensorDataInput.accel_roll, DEGREE_QCALC);
	
	s_IMU_FilterProcessInfo.static_yaw_angle = 0;
	s_IMU_FilterProcessInfo.static_pitch_angle = GetAsin(SensorDataInput.accel_pitch);
	s_IMU_FilterProcessInfo.static_roll_angle = GetAsin(SensorDataInput.accel_roll);
	
	if(is_first_update)
	{
		is_first_update = false;
		s_IMU_FilterProcessInfo.fused_pitch_angle = s_IMU_FilterProcessInfo.static_pitch_angle;
		s_IMU_FilterProcessInfo.fused_roll_angle = s_IMU_FilterProcessInfo.static_roll_angle;
		//s_IMU_FilterProcessInfo.fused_yaw_angle = s_IMU_FilterProcessInfo.static_yaw_angle;
	}
	
	//calculate angle rate
	s_IMU_FilterProcessInfo.pitch_anglerate = (cos_roll_angle * (SensorDataInput.gyro_x / 4) + \
											   sin_roll_angle * (SensorDataInput.gyro_z / 4)) / 1024;
	
    if(cos_pitch_angle == 0) cos_pitch_angle = 1;
	s_IMU_FilterProcessInfo.roll_anglerate = ((sin_pitch_angle * sin_roll_angle) / cos_pitch_angle) * (SensorDataInput.gyro_x / 4) / 1024 + \
											   SensorDataInput.gyro_y - ((cos_roll_angle * sin_pitch_angle) / \
											   cos_pitch_angle) * (SensorDataInput.gyro_z / 4) / 1024;

//	s_IMU_FilterProcessInfo.yaw_anglerate = (SensorDataInput.gyro_z/4 * cos_roll_angle/cos_pitch_angle)*4 - 
//                                            (SensorDataInput.gyro_x/4 * sin_roll_angle/cos_pitch_angle)*4;
    s_IMU_FilterProcessInfo.yaw_anglerate = SensorDataInput.gyro_z;

    //互补运算
	//calculate fused angle
	s_IMU_FilterProcessInfo.fused_pitch_angle = ((4096 - fused_kg) * (s_IMU_FilterProcessInfo.fused_pitch_angle + s_IMU_FilterProcessInfo.pitch_anglerate * \
												SENSOR_UPDATE_DT / 4096) + fused_kg * (s_IMU_FilterProcessInfo.static_pitch_angle)) / 4096;
	
	s_IMU_FilterProcessInfo.fused_roll_angle = ((4096 - fused_kg) * (s_IMU_FilterProcessInfo.fused_roll_angle + s_IMU_FilterProcessInfo.roll_anglerate * \
												SENSOR_UPDATE_DT / 4096) + fused_kg * (s_IMU_FilterProcessInfo.static_roll_angle)) / 4096;
	
	//s_IMU_FilterProcessInfo.fused_yaw_angle = SensorDataOutput->static_yaw_angle;

	//output 
	SensorDataOutput->static_pitch_angle = s_IMU_FilterProcessInfo.static_pitch_angle;
	SensorDataOutput->static_roll_angle = s_IMU_FilterProcessInfo.static_roll_angle;
	//SensorDataOutput->static_yaw_angle = s_IMU_FilterProcessInfo.static_yaw_angle;
	
	SensorDataOutput->fused_pitch_angle = s_IMU_FilterProcessInfo.fused_pitch_angle;
	SensorDataOutput->fused_roll_angle = s_IMU_FilterProcessInfo.fused_roll_angle;
	SensorDataOutput->fused_yaw_angle = s_IMU_FilterProcessInfo.fused_yaw_angle;
	
	SensorDataOutput->pitch_anglerate = s_IMU_FilterProcessInfo.pitch_anglerate;
	SensorDataOutput->roll_anglerate = s_IMU_FilterProcessInfo.roll_anglerate;
	SensorDataOutput->yaw_anglerate = s_IMU_FilterProcessInfo.yaw_anglerate;
}

void SensorUpdateAnglerate(IMURawInfo SensorDataInput, IMUFilterInfo *SensorDataOutput)
{
	static bool is_first_update = true;
	s32 fused_kg = 20;
	s32 sin_pitch_angle = GetSin(s_IMU_FilterProcessInfo.fused_pitch_angle);
	s32 cos_pitch_angle = GetCos(s_IMU_FilterProcessInfo.fused_pitch_angle);
	s32 sin_roll_angle = GetSin(s_IMU_FilterProcessInfo.fused_roll_angle);
	s32 cos_roll_angle = GetCos(s_IMU_FilterProcessInfo.fused_roll_angle);
	
	LIMIT(SensorDataInput.accel_pitch, DEGREE_QCALC);
	LIMIT(SensorDataInput.accel_roll, DEGREE_QCALC);
	
	//calculate angle rate
	s_IMU_FilterProcessInfo.pitch_anglerate = (cos_roll_angle * (SensorDataInput.gyro_x / 4) + \
											   sin_roll_angle * (SensorDataInput.gyro_z / 4)) / 1024;
	
    if(cos_pitch_angle == 0) cos_pitch_angle = 1;
	s_IMU_FilterProcessInfo.roll_anglerate = ((sin_pitch_angle * sin_roll_angle) / cos_pitch_angle) * (SensorDataInput.gyro_x / 4) / 1024 + \
											   SensorDataInput.gyro_y - ((cos_roll_angle * sin_pitch_angle) / \
											   cos_pitch_angle) * (SensorDataInput.gyro_z / 4) / 1024;

	s_IMU_FilterProcessInfo.yaw_anglerate = (SensorDataInput.gyro_z/4 * cos_roll_angle/cos_pitch_angle)*4 - 
                                            (SensorDataInput.gyro_x/4 * sin_roll_angle/cos_pitch_angle)*4;	
	
	SensorDataOutput->pitch_anglerate = s_IMU_FilterProcessInfo.pitch_anglerate;
	SensorDataOutput->roll_anglerate = s_IMU_FilterProcessInfo.roll_anglerate;
	SensorDataOutput->yaw_anglerate = s_IMU_FilterProcessInfo.yaw_anglerate;
}

void QuaternionToEular(long quat[4], IMUFilterInfo *SensorDataOutput)
{
    static bool is_first_update = true;
    float pitch,roll;
    //float yaw;
    float w,x,y,z;	
	float t1,t11,t2;
    s32 gyro_x = 0, gyro_y = 0, gyro_z = 0;
    w = quat[0] / 1073741824.0f;
    x = quat[1] / 1073741824.0f;
    y = quat[2] / 1073741824.0f;
    z = quat[3] / 1073741824.0f;
    
    t1 = 2 * (w * x + y * z);
    t11 = 1 - 2 * (x *x + y * y);
    t2 = 2 * (w * y - z * x);

    roll = atan2(t1,t11) * 57.3f;
    pitch = asin(t2) * 57.3f;
    
    SensorDataOutput->fused_pitch_angle = -pitch*DEGREE_QCALC;
    SensorDataOutput->fused_roll_angle = roll*DEGREE_QCALC;
  
    if(is_first_update)
	{
		is_first_update = false;
		s_IMU_FilterProcessInfo.fused_pitch_angle = SensorDataOutput->fused_pitch_angle;
        s_IMU_FilterProcessInfo.fused_roll_angle = SensorDataOutput->fused_roll_angle;
	}

}

/*******************************************************************************
* Description    : 自动校准陀螺仪零点
* Input          : None
* Output         : None
* Return		 : None
* Notes          : None
*******************************************************************************/
static u8 CalibrationSecond = 10;
static u8 CalibrationFactor = 1;
static s32 ZeroX = -3000, ZeroY = -4000, ZeroZ = -2000;
bool GyroZeroCalibration(s32 gx, s32 gy, s32 gz)
{
    bool ret = false;
    static u16 s_count = 0;
    static ThreeAxis_Type s_sum = {0,0,0};
    
    if(IsWheelRest())
    {
        if(s_count > 20)
        {
            if(abs(gx - s_sum.x/s_count) > REST_ANGLE_THRESHOLD) goto CALIBRATION_RESET;
            if(abs(gy - s_sum.y/s_count) > REST_ANGLE_THRESHOLD) goto CALIBRATION_RESET;
            if(abs(gz - s_sum.z/s_count) > REST_ANGLE_THRESHOLD) goto CALIBRATION_RESET;
        }
        
        s_sum.x += gx;
        s_sum.y += gy;
        s_sum.z += gz;
        s_count++;
        
        if(s_count >= CalibrationSecond * IMU_UPDATE_FREQ) //累计秒数
        {
            ZeroX = s_sum.x/s_count;
            ZeroY = s_sum.y/s_count;
            ZeroZ = s_sum.z/s_count;
            ret = true;
            goto CALIBRATION_RESET;
        }
    }
    else
    {
        CALIBRATION_RESET:
        s_sum.x = 0;
        s_sum.y = 0;
        s_sum.z = 0;
        s_count = 0;
    }
    
    return ret;
}

/*******************************************************************************
* Description    : 上电启动后首次校准陀螺仪零点
* Input          : None
* Output         : None
* Return		 : None
*******************************************************************************/
static bool FirstGyroZeroCalibration()
{
    MPUSensorData imu_data;
    s32 gyro_x = 0, gyro_y = 0, gyro_z = 0;
        
    #ifdef USE_DMP
    INVMPU_DmpReadFifo(&imu_data);
    #else
    INVMPU_ReadGyro(&imu_data);
    #endif

    gyro_y = imu_data.gyro_y * DEGREE_QCALC / GYRO_FSR_CALC;
    gyro_x = imu_data.gyro_x * DEGREE_QCALC / GYRO_FSR_CALC;
    gyro_z = imu_data.gyro_z * DEGREE_QCALC / GYRO_FSR_CALC;
    
    if(GyroZeroCalibration(gyro_x, gyro_y, gyro_z))
    {
        return true;
    }
    
    return false;
}

/*******************************************************************************
* Function Name  : SensorUpdateTask
* Description    : 陀螺仪加速度计传感器数据采集
* Input          : None
* Output         : None
* Return		 : None
* Notes          : None
*******************************************************************************/
void SensorUpdateTask(void)
{
    static u8 imu_test_flag = 1;
    static u8 zero_calibration_flag = 1;
    
    MPUSensorData imu_data;
    IMURawInfo	SensorSystemPreFilter = {0};
    
    s32 gyro_x = 0, gyro_y = 0, gyro_z = 0;
	s32 accel_x = 0, accel_y = 0, accel_z = 0;   
    
    if(!RunFlag.imu) return;
    RunFlag.imu = 0;
    
    if(imu_test_flag)
    {
        if(!MPU6500_Test()) return;
        imu_test_flag = 0;
        
        //初始化滤波器
        FIR_Filter_int_Clear(&s_tAccxFilter);
        FIR_Filter_int_Clear(&s_tAccyFilter);
        FIR_Filter_int_Clear(&s_tAcczFilter);
            
        FIR_Filter_int_Clear(&s_tGyroxFilter);
        FIR_Filter_int_Clear(&s_tGyroyFilter);
        FIR_Filter_int_Clear(&s_tGyrozFilter);
        
        HAL_Delay(10);        
    }
    
    if(zero_calibration_flag)
    {
        CalibrationSecond = 1;
        CalibrationFactor = 20;
        
        if(!FirstGyroZeroCalibration()) return;
        zero_calibration_flag = 0;
        CalibrationSecond = 10;
        CalibrationFactor = 1;
        
        CLEAR_ERR(DRV_ERR_IMU);
    }
    
    INVMPU_ReadGyro(&imu_data);
    INVMPU_ReadAccel(&imu_data);
    
    gyro_x = imu_data.gyro_x * DEGREE_QCALC / GYRO_FSR_CALC;
    gyro_y = imu_data.gyro_y * DEGREE_QCALC / GYRO_FSR_CALC;
	gyro_z = imu_data.gyro_z * DEGREE_QCALC / GYRO_FSR_CALC;
    
    //零点调整
    GyroZeroCalibration(gyro_x, gyro_y, gyro_z);        
    gyro_x -= ZeroX;
    gyro_y -= ZeroY;
    gyro_z -= ZeroZ;
    
    accel_x = imu_data.accel_x * DEGREE_QCALC / ACCEL_FSR_CALC;
	accel_y = imu_data.accel_y * DEGREE_QCALC / ACCEL_FSR_CALC;
	accel_z = imu_data.accel_z * DEGREE_QCALC / ACCEL_FSR_CALC;	
    
    //FIR滤波
    SensorSystemPreFilter.accel_roll = FIR_Filter_int(&s_tAccyFilter, accel_x);
    SensorSystemPreFilter.accel_pitch = FIR_Filter_int(&s_tAccxFilter, accel_y);
    SensorSystemPreFilter.accel_z = FIR_Filter_int(&s_tAcczFilter, accel_z);
    
    SensorSystemPreFilter.gyro_x = FIR_Filter_int(&s_tGyroyFilter, gyro_x);
    SensorSystemPreFilter.gyro_y = FIR_Filter_int(&s_tGyroxFilter, gyro_y);
    SensorSystemPreFilter.gyro_z = FIR_Filter_int(&s_tGyrozFilter, gyro_z);
    
    //惯导滤波算法
    SensorUpdate_DoubleAxis(SensorSystemPreFilter, &g_SensorSystemStatus);        

}
