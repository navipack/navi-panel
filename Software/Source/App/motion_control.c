/**
******************************************************************************
* @file    motion_control.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   运动控制模块，包含了一些加减速和初始化之类的算法
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "motion_control.h"
#include "global_defines.h"
#include "svpwm_types.h"
#include "system_supervise.h"
#include "math.h"
#include "control_params.h"
#include "current_params.h"
#include "Queue.h"
#include "math_lib.h"
#include "gpio_user.h"
//#include "ShellyTalkInternal.h"
#include "sensor_update.h"
#include "comm.h"
#include "infrared_drop.h"
#include "tim_user.h"

#define FREQ(cnt,hz)  (++(cnt) >= MOTION_PREQ/(hz))

#define FIR_MOVING_AVG_DLY_SIZE 1000
#define _xHz_CNT 	4


extern u8 MotionCmdProcessFlag;

PIDObjTyp ThetaLoopPIDV;
PIDObjTyp ThetaLoopPIDW;
PIDObjTyp ThetaLoopPIDMiddleW;

CTwoCarLocation TestTargetLocation;
CCarLocation TestPresentLocation;
CCarLocation gSpecifiedDecStartingLocation;
//CCarLocation gCTargetLocation;
//CCarLocation gCFartherTargetLocation;

u16 CntXHz = _xHz_CNT;


/////////////////// Local Variable ////////////////////////
//static OS_EVENT* PresentLoactionLock = NULL;
//static OS_EVENT* PresentLoactionMBox = NULL;
//static MotionControlMode_TypeDef MotionControlMode = MOTION_POINT_MODE;
static CSpeedVW VWTModeSpeed = {0, 0};
static u8 VW_Update = 0;
static u32 HeartBeatCount = 0;
static u8 EnterSpecialDecreasing = 0;
static s32 LinearVelocity;
static s32 AngVelocity;
//static VelocityCurve_Type VelocityCurve;
//static VelocityCurve_Type HeadCurve;
static bool CarMotionEnable = true;

void MotorParamsInit(void)
{
    MotorParams[0].AccumulatedDistance = 0;
    MotorParams[0].AccumulatedDistanceRemainder = 0;
    
    MotorParams[1].AccumulatedDistance = 0;
    MotorParams[1].AccumulatedDistanceRemainder = 0;
}

/**
* @brief  初始化所有三个编码器和霍尔等设备
* @param  None
* @retval None
* @return None
*/
void MotionInitEncoderAndHall(void)
{
    u8 i;
    //for(i=0; i<3; i++)
    //for(i=0; i<2; i++)
    //    EncInit(i);

	//HallIrqInit();
    
    EncInit(0);
}

/**
* @brief  重设位置环计数器
* @param  None
* @retval None
* @return None
*/
void ResetPosLoopCount(void)
{
	CntXHz = _xHz_CNT;
}

/**
* @brief  处理加速的函数
* @param  mt_idx rl: 0表示左轮；1表示右轮
* @retval None
* @return None
*/
u8 EzCANCalcMotoSpeedAccTime(u8 mt_idx)
{
#define SPEED_LOOP_FREQ ((s32)2000)
    
    //加速环节计算
	MotorParams[mt_idx].TargetSpeedIncDivisor = (s32)MotorParams[mt_idx].AccPositiveTime * (s32)SPEED_LOOP_FREQ;
	MotorParams[mt_idx].TargetSpeedIncRemainder = MotorParams[mt_idx].MaxSpeed * 1000 / MotorParams[mt_idx].TargetSpeedIncDivisor;
	MotorParams[mt_idx].TargetSpeedIncModulus = MotorParams[mt_idx].MaxSpeed * 1000 % MotorParams[mt_idx].TargetSpeedIncDivisor;	

    //减速环节计算
    MotorParams[mt_idx].TargetSpeedIncDivisor = (s32)MotorParams[mt_idx].AccNegativeTime * (s32)SPEED_LOOP_FREQ;
	MotorParams[mt_idx].TargetSpeedIncRemainder = MotorParams[mt_idx].MaxSpeed * 1000 / MotorParams[mt_idx].TargetSpeedIncDivisor;
	MotorParams[mt_idx].TargetSpeedIncModulus = MotorParams[mt_idx].MaxSpeed * 1000 % MotorParams[mt_idx].TargetSpeedIncDivisor;	
    
	return 1;
}

/**
* @brief  计算全局坐标
* @param  None
* @retval None
* @return None
*/
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329575)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

#define M_PI       (float)(3.14159265358979323846)

#define WHEEL_SPAN		(float)(0.28)	//??:m
#define WHEEL_RADIUS	(float)(0.05)	//??:m
#define GEAR_RATIO		(float)(14)

#define ENCODER_PLUSE	(int)(500)
#define MAX_SPEED_FOR_REAL (float)(0.08)	//??:m/s
#define MAX_SPEED_FOR_MOTOR (int)(200000)	//??:?/s

#define R_TO_M(r)	(int)(((float)r*4.0*ENCODER_PLUSE*GEAR_RATIO)/(2*M_PI*WHEEL_RADIUS))	//?????????????
#define M_TO_R(m)	(float)((float)m*(2.0*M_PI*WHEEL_RADIUS)/(4*ENCODER_PLUSE*GEAR_RATIO))

extern s32 TestActualVW[4];
s32 MAX_CONST = 2*MAX_PWM;

/**************************************************************************
*
*
***************************************************************************/
bool CalculateTheta( const MotionTargetType *first_location, const MotionTargetType *second_location, s32 *theta )
{
	s32 distanceX = 0;
	s32 distanceY = 0;
	s32 temp_distanceTheta = 0;
	
	distanceX = second_location->sAxisX  - first_location->sAxisX;
	distanceY = second_location->sAxisY  - first_location->sAxisY;
	
	temp_distanceTheta = GetAtan(distanceX, distanceY, 1);
	
	while( temp_distanceTheta >= DEGREE(360) )
	{
		temp_distanceTheta -= DEGREE(360);
	}
	while( temp_distanceTheta < 0 )
	{
		temp_distanceTheta += DEGREE(360);
	}
	
	*theta = temp_distanceTheta;
    return true;
}


/**************************************************************************
* Description    : 生成到达线
***************************************************************************/
bool JudgeLineGenerator( const MotionTargetType* present_location, const MotionTargetType* target_location, LineParameter *judge_line )
{
    PointCoordinate firstpoint;
    PointCoordinate secondpoint;
    LineParameter ligatureline;
    LineParameter verticalline;
    
    firstpoint.x = present_location->sAxisX;
    firstpoint.y = present_location->sAxisY;
    secondpoint.x = target_location->sAxisX;
    secondpoint.y = target_location->sAxisY;
    
    ligatureline = CalculateLine( &firstpoint, &secondpoint );
    verticalline = CalculateVerticalLine( &ligatureline, &secondpoint );
     
    *judge_line = verticalline;
    
    return true;
}
/**
* @brief  未融合的底盘位姿更新
* @param  present_location : 位姿数据
* @param  deltadistance    : 移动距离
* @param  deltatheta       : 移动角度
* @retval None
*/
void CarLocationUpdate_NoFusion( CDistanceValue* distance, s32 delta_distance, s32 delta_theta)
{	
	distance->theta += delta_theta;

    distance->theta %= DEGREE(360);
    if(distance->theta < 0)
	{
		distance->theta += DEGREE(360);
	}
    
    distance->distance += delta_distance;
}

/**
* @brief  底盘位姿更新
* @param  present_location        : 融合位姿数据
* @param  no_fusion_distance      : 不融合位姿数据
* @param  no_fusion_sync_distance : 不融合位姿数据，可被重置
* @param  present_speed           : 速度
* @param  period                  : 周期时间，单位微秒
* @retval None
*/
#define W_NOISE_TH  (DEGREE(1)/4)
void CarLocationUpdate(CDistanceValue* no_fusion_distance, CDistanceValue* no_fusion_sync_distance, 
    const CSpeedVW *present_speed, s32 period )
{
    s32 delta_theta;
    s32 delta_distance = 0;
    static s32 delta_theta_remainder;
    static s32 delta_distance_remainder;
    
    //位移
    delta_distance = present_speed->sV * period + delta_distance_remainder;
    delta_distance_remainder = delta_distance % 1000000;
    delta_distance /= 1000000;
    
    //角度变化，左转弯为角速度正方向
    if(abs(present_speed->sW) < W_NOISE_TH)
    {
        delta_theta = 0;
    }
    else
    {
        delta_theta = present_speed->sW * period + delta_theta_remainder;
        delta_theta_remainder = delta_theta % 1000000;
        delta_theta /= 1000000;
    }
    CarLocationUpdate_NoFusion(no_fusion_distance, delta_distance, delta_theta);
    CarLocationUpdate_NoFusion(no_fusion_sync_distance, delta_distance, delta_theta);
}


/**************************************************************************
*
*
***************************************************************************/

bool CalculateDistance( const MotionTargetType *first_location, const MotionTargetType *second_location, s32 *distance )
{
	s32 distanceX = 0;
	s32 distanceY = 0;
	s64 temp_squrt;
	
	distanceX = second_location->sAxisX  - first_location->sAxisX;
	distanceY = second_location->sAxisY  - first_location->sAxisY;
	
	if( distanceX < 0 )
		distanceX = -distanceX;
	if( distanceY < 0 )
		distanceY = -distanceY;
	temp_squrt = (distanceX/10)*(distanceX/10) + (distanceY/10)*(distanceY/10);
	*distance = GetSqurt32( temp_squrt )*10;
	return true;
}


/**************************************************************************
* Description    : 位置融合
***************************************************************************/
#define COR_FULL_BITS       13
#define COR_FULL_FACTOR     (1<<COR_FULL_BITS)
// 位移代替除法计算，并保留除法余数
#define FUSION_DIVISION(value, result, remainder) do{\
    if((value) < 0) {\
        (result) = ((value) >> COR_FULL_BITS) + 1;\
        (remainder) = (s32)(value) | (~(s32)(COR_FULL_FACTOR-1));\
    } else {\
        (result) = ((value) >> COR_FULL_BITS);\
        (remainder) = (s32)(value) & (s32)(COR_FULL_FACTOR-1);\
    }\
}while(0)
bool CorrectionCarLocation( const MotionTargetType *correct_location, MotionTargetType *present_location, s32 fusion_factor_xy, s32 fusion_factor_phi)
{
	s32 temp_CORrandian;
	s32 temp_PRErandian;
	s32 theta;
    s64 tmp;
    static s32 remainder_x, remainder_y, remainder_phi;
    
    temp_CORrandian = correct_location->sTheta;
    temp_PRErandian = present_location->sTheta;
    
    if( abs( temp_CORrandian - temp_PRErandian ) > DEGREE(180) )
    {
        if( temp_CORrandian > temp_PRErandian )
        {
            temp_PRErandian = temp_PRErandian + DEGREE(360);
        }
        else
        {
            temp_CORrandian = temp_CORrandian + DEGREE(360);
        }
    }
    
    tmp = ((s64)temp_PRErandian * (COR_FULL_FACTOR - fusion_factor_phi) + (s64)temp_CORrandian * fusion_factor_phi ) + remainder_phi;
    theta = tmp >> COR_FULL_BITS;
    remainder_phi = (s32)tmp & (s32)(COR_FULL_FACTOR-1);
    
    while(theta > DEGREE(360))
    {
        theta -= DEGREE(360);
    }
    present_location->sTheta = theta;
    
    tmp = (s64)present_location->sAxisX * (COR_FULL_FACTOR - fusion_factor_xy) + (s64)correct_location->sAxisX * fusion_factor_xy + remainder_x;
    FUSION_DIVISION(tmp, present_location->sAxisX, remainder_x);
    tmp = (s64)present_location->sAxisY * (COR_FULL_FACTOR - fusion_factor_xy) + (s64)correct_location->sAxisY * fusion_factor_xy + remainder_y;
    FUSION_DIVISION(tmp, present_location->sAxisY, remainder_y);
        
	return true;	
}

/**************************************************************************
*
*localization fusion
*
* theta:	1° = 4096
***************************************************************************/
typedef struct{
	s32 delta_x;
	s32 delta_y;
	s32 delta_phi;
} DELTA_POSE;

#define GYRO_W_TH   										3500
#define GYRO_ACC_W_TH   									4000
#define SLAM_DELTA_PHI_TH   								DEGREE(10)//0-360
#define SLAM_DELTA_X_TH   									200 //unit :mm
#define SLAM_DELTA_Y_TH   									200 //unint :mm
#define ST_SLAM_DIF_TH   									12
#define LF_ZERO   											120
#define GYRO_PERIOD   										50//50/250 = 200ms
#define DELTA_ORIGINAL_ST_POSE_PERIOD   					50
#define DELAY_FOR_SLAM_RECOVERY_TIME   						250*2
#define DELAY_FOR_BRAKE_RECOVERY_TIME   					125
#define AVG_FILTER_TAP 										5

/**************************************************************************
*
*
***************************************************************************/
bool LinearVelocityGenerator( s32 setvalue, s32 *presentmax )
{
	static s32 velocity_inc = BasicVelocityInc;
	static s32 velocity_max = LocationLoopV_Max;
	
	if( setvalue < 0 )//special decreasing
	{
		velocity_inc = -( setvalue<<1 );
		if( *presentmax > 0 )
		{
			*presentmax -= velocity_inc;
			if( *presentmax < 0 )
				*presentmax = 0;
		}
	}
	else if( setvalue >= 0 )
	{
		velocity_inc = BasicVelocityInc;
		velocity_max = LocationLoopV_Max * setvalue / 100;
		if( *presentmax < velocity_max )
		{
			*presentmax += velocity_inc;
			if( *presentmax > velocity_max )
			{
				*presentmax = velocity_max;
			}
		}
		else if( *presentmax > velocity_max )
		{
			*presentmax -= velocity_inc;
			if( *presentmax < 0 )
			{
				*presentmax = 0;
			}
		}
	}
	
	return true;
}

/**************************************************************************
*
*
***************************************************************************/
s32 Max_V = LocationLoopV_Max;

/**
* @brief  设置VW模式的速度值
* @param  v: 线速度
* @param  w: 角速度
* @param  t: 运行时间
* @retval None
*/
void SetVWValue(s32 v, s32 w, u16 t)
{
    if(v == 0 && w != 0 && abs(w) < 150)
    {
        w = ((w >> 8) | 0x01) * 150;
    }
    
    VWTModeSpeed.sV = v;
    VWTModeSpeed.sW = w;
    //VWTModeSpeed.time = t * (MOTION_PREQ/1000);
    VW_Update = 1;
}

void SetCarMotionEnable(bool b)
{
    CarMotionEnable = b;
}


/**
* @brief  传感器触发的停止动作处理
* @param  target: 输出速度值
* @param  time_threshold : 保护超时门限
* @retval 是否处于危险保护状态
*/
bool DropAndCollisionSensorHandler(CSpeedVW *target, u16 time_threshold)
{
    static bool drop_stop = false, collision_stop = false, is_protect = false;
    static u8 last_collision = 0;
    static u16 time_cnt = 0;
    u8 drop, collision;
    
    // 跌落传感器数据
    //drop = 0;
    drop = InfraredDrop_GetData();
    //drop = UltrasonicDrop_GetData();
    
    // 碰撞传感器数据
    collision = 0;
    
    if(last_collision == collision)
    {
        if(collision != 0)
        {
            collision_stop = true;
            NavipackComm.status.collisionSensor  = collision;
        }
        else
        {
            collision_stop = false;
            NavipackComm.status.collisionSensor = 0;
        }
    }
    last_collision = collision;
    
    // 跌落
    NavipackComm.status.dropSensor = drop;
    drop_stop = drop != 0;
    
    // 保护模式的超时机制，防止一直保护
    if(is_protect && ++time_cnt > time_threshold)
    {
        time_cnt = 0;
        drop_stop = false;
        collision_stop = false;
    }
    
    if(drop_stop || collision_stop)
    {
        if(target->sV > 0)
        {
            is_protect = true;
            target->sV = -500;
        }
    }
    else
    {
        if(is_protect)
        {
            is_protect = false;
            target->sV = 0;
            target->sW = 0;
        }
    }
    
    return is_protect;
}

/**
* @brief  判断主动轮是否在转动
* @param  None
* @retval 是否转动
*/
bool IsWheelRest()
{
    if(MotorParams[0].PresentSpeed == 0 && MotorParams[1].PresentSpeed == 0 && //编码器输出判断
        MotorParams[0].PresentCurrentDQ.Iq < 100 && MotorParams[1].PresentCurrentDQ.Iq < 100) //电机输出电流判断，毫伏
    {
        return true;
    }
    return false;
}

/**
* @brief  底盘运动控制
* @param  None
* @retval None
*/
//static u8 HeadMode = HEAD_INIT_MODE;
//static u8 PlatMode = MOTION_VW_MODE;
void ChassisMovingController()
{
    static u16 cnt = 0, cor_cnt = 0, stop_cnt = 0;
    static u8 origin_zone_flag;
    static CTwoDistance two_target_distance;
    static CTwoCarLocation two_target_location;
    static LineParameter arrive_judge_line;
    static bool is_queue_reset, is_protect = false;
    static CDistanceValue no_fusion_distance = {0,0};
    static CDistanceValue no_fusion_sync_distance = {0,0};
    static CSpeedVW present_vw = {0,0};
    static CSpeedVW target_VW = {0,0};
    static NaviPack_StatusType* status = &NavipackComm.status;
    
    bool same_dir;
    
    // 强制设置位置
    //GetForcePresentLocation(&present_location, &no_fusion_sync_distance);
    // 车当前位姿更新
    CarLocationUpdate(&no_fusion_distance, &no_fusion_sync_distance, &present_vw, 1000000/MOTION_PREQ);
   
    if(VW_Update && !is_protect)
    {
        VW_Update = 0;
        target_VW.sV = VWTModeSpeed.sV;
        target_VW.sW = VWTModeSpeed.sW;
    }
    
    present_vw.sV = GetVelocity(target_VW.sV);
    present_vw.sW = GetOmega(target_VW.sW);
    
    // 通讯反馈
    if(Navipack_LockReg(REG_ID_STATUS))
    {
        status->angularPos = DEGREE_TO_RADIAN(no_fusion_distance.theta);
        status->leftEncoderPos = MotorParams[0].AccumulatedDistance;
        status->rightEncoderPos = MotorParams[1].AccumulatedDistance;
        status->lineVelocity = present_vw.sV;
        status->angularVelocity = DEGREE_TO_RADIAN(present_vw.sW);
        Navipack_UnlockReg(REG_ID_STATUS);
    }

#ifdef _DEBUG
    if(UserReg.debugFlag & 0x02)
    {
        is_protect = false;
    }
    else if(FREQ(stop_cnt, 500))
#else
    if(FREQ(stop_cnt, 500))
#endif
    {
        stop_cnt = 0;
        is_protect = DropAndCollisionSensorHandler(&target_VW, 300); // 碰撞及跌落传感器触发刹车策略
    }
    
    if(!CarMotionEnable && !is_protect)
    {
        target_VW.sV = 0;
        target_VW.sW = 0;
    }
    
    // 线速度、角速度环
    AngularVelocityController(target_VW.sV, target_VW.sW, present_vw.sV, present_vw.sW);
}

/**
* @brief  运动控制 Task
* @param  p_arg: 参数
* @retval None
*/
void MotionCtrlTask(void)
{
    static u8 motor_enable_flag = 1;
    static u16 drop_init_cnt = 0;
    
    if(!RunFlag.motion) return;
    RunFlag.motion = 0;
    
    if(motor_enable_flag)
    {
        motor_enable_flag = 0;
        MotorPIDInit();
        ChassisMotorDriverEnable( true );
    }
    
    if(drop_init_cnt < 500)
    {            
        InfraredDrop_InitData(true);
        drop_init_cnt++;
    }
    else if(drop_init_cnt == 500)
    {
        InfraredDrop_InitData(false);
        drop_init_cnt++;
    }
    else
    {
        ChassisMovingController();
    }
    
}
