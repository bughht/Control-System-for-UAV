/*
 * @Author: Bughht
 * @Date: 2022-01-20 18:10:23
 * @LastEditors: Harryhht
 * @LastEditTime: 2022-03-03 01:28:37
 * @Description: Senior Controlling System
 */

#pragma once

#include "Basic.h"
#include "AC_Math.h"
#include "ControlSystem.h"
#include "Modes.h"
#include "M37_SHU.h"

#include "MavlinkRCProcess.h"
#include "MeasurementSystem.h"

#include "drv_Ultrasonic.h"
#include "drv_LED.h"
#include "drv_Uart7.h"

#define get_position_t265() pos_vision

#define get_height_self() (get_Position().z - Height)
#define get_height_t265() (get_position_t265().z - Height_t265)
#define get_height_asonic() (ultrasonic - Height_asonic)

#define Vel_MAX 95  //最大速度
#define FlightR 7.1 //飞机半径
#define EXITThreshold 5
#define POLEMAXDISTANCE_1 60
#define POLEMAXDISTANCE_2 60

//定高方式
extern enum LockHeightOrder {
    LockHeightType_asonic = 0,
    LockHeightType_t265,
    LockHeightType_self
} LockHeightType;

// #define LockHeightType_asonic 0
// #define LockHeightType_t265 1
// #define LockHeightType_self 2
// uint8_t LockHeightType = LockHeightType_t265;

extern enum MyFlightOrder {
    Exit = 1,
    SetPositionWithCheck,
    SetPositionWithCheck_WXJ,
    SetPositionWithCheck_IMG,
    TurnSelfCenter,
    Circlrmode,
    LandByUltrasonic,
    LandByT265,
    LandByImage,
    TakeOffWithInit,
    TakeOffWithDelay,
    SetHeightWithCheck,
    TakeOff45degree
} OrderMode;

extern float receive_x,
    receive_y, receive_z, receive_QR, receive_QRType, receive_QRHeight, receive_QRx;

extern vector3_float pos_vision;
extern int CAM_dy, CAM_dx;
extern int Pole_x, Pole_y;
extern float state_x, state_y;

extern bool guided_enabled;
extern bool pos_enabled;

//封装的操作所需的姿态数据
extern uint32_t time_count;

extern uint32_t time_delay;
extern vector3_float TgtPos;
extern uint8_t MaxPosLimit;

extern vector3_float current_pos;
extern float target_yaw;
extern float turnyaw_errx, turnyaw_erry;
extern float CircleRadiu, CirCleYaw_err, CirCleDelayYaw_err;
extern float tempReceive_y;
extern float vel, velx, vely;
extern float X_offset, Y_offset, Z_offset;

// Height为起飞时飞机自身的高度，Height_t265为起飞时t265的高度
extern float Height, Height_t265;
extern float Height, Height_t265, Height_asonic;
extern bool IsDirUp;
extern float tgt_height, Lock_Height;
extern float yawForRotation1;
extern float yawForRotation2;
extern float yawForRevolution;
extern float R;

//超声高度
extern float ultrasonic;
//是否执行封装的操作
extern bool IsInflightOrder;
//是否软件锁定高度
extern bool IsLockHeight;
//封装的操作的类型

void Software_LockHeight(void);
void OrderServer(void);

void OrderSet_PositionWithCheck(float tgtx, float tgty, uint8_t limit);
void OrderSet_PositionWithCheck_WXJ(float tgtx, float tgty, uint8_t limit);
void OrderSet_PositionWithCheck_IMG(float tgtx, float tgty, uint8_t limit);
void OrderSet_PositionWithCheckRelative(float tgtx, float tgty, uint8_t limit);
void OrderSet_PositionWithCheckRelative_BodyHeading(float tgtx, float tgty, uint8_t limit);
void OrderSet_TurnSelfCenter(float tgtyaw, float yaw_rate);
void OrderSet_Circlrmode(float tgtyaw, float yaw_rate, float radiu, float yaw_err);
void OrderSet_LandByUltrasonic();
void OrderSet_LandByT265(float, float);
void OrderSet_LandByImage(void);
void OrderSet_TakeOffWithInit(float myHeight, uint32_t delay);
void OrderSet_TakeOffWithDelay(float myHeight, uint32_t delay);
void OrderSet_TakeOff45degree(float myHeight);
void OrderSet_HeightWithCheck(float, float, float myheight, float myvelosity);

void Ctrl_LockHeight(float myHeight);
void Ctrl_unLockHeight();
void Ctrl_SetLockHeightType(uint8_t type);

float Func_Dis2Vel(float distance);

bool Get_Guided_Mode_Enabled(void);
bool Get_POS_Control_Enabled(void);
void Set_POS_Control_Enabled(bool value);