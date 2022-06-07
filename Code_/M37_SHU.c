/*
 * @Author: BUGHHT
 * @Date: 2022-01-20 13:42:34
 * @LastEditors: Harryhht
 * @LastEditTime: 2022-03-03 01:50:26
 * @Description: 由于前人提供的M36中塞入了太多不必要的函数，因此重开新的M37，参考前人标准并删去不需要的内容，同时设计阶段考虑代码的可维护性。
 * （当然随时可能摸鱼不写注释，之后会补上的）
 */
#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include <stdio.h>
#include "M37_SHU.h"

#include "AC_Math.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"
#include "SeniorControl.h"

#include "drv_Uart2.h"
#include "drv_LED.h"

static void M37_SHU_MainFunc();
static void M37_SHU_enter();
static void M37_SHU_exit();
const Mode M37_SHU = {
    50,
    M37_SHU_enter,
    M37_SHU_exit,
    M37_SHU_MainFunc,
};
typedef struct
{
    //退出模式计数器
    uint16_t exit_mode_counter;

} MODE_INF;
static MODE_INF *Mode_Inf;

static void M37_SHU_enter()
{
    Led_setStatus(LED_status_running1);
    // Initialize Variable Mode_Inf
    Mode_Inf = malloc(sizeof(MODE_INF));
    Mode_Inf->exit_mode_counter = 0;
    Position_Control_Enable();
}

static void M37_SHU_exit()
{
    Position_Control_Disable();
    Altitude_Control_Disable();
    free(Mode_Inf);
}

static char sendinfo[5] = "{D9}\n";
static uint8_t sendtype = 0xFF;
static void M37_SHU_MainFunc()
{

    if (sendtype != 0xFF)
    {
        sendinfo[2] = sendtype + '0';
        Uart7_Send(sendinfo, 5);
        sendtype = 0xFF;
    }
    Position_Control_Enable();
    Software_LockHeight();
    if (!IsInflightOrder)
    {
        // Led_setSignal(LED_signal_success);
        Led_setStatus(LED_status_waiting);
        static int Modeindex = 0;
        switch (Modeindex)
        {
        //检查t265数据
        case 0:
        {
            if (vision_present && ultrasonic && LIDAR_DETECTED)
            {
                ++Modeindex;
                Uart7_Send(sendinfo, 5);
            }
            break;
        }
        //芜湖，起飞
        case 1:
        {
            // sendtype=0;
            //   OrderSet_TakeOffWithDelay(80.f, 20);
            OrderSet_TakeOffWithInit(80.f, 0);
            Modeindex++;
            break;
        }
        //定高
        case 2:
        {
            OrderSet_HeightWithCheck(0, 0, 90.f, 30.f);
            Ctrl_SetLockHeightType(LockHeightType_t265);
            Ctrl_LockHeight(90);
            ++Modeindex;
            // Modeindex = -1;
            break;
        }
        // Tasks
        case 3:
        {
            OrderSet_PositionWithCheck_IMG(300.f, 0.f, 20);
            ++Modeindex;
            break;
        }
        case 4:
        {
            // OrderSet_LandByT265(200.f, 0.f);
            OrderSet_LandByImage();
            ++Modeindex;
            // Modeindex = -4;
            break;
        }
        case 5:
        {
            sendtype = 0;
            state_x = pos_vision.x;
            state_y = pos_vision.y;
            ++Modeindex;
            break;
        }
        case 6:
        {
            OrderSet_TakeOffWithDelay(20, 1);
            ++Modeindex;
            break;
        }
        case 7:
        {
            OrderSet_HeightWithCheck(state_x, state_y, 90.f, 30.f);
            Ctrl_SetLockHeightType(LockHeightType_t265);
            Ctrl_LockHeight(90);
            ++Modeindex;
            break;
        }
        case 8:
        {
            OrderSet_PositionWithCheck_IMG(state_x, -300.f, 20);
            ++Modeindex;
            break;
        }
        case 9:
        {
            OrderSet_LandByImage();
            ++Modeindex;
            break;
        }
        case 10:
        {
            sendtype = 1;
            state_x = pos_vision.x;
            state_y = pos_vision.y;
            ++Modeindex;
            break;
        }
        case 11:
        {
            OrderSet_TakeOffWithDelay(60, 1);
            ++Modeindex;
            break;
        }
        case 12:
        {
            OrderSet_HeightWithCheck(state_x, state_y, 90.f, 30.f);
            Ctrl_SetLockHeightType(LockHeightType_t265);
            Ctrl_LockHeight(90);
            ++Modeindex;
            break;
        }
        case 13:
        {
            OrderSet_PositionWithCheck_IMG(0.f, state_y, 30.f);
            ++Modeindex;
            break;
        }
        case 14:
        {
            OrderSet_LandByImage();
            ++Modeindex;
            break;
        }
        case 15:
        {
            sendtype = 2;
            state_x = pos_vision.x;
            state_y = pos_vision.y;
            ++Modeindex;
        }
        case 16:
        {
            OrderSet_TakeOffWithDelay(50, 1);
            ++Modeindex;
            break;
        }
        case 17:
        {
            OrderSet_HeightWithCheck(state_x, state_y, 50.f, 30.f);
            Ctrl_SetLockHeightType(LockHeightType_t265);
            Ctrl_LockHeight(50);
            Modeindex = -1;
            break;
        }
        //降落准备：回到原点
        case -1:
        {
            OrderSet_PositionWithCheck_IMG(0.f, 0.f, 30);
            Modeindex--;
            break;
        }
        case -2:
        {
            Position_Control_set_XYLock();
            Modeindex--;
            break;
        }
        //降落
        case -3:
        {
            if (ultrasonic)
            {
                OrderSet_LandByUltrasonic();
            }
            else
                Buzzer(true);
            // Modeindex = 1; // test refly
            Modeindex--;
            break;
        }
        }
    }
    else
    {
        // Led_setSignal(LED_signal_continue);
        OrderServer();
    }
}
