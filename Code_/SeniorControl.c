/*
 * @Author: PIoneers
 * @Date: 2022-01-20 18:10:16
 * @LastEditors: Harryhht
 * @LastEditTime: 2022-03-03 01:51:15
 * @Description: Senior Controlling System
 */

#include "SeniorControl.h"
#include "drv_LED.h"
#include "InteractiveInterface.h"
enum LockHeightOrder LockHeightType = LockHeightType_t265;
enum MyFlightOrder OrderMode = Exit;
bool guided_enabled = false;
bool pos_enabled = true;
extern float receive_x, receive_y;
//封装的操作所需的姿态数据
uint32_t time_count = 0;
extern float receive_x, receive_y;
float state_x, state_y;
uint32_t time_delay;
vector3_float TgtPos;
uint8_t MaxPosLimit = 30;

vector3_float current_pos;
float target_yaw;
float turnyaw_errx, turnyaw_erry;
float CircleRadiu, CirCleYaw_err, CirCleDelayYaw_err;
float tempReceive_y;
float vel, velx, vely;
float X_offset = 0, Y_offset = 0, Z_offset = 0;
// Height为起飞时飞机自身的高度，Height_t265为起飞时t265的高度
float Height, Height_t265;
float Height = 0, Height_t265 = 0, Height_asonic = 15;
bool IsDirUp = false;
float tgt_height = 0, Lock_Height = 0;
float yawForRotation1 = PI / 18;
float yawForRotation2 = 0;
float yawForRevolution = 0;
float R = 0;

//超声高度
// float ultrasonic;
//是否执行封装的操作
bool IsInflightOrder = false;
//是否软件锁定高度
bool IsLockHeight = false;
//封装的操作的类型

void Software_LockHeight(void)
{
    if (IsLockHeight)
    {

        static uint8_t LockHeightDivCount = 0;
        if (++LockHeightDivCount > 20)
        {
            LockHeightDivCount = 0;
            switch (LockHeightType)
            {
            case LockHeightType_asonic:
            {
                Position_Control_set_TargetVelocityZ(constrain_float((Lock_Height - get_height_asonic()) * 0.5, 10));
                // Position_Control_set_TargetPositionZRelative(constrain_float(Lock_Height - get_height_asonic(), 10));
                break;
            }
            case LockHeightType_t265:
            {
                // Position_Control_set_TargetPositionZRelative(constrain_float(Lock_Height - get_height_t265(), 10));
                Position_Control_set_TargetVelocityZ(constrain_float((Lock_Height - get_height_t265()) * 0.5, 10));
                break;
            }
            case LockHeightType_self:
            {
                Position_Control_set_TargetPositionZRelative(constrain_float(Lock_Height - get_height_self(), 10));
                break;
            }
            }
        }
    }
}

void OrderServer(void)
{
    static uint8_t div_count = 0;
#define fre_5Hz 10
    // Led_setSignal(LED_signal_continue);
    switch (OrderMode)
    {
    case Exit:
    {
        div_count = 0;
        IsInflightOrder = false;
        break;
    }
    case SetPositionWithCheck:
    {
        Ctrl_LockHeight(LockHeightType_t265);
        vector3_float pos_now = get_position_t265();
        float dis = sqrtf((TgtPos.x - pos_now.x) * (TgtPos.x - pos_now.x) + (TgtPos.y - pos_now.y) * (TgtPos.y - pos_now.y));

        if (++div_count > fre_5Hz)
        {
            div_count = 0;
            if (dis > MaxPosLimit)
            {
                Position_Control_set_TargetPositionXYRelative((TgtPos.x - pos_now.x) * MaxPosLimit / dis, (TgtPos.y - pos_now.y) * MaxPosLimit / dis);
            }
            else
            {
                Position_Control_set_TargetPositionXYRelative(TgtPos.x - pos_now.x, TgtPos.y - pos_now.y);
            }
        }

        if (dis < EXITThreshold)
        {
            OrderMode = Exit;
        }
        break;
    }
    case SetPositionWithCheck_WXJ:
    {
        Ctrl_LockHeight(LockHeightType_t265);
        vector3_float pos_now = get_position_t265();
        float dis_x = TgtPos.x - pos_now.x;
        float dis_y = TgtPos.y - pos_now.y;
        float dis = sqrtf(dis_x * dis_x + dis_y * dis_y);
        float ang = atan2(dis_y, dis_x);
        float vel = Func_Dis2Vel(dis);

        if (++div_count > fre_5Hz)
        {
            div_count = 0;
            if (dis > MaxPosLimit)
            {
                div_count = 0;
                velx = vel * cosf(ang);
                vely = vel * sinf(ang);
                Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
            }
            else
            {
                vel -= 5;
                velx = vel * cosf(ang);
                vely = vel * sinf(ang);
                Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
                // Position_Control_set_TargetPositionXYRelative(dis_x, dis_y);
            }
        }

        if (dis < EXITThreshold)
        {
            OrderMode = Exit;
        }
        break;
    }

    case SetPositionWithCheck_IMG:
    {
        vector3_float pos_now = get_position_t265();
        float kx = 0.33, ky = 0.33;
        float stupid_dx = state_x - pos_now.x;
        float stupid_dy = state_y - pos_now.y;
        float stupid_dis = sqrtf(stupid_dx * stupid_dx + stupid_dy * stupid_dy);

        float pole_dis_x = Pole_x - pos_now.x;
        float pole_dis_y = Pole_y - pos_now.y;
        float pole_dis = sqrtf(pole_dis_x * pole_dis_x + pole_dis_y * pole_dis_y);

        float dis_x = TgtPos.x - pos_now.x;
        float dis_y = TgtPos.y - pos_now.y;
        float dis = sqrtf(dis_x * dis_x + dis_y * dis_y);
        float ang = atan2(dis_y, dis_x);
        float vel = Func_Dis2Vel(dis);
        float pole_ang = atan2(pole_dis_y, pole_dis_x) + PI_f * 0.45;
        float delta_ang = (fabsf(pole_ang - ang) > PI_f ? fabsf(pole_ang - ang) - PI_f : fabsf(pole_ang - ang));
        static float init_ang = -10086.f;
        if (pole_dis <= POLEMAXDISTANCE_1)
        {
            if (init_ang == -10086.f)
                init_ang = PI_f - delta_ang;
            if (delta_ang > init_ang)
                ;
            else if (delta_ang > PI_f / 2)
                ang = pole_ang - PI_f * 0.9;
            else
                ang = pole_ang;
            // vel = 35;
        }
        if (pole_dis <= POLEMAXDISTANCE_2)
        {
            vel = 35;
        }

        // if (CAM_DETECTED)
        // {
        //     Led_setSignal(LED_signal_success);
        //     dis_x = CAM_dx * kx;
        //     dis_y = CAM_dy * ky;
        //     dis = sqrtf(dis_x * dis_x + dis_y * dis_y);
        //     ang = atan2(dis_y, dis_x);
        //     vel = Func_Dis2Vel(dis) - 10;
        //     velx = vel * cosf(ang);
        //     vely = vel * sinf(ang);
        //     Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
        // }

        // if (++div_count > fre_5Hz)
        if (++div_count > 0)
        {
            div_count = 0;
            if (dis > MaxPosLimit)
            {
                vel -= 3;
                velx = vel * cosf(ang);
                vely = vel * sinf(ang);
                Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
            }
            else
            {
                vel -= 5;
                velx = vel * cosf(ang);
                vely = vel * sinf(ang);
                Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
            }
        }

        if ((CAM_DETECTED && stupid_dis > 150) || dis < EXITThreshold)
        {
            Position_Control_set_TargetVelocityBodyHeadingXY(0, 0);
            OrderMode = Exit;
        }
        break;
    }

    case TurnSelfCenter:
    {
        float currentYaw = Quaternion_getYaw(get_Airframe_attitude());

        //改进
        if (currentYaw >= target_yaw - PI / 72 && currentYaw <= target_yaw + PI / 72)
        {
            Attitude_Control_set_Target_Yaw(target_yaw);
            Position_Control_set_TargetPositionXYRelative(
                current_pos.x - turnyaw_errx + cosf(target_yaw) * FlightR - get_position_t265().x,
                current_pos.y - turnyaw_erry + sinf(target_yaw) * FlightR - get_position_t265().y);
            OrderMode = Exit;
        }
        else
        {
            if (++div_count > fre_5Hz)
            {
                div_count = 0;
                Position_Control_set_TargetPositionXYRelative(
                    current_pos.x - turnyaw_errx + cosf(currentYaw) * FlightR - get_position_t265().x,
                    current_pos.y - turnyaw_erry + sinf(currentYaw) * FlightR - get_position_t265().y);
            }
        }
        break;
    }
    case Circlrmode:
    {
        float currentYaw = Quaternion_getYaw(get_Airframe_attitude());
        float center_yaw = currentYaw + CirCleYaw_err;
        if (center_yaw > M_PI)
            center_yaw -= 2 * M_PI;
        else if (center_yaw < -M_PI)
            center_yaw += 2 * M_PI;

        if (receive_QR || receive_QRType || (++time_count > 200 && currentYaw >= target_yaw - PI / 72 && currentYaw <= target_yaw + PI / 72))
        { // time original:200

            //					Position_Control_set_TargetPositionXYRelative(
            //					current_pos.x - cosf(target_yaw+CirCleYaw_err)*CircleRadiu - get_position_t265().x,
            //					current_pos.y - sinf(target_yaw+CirCleYaw_err)*CircleRadiu - get_position_t265().y
            //					);
            if (receive_QR || receive_QRType)
            {
                time_count = 0;
                OrderMode = Exit;
            }
            else
            {
                time_count = 0;
                Position_Control_set_TargetPositionXYRelative(0, 0);
                Attitude_Control_set_Target_Yaw(target_yaw);
                OrderMode = Exit;
            }
        }
        else
        {
            if (++div_count > fre_5Hz)
            {
                div_count = 0;
                Position_Control_set_TargetPositionXYRelative(
                    current_pos.x - cosf(center_yaw + CirCleDelayYaw_err) * CircleRadiu - get_position_t265().x,
                    current_pos.y - sinf(center_yaw + CirCleDelayYaw_err) * CircleRadiu - get_position_t265().y);
            }
        }
        break;
    }
    case LandByUltrasonic:
    {
        // if (modedelay(&M37_SHU, 5, 17))
        if (++div_count > fre_5Hz)
        {
            div_count = 0;
            // if (ultrasonic && (ultrasonic < 5.f && get_VelocityENU().z > 30))
            if (ultrasonic && get_height_asonic() < 8)
            {
                OrderMode = Exit;
                set_inFlight_false();
            }
        }
        break;
    }
    case LandByT265:
    {
        static uint8_t LandFlag = 0;
        float Z_recheck = 60.f; //重新校准xy方向的高度
        float dx, dy;
        float dis;
        float ang;
        float vel, velx, vely;

        if (++div_count > fre_5Hz)
        {
            div_count = 0;
            switch (LandFlag)
            {
            case 0:
            {
                Position_Control_set_TargetVelocityZ(-20);
                if (pos_vision.z < Z_recheck)
                {
                    LandFlag++;
                    // Position_Control_set_TargetVelocityZ(0);
                    Position_Control_set_ZLock();
                }
                break;
            }
            case 1:
            {
                // alpha = C ; 待测
                // dx = receive_x * alpha;
                // dy = receive_y * alpha;8
                dx = TgtPos.x - pos_vision.x;
                dy = TgtPos.y - pos_vision.y;
                dis = sqrt(dx * dx + dy * dy);
                ang = atan2(dy, dx);
                vel = Func_Dis2Vel(dis) - 5;
                velx = vel * cosf(ang);
                vely = vel * sinf(ang);
                // Position_Control_set_TargetVelocityZ(0);
                Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
                if (dis < EXITThreshold)
                {
                    // Position_Control_set_TargetVelocityBodyHeadingXY(0, 0);
                    Position_Control_set_XYLock();
                    LandFlag++;
                }
                break;
            }
            case 2:
            {
                Position_Control_set_TargetVelocityZ(-30);
                if (pos_vision.z <= 5.f)
                {
                    OrderMode = Exit;
                    set_inFlight_false();
                }
                break;
            }
            default:
                break;
            }
        }
        break;
    }
    case LandByImage:
    {
        static int LandFlag = -1;
        float Z_recheck = 40.f; //重新校准xy方向的高度
        float kx1 = 0.5, ky1 = 0.5;
        float kx2 = 0.2, ky2 = 0.2;
        float dx = 0, dy = 0;
        float dis;
        float ang;
        float vel, velx, vely;

        Buzzer(false);
        if (++div_count > fre_5Hz)
        {
            div_count = 0;
            if (CAM_DETECTED)
            {
                switch (LandFlag)
                {
                case -1:
                {
                    // Ctrl_SetLockHeightType(LockHeightType_asonic);
                    Ctrl_unLockHeight();
                    Position_Control_set_TargetVelocityZ(-5);
                    dx = CAM_dx * kx1;
                    dy = CAM_dy * ky1;
                    dis = sqrt(dx * dx + dy * dy);
                    ang = atan2(dy, dx);
                    vel = Func_Dis2Vel(dis) - 5;
                    velx = vel * cosf(ang);
                    vely = vel * sinf(ang);
                    // Position_Control_set_TargetVelocityZ(0);
                    Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
                    if (dis < EXITThreshold)
                    {
                        // Position_Control_set_TargetVelocityBodyHeadingXY(0, 0);
                        Position_Control_set_XYLock();
                        LandFlag++;
                    }
                    if (get_height_asonic() <= 5.f)
                    {
                        OrderMode = Exit;
                        set_inFlight_false();
                    }
                    break;
                }
                case 0:
                {
                    Ctrl_unLockHeight();
                    // Ctrl_LockHeight(Z_recheck);
                    Position_Control_set_TargetVelocityZ(-30);
                    if (get_height_asonic() <= 5.f)
                    {
                        OrderMode = Exit;
                        set_inFlight_false();
                    }
                    else if (get_height_asonic() < Z_recheck + 5)
                    {
                        LandFlag++;
                        Position_Control_set_TargetVelocityZ(0);
                        Ctrl_SetLockHeightType(Height_asonic);
                        Ctrl_LockHeight(Z_recheck);
                    }
                    break;
                }
                case 1:
                {
                    // alpha = C ; 待测
                    // dx = receive_x * alpha;
                    // dy = receive_y * alpha;8
                    // Ctrl_LockHeight(Z_recheck);
                    dx = CAM_dx * kx2;
                    dy = CAM_dy * ky2;
                    dis = sqrt(dx * dx + dy * dy);
                    ang = atan2(dy, dx);
                    vel = Func_Dis2Vel(dis) - 5;
                    velx = vel * cosf(ang);
                    vely = vel * sinf(ang);
                    // Position_Control_set_TargetVelocityZ(0);
                    Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
                    if (dis < EXITThreshold)
                    {
                        // Position_Control_set_TargetVelocityBodyHeadingXY(0, 0);
                        // Position_Control_set_XYLock();
                        LandFlag++;
                    }
                    break;
                }
                case 2:
                {
                    Ctrl_unLockHeight();
                    Position_Control_set_TargetVelocityZ(-30);
                    dx = CAM_dx * kx2;
                    dy = CAM_dy * ky2;
                    dis = sqrt(dx * dx + dy * dy);
                    ang = atan2(dy, dx);
                    vel = Func_Dis2Vel(dis) - 5;
                    velx = vel * cosf(ang);
                    vely = vel * sinf(ang);
                    // Position_Control_set_TargetVelocityZ(0);
                    Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
                    if (get_height_asonic() <= 5.f)
                    {
                        OrderMode = Exit;
                        set_inFlight_false();
                    }
                    break;
                }
                default:
                    break;
                }
            }
        }
        break;
    }
    case TakeOffWithInit:
    {
        static uint8_t takeoffstate = 0;
        static uint8_t delaystate = 0;
        switch (takeoffstate)
        {
        case 0:
        {
            switch (delaystate)
            {
            case 0:
                Led_setSignal(LED_signal_success);
                if (modedelay(&M37_SHU, time_delay, 32))
                    delaystate++;
                break;
            case 1:
                Led_setSignal(LED_signal_error);
                if (modedelay_s(5))
                    delaystate++;
                break;
            case 2:
                Buzzer(false);
                bool mystate = Position_Control_Takeoff_HeightRelative(tgt_height);
                if (!mystate)
                    Led_setSignal(LED_signal_2);
                X_offset = pos_vision.x;
                Y_offset = pos_vision.y;
                Z_offset = pos_vision.z;
                Pole_x = Pole_x - X_offset;
                Pole_y = Pole_y - X_offset;
                takeoffstate++;
                delaystate++;
                break;
            default:
                break;
            }
            break;
        }
        case 1:
        {
            if (fabsf(pos_vision.z - tgt_height) < EXITThreshold)
            {
                Position_Control_set_TargetVelocityZ(0);
                OrderMode = Exit;
            }
            else
            {
                // Position_Control_set_TargetVelocityZ(tgt_height - pos_vision.z);
                Position_Control_set_TargetVelocityZ(constrain_float((tgt_height - pos_vision.z), 60));
            }
            break;
        }
        default:
            break;
        }
        break;
    }
    case TakeOffWithDelay:
    {
        static uint8_t takeoffstate = 0;
        static uint8_t delaystate = 0;
        switch (takeoffstate)
        {
        case 0:
            switch (delaystate)
            {
            case 0:
                Led_setSignal(LED_signal_success);
                if (modedelay(&M37_SHU, time_delay, 34))
                    delaystate++;
                break;
            case 1:
                Led_setSignal(LED_signal_error);
                if (modedelay_s(2))
                    delaystate++;
                break;
            case 2:
                Buzzer(false);
                bool mystate = Position_Control_Takeoff_HeightRelative(tgt_height);
                if (!mystate)
                    Led_setSignal(LED_signal_2);
                takeoffstate++;
                delaystate++;
                break;
            default:
                break;
            }
            break;
        case 1:
        {
            if (fabsf(pos_vision.z - tgt_height) < EXITThreshold)
            {
                Position_Control_set_TargetVelocityZ(0);
                OrderMode = Exit;
            }
            else
            {
                Position_Control_set_TargetVelocityZ(constrain_float((tgt_height - pos_vision.z), 60));
                // Position_Control_set_TargetVelocityZ(tgt_height - pos_vision.z);
            }
            // if (get_Altitude_ControlMode() == Position_ControlMode_Position)
            // {
            //     Ctrl_LockHeight(tgt_height);
            // }
            // if (fabsf(pos_vision.z - tgt_height) < EXITThreshold)
            // {
            //     OrderMode = Exit;
            // }
            break;
        }

        default:
            break;
        }
        break;
    }
    case SetHeightWithCheck:
    {
        vector3_float pos_now = get_position_t265();
        float dis_x = TgtPos.x - pos_now.x;
        float dis_y = TgtPos.y - pos_now.y;
        float dis = sqrtf(dis_x * dis_x + dis_y * dis_y);
        float ang = atan2(dis_y, dis_x);
        float vel = Func_Dis2Vel(dis);
        if (dis > MaxPosLimit)
        {
            div_count = 0;
            velx = vel * cosf(ang);
            vely = vel * sinf(ang);
            Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
        }
        else
        {
            vel -= 5;
            velx = vel * cosf(ang);
            vely = vel * sinf(ang);
            Position_Control_set_TargetVelocityBodyHeadingXY(velx, vely);
            // Position_Control_set_TargetPositionXYRelative(dis_x, dis_y);
        }
        if (IsDirUp)
        {
            if (get_height_t265() > tgt_height - 5)
            {
                Position_Control_set_TargetVelocityZ(0);
                Ctrl_LockHeight(tgt_height);
                OrderMode = Exit;
            }
        }
        else
        {
            if (get_height_t265() < tgt_height + 5)
            {
                Position_Control_set_TargetVelocityZ(0);
                Ctrl_LockHeight(tgt_height);
                OrderMode = Exit;
            }
        }
        break;
    }
    case TakeOff45degree:
    {
        static uint8_t takeoff45state = 0;
        switch (takeoff45state)
        {
        case 0:
        {
            Buzzer(true);
            if (modedelay_s(2))
            {
                Buzzer(false);
                bool mystate = Position_Control_Takeoff_HeightRelative(20.f);
                if (!mystate)
                    Led_setStatus(LED_status_error);

                takeoff45state++;
            }
            break;
        }
        case 1:
        {
            if (get_Altitude_ControlMode() == Position_ControlMode_Position)
            {
                takeoff45state++;
            }
            break;
        }
        case 2:
        {
            Position_Control_set_TargetVelocityZ(20.f);
            Position_Control_set_TargetVelocityBodyHeadingXY(0, 20.f);
            takeoff45state++;
            break;
        }
        case 3:
        {
            if (get_height_t265() > tgt_height)
            {
                Position_Control_set_ZLock();
                Position_Control_set_XYLock();
                OrderMode = Exit;
                takeoff45state = 0;
            }
            break;
        }
        }
        break;
    }
    default:
    {
        Led_setStatus(LED_status_error);
        OrderMode = Exit;
        break;
    }
    }
}

void OrderSet_PositionWithCheck(float tgtx, float tgty, uint8_t limit)
{
    IsInflightOrder = true;
    OrderMode = SetPositionWithCheck;

    TgtPos.x = tgtx;
    TgtPos.y = tgty;
    MaxPosLimit = limit;
}

void OrderSet_PositionWithCheck_WXJ(float tgtx, float tgty, uint8_t limit)
{
    IsInflightOrder = true;
    OrderMode = SetPositionWithCheck_WXJ;

    TgtPos.x = tgtx;
    TgtPos.y = tgty;
    MaxPosLimit = limit;
}

void OrderSet_PositionWithCheck_IMG(float tgtx, float tgty, uint8_t limit)
{
    IsInflightOrder = true;
    OrderMode = SetPositionWithCheck_IMG;

    TgtPos.x = tgtx;
    TgtPos.y = tgty;
    MaxPosLimit = limit;
}

void OrderSet_TakeOffWithInit(float myHeight, uint32_t delay)
{
    IsInflightOrder = true;
    OrderMode = TakeOffWithInit;
    Height = pos_vision.z;
    time_delay = delay;
    tgt_height = myHeight;
}

void OrderSet_TakeOffWithDelay(float myHeight, uint32_t delay)
{
    IsInflightOrder = true;
    OrderMode = TakeOffWithDelay;
    time_delay = delay;
    tgt_height = myHeight;
}

void OrderSet_PositionWithCheckRelative(float tgtx, float tgty, uint8_t limit)
{
    OrderSet_PositionWithCheck(get_position_t265().x + tgtx, get_position_t265().y + tgty, limit);
}
void OrderSet_PositionWithCheckRelative_BodyHeading(float tgtx, float tgty, uint8_t limit)
{
    vector3_float current_pos = get_position_t265();
    float Yaw = Quaternion_getYaw(get_Airframe_attitude());
    float Yaw_sin, Yaw_cos;
    arm_sin_cos_f32(rad2degree(Yaw), &Yaw_sin, &Yaw_cos);
    float posx_ENU = map_BodyHeading2ENU_x(tgtx, tgty, Yaw_sin, Yaw_cos);
    float posy_ENU = map_BodyHeading2ENU_y(tgtx, tgty, Yaw_sin, Yaw_cos);
    OrderSet_PositionWithCheck(current_pos.x + posx_ENU, current_pos.y + posy_ENU, limit);
}

void OrderSet_TurnSelfCenter(float tgtyaw, float yaw_rate)
{
    IsInflightOrder = true;
    OrderMode = TurnSelfCenter;

    Attitude_Control_set_Target_YawRate(yaw_rate);
    target_yaw = tgtyaw;
    float currentYaw = Quaternion_getYaw(get_Airframe_attitude());
    current_pos = get_position_t265();
    turnyaw_errx = 1.5 * cosf(currentYaw) * FlightR, turnyaw_erry = 1.5 * sinf(currentYaw) * FlightR;
}

void OrderSet_Circlrmode(float tgtyaw, float yaw_rate, float radiu, float yaw_err)
{
    IsInflightOrder = true;
    OrderMode = Circlrmode;
    //消除双目安装偏差
    float currentYaw = Quaternion_getYaw(get_Airframe_attitude());
    //	turnyaw_errx = cosf(currentYaw)*FlightR, turnyaw_erry = sinf(currentYaw)*FlightR;
    //根据半径radiu和圆心对飞机的偏移量yaw_err ,求出圆心坐标（yaw_err,radiu相当于极坐标的 斯塔 和 r 。都是bodyheading坐标系）
    float center_yaw = Quaternion_getYaw(get_Airframe_attitude()) + yaw_err;
    if (center_yaw > M_PI)
        center_yaw -= 2 * M_PI;
    else if (center_yaw < -M_PI)
        center_yaw += 2 * M_PI;
    //	current_pos.x = get_position_t265().x - turnyaw_errx + cosf(center_yaw)*(radiu);
    //	current_pos.y = get_position_t265().y - turnyaw_erry + sinf(center_yaw)*(radiu);
    current_pos.x = get_position_t265().x + cosf(center_yaw) * (radiu);
    current_pos.y = get_position_t265().y + sinf(center_yaw) * (radiu);
#define DelayYaw_err 2 * M_PI / 16
    if (yaw_rate > 0)
        CirCleDelayYaw_err = DelayYaw_err;
    else
        CirCleDelayYaw_err = -DelayYaw_err;

    Attitude_Control_set_Target_YawRate(yaw_rate);
    target_yaw = tgtyaw;
    CircleRadiu = radiu;
    CirCleYaw_err = yaw_err;
}

void OrderSet_LandByUltrasonic()
{
    IsInflightOrder = true;
    OrderMode = LandByUltrasonic;

    Position_Control_set_XYLock();
    Position_Control_set_TargetVelocityZ(-20);
    Ctrl_unLockHeight();
}

void OrderSet_LandByT265(float tgtx, float tgty)
{
    IsInflightOrder = true;
    OrderMode = LandByT265;
    TgtPos.x = tgtx;
    TgtPos.y = tgty;
    Position_Control_set_XYLock();
    Ctrl_unLockHeight();
}

void OrderSet_LandByImage(void)
{
    IsInflightOrder = true;
    OrderMode = LandByImage;
}

void OrderSet_HeightWithCheck(float x, float y, float myheight, float myvelosity)
{
    IsInflightOrder = true;
    OrderMode = SetHeightWithCheck;

    //解锁高度锁定
    Ctrl_unLockHeight();

    if (myvelosity < 0)
        myvelosity = -myvelosity;
    if (myheight < get_height_t265())
    {
        myvelosity = -myvelosity;
        IsDirUp = false;
    }
    else
    {
        IsDirUp = true;
    }

    Position_Control_set_TargetVelocityZ(myvelosity);
    tgt_height = myheight;
    TgtPos.x = x;
    TgtPos.y = y;
}

void OrderSet_TakeOff45degree(float myHeight)
{
    IsInflightOrder = true;
    OrderMode = TakeOff45degree;
    time_count = 0;

    Height = get_Position().z;
    Height_t265 = get_position_t265().z;
    Height_asonic = ultrasonic;
    tgt_height = myHeight;
}

float Func_Dis2Vel(float distance)
{
    float Kp = 0.215f;
    float bias = 15.f;
    float Vel = Kp * distance + bias;
    // return (Vel > Vel_MAX) ? Vel_MAX : Vel;
    return Vel > Vel_MAX ? Vel_MAX : Vel;
}

void Ctrl_LockHeight(float myHeight)
{
    IsLockHeight = true;
    Lock_Height = myHeight;
}

void Ctrl_unLockHeight()
{
    IsLockHeight = false;
}
void Ctrl_SetLockHeightType(uint8_t type)
{
    LockHeightType = type;
}

bool Get_Guided_Mode_Enabled(void)
{
    return guided_enabled;
}
bool Get_POS_Control_Enabled(void)
{
    return pos_enabled;
}
void Set_POS_Control_Enabled(bool value)
{
    pos_enabled = value;
}