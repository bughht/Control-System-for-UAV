#include "Basic.h"
#include "ctrl_Position.h"

#include "ControlSystem.h"
#include "MeasurementSystem.h"
#include "Configurations.h"
#include "drv_PWMOut.h"
#include "Receiver.h"
#include "Sensors.h"

#include "AC_Math.h"
#include "Quaternion.h"
#include "TD4.h"
#include "Filters_Butter.h"

/*ģʽ������*/
static bool Altitude_Control_Enabled = false;
static bool Position_Control_Enabled = false;

//λ�ÿ���ģʽ
static Position_ControlMode Altitude_ControlMode = Position_ControlMode_Position;
static Position_ControlMode HorizontalPosition_ControlMode = Position_ControlMode_Position;

//����TD4�˲���
static TD4 Target_tracker[3];
//�����ٶȵ�ͨ�˲���
static Filter_Butter4_LP_float TargetVelocityFilter[3];

static vector3_float target_position;
static vector3_float target_velocity;
static float VelCtrlMaxRoll = -1, VelCtrlMaxPitch = -1;

// glw �������
extern int Buzzer_flag;

int Buzzer_flag = 0;

// glw ������
//	extern int IsInTakeoff;
//	int IsInTakeoff = 0;
extern vector3_float pos_vision;
extern float Height, Height_t265;
/*�߶�*/
bool Altitude_Control_Enable()
{
	if (Altitude_Control_Enabled == true ||
		get_Altitude_Measurement_System_Status() != Measurement_System_Status_Ready)
		return false;

	Altitude_ControlMode = Position_ControlMode_Locking;
	Altitude_Control_Enabled = true;
	Attitude_Control_Enable();
	return true;
}
bool Altitude_Control_Disable()
{
	if (Altitude_Control_Enabled == false)
		return false;
	Altitude_Control_Enabled = false;
	Position_Control_Disable();
	return true;
}

Position_ControlMode get_Altitude_ControlMode()
{
	return Altitude_ControlMode;
}

bool Position_Control_set_TargetPositionZ(float posz)
{
	if (Altitude_Control_Enabled == false)
		return false;
	target_position.z = posz;
	if (Is_PositionControlMode(Altitude_ControlMode) == false)
		Target_tracker[2].x1 = get_Position().z;
	Altitude_ControlMode = Position_ControlMode_RouteLine;
	return true;
}
bool Position_Control_set_TargetPositionZRelative(float posz)
{
	return Position_Control_set_TargetPositionZ(get_Position().z + posz);
}
bool Position_Control_set_TargetVelocityZ(float velz)
{
	if (Altitude_Control_Enabled == false)
		return false;
	target_velocity.z = velz;
	Altitude_ControlMode = Position_ControlMode_Velocity;
	return true;
}
bool Position_Control_set_ZLock()
{
	if (Altitude_Control_Enabled == false)
		return false;
	if (Altitude_ControlMode == Position_ControlMode_Velocity)
		Altitude_ControlMode = Position_ControlMode_Locking;
	return true;
}

//��ɵ���ǰ�߶��Ϸ���height�߶�
static float TakeoffHeight;
bool Position_Control_Takeoff_HeightRelative(float height)
{
	if (Altitude_Control_Enabled == false)
		return false;
	if (height < 10)
		return false;
	if (get_is_inFlight() == true)
		return false;

	Altitude_ControlMode = Position_ControlMode_Takeoff;
	TakeoffHeight = height;
	return true;
}
/*�߶�*/

/*ˮƽλ��*/
bool get_Position_Control_Enabled()
{
	return Position_Control_Enabled;
}
bool Position_Control_Enable()
{
	if (Position_Control_Enabled == true ||
		get_Position_Measurement_System_Status() != Measurement_System_Status_Ready)
		return false;

	HorizontalPosition_ControlMode = Position_ControlMode_Locking;
	Position_Control_Enabled = true;
	Altitude_Control_Enable();
	return true;
}
bool Position_Control_Disable()
{
	if (Position_Control_Enabled == false)
		return false;
	Position_Control_Enabled = false;
	return true;
}

Position_ControlMode get_Position_ControlMode()
{
	return HorizontalPosition_ControlMode;
}

//ֱ�߷���ϵ��
static float LineA, LineB, LineC, LineInvA2PB2;
bool Position_Control_set_TargetPositionXY(float posx, float posy)
{
	if (Position_Control_Enabled == false)
		return false;
	target_position.x = posx;
	target_position.y = posy;

	//��ֱ�߷���
	vector3_float current_pos = get_Position();
	LineA = current_pos.y - target_position.y;
	LineB = target_position.x - current_pos.x;
	LineC = current_pos.x * target_position.y - target_position.x * current_pos.y;
	float A2PB2 = LineA * LineA + LineB * LineB;

	if (A2PB2 > 1 * 1)
	{ //���������Ѳ��ģʽ
		LineInvA2PB2 = 1.0f / A2PB2;
		HorizontalPosition_ControlMode = Position_ControlMode_RouteLine;
	}
	else //�̾������λ������ģʽ
		HorizontalPosition_ControlMode = Position_ControlMode_Position;
	return true;
}
bool Position_Control_move_PositionXYRelative(float posx, float posy)
{
	if (Position_Control_Enabled == false)
		return false;
	target_position.x += posx;
	target_position.y += posy;
	LineC -= LineA * posx + LineB * posy;
	return true;
}
bool Position_Control_set_TargetPositionXYRelative(float posx, float posy)
{
	vector3_float current_pos = get_Position();
	return Position_Control_set_TargetPositionXY(current_pos.x + posx, current_pos.y + posy);
}
bool Position_Control_set_TargetPositionXYRelativeBodyHeading(float posx, float posy)
{
	vector3_float current_pos = get_Position();
	float Yaw = Quaternion_getYaw(get_Airframe_attitude());
	float Yaw_sin, Yaw_cos;
	arm_sin_cos_f32(rad2degree(Yaw), &Yaw_sin, &Yaw_cos);
	float posx_ENU = map_BodyHeading2ENU_x(posx, posy, Yaw_sin, Yaw_cos);
	float posy_ENU = map_BodyHeading2ENU_y(posx, posy, Yaw_sin, Yaw_cos);
	return Position_Control_set_TargetPositionXY(current_pos.x + posx_ENU, current_pos.y + posy_ENU);
}
bool Position_Control_move_TargetPositionXYRelativeBodyHeading(float posx, float posy)
{
	float Yaw = Quaternion_getYaw(get_Airframe_attitude());
	float Yaw_sin, Yaw_cos;
	arm_sin_cos_f32(rad2degree(Yaw), &Yaw_sin, &Yaw_cos);
	float posx_ENU = map_BodyHeading2ENU_x(posx, posy, Yaw_sin, Yaw_cos);
	float posy_ENU = map_BodyHeading2ENU_y(posx, posy, Yaw_sin, Yaw_cos);
	return Position_Control_move_PositionXYRelative(posx_ENU, posy_ENU);
}
bool Position_Control_set_TargetPositionXY_LatLon(double Lat, double Lon)
{
	if (Position_Control_Enabled == false)
		return false;
	float x, y;
	if (get_Point_From_LatLon(&x, &y, Lat, Lon) == false)
		return false;
	return Position_Control_set_TargetPositionXY(x, y);
}

bool Position_Control_set_TargetVelocityBodyHeadingXY(float velx, float vely)
{
	if (Position_Control_Enabled == false)
		return false;

	//���ٶ���ת��ENU
	float Yaw = Quaternion_getYaw(get_Airframe_attitude());
	float Yaw_sin, Yaw_cos;
	arm_sin_cos_f32(rad2degree(Yaw), &Yaw_sin, &Yaw_cos);
	float velx_ENU = map_BodyHeading2ENU_x(velx, vely, Yaw_sin, Yaw_cos);
	float vely_ENU = map_BodyHeading2ENU_y(velx, vely, Yaw_sin, Yaw_cos);

	target_velocity.x = velx_ENU;
	target_velocity.y = vely_ENU;
	HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
	VelCtrlMaxRoll = VelCtrlMaxPitch = -1;
	return true;
}
bool Position_Control_set_TargetVelocityBodyHeadingXY_WXJ(float velx, float vely)
{
	if (Position_Control_Enabled == false)
		return false;

	//			//���ٶ���ת��ENU
	//			float Yaw = Quaternion_getYaw( get_Airframe_attitude() );
	//			float Yaw_sin , Yaw_cos;
	//			arm_sin_cos_f32( rad2degree(Yaw) , &Yaw_sin , &Yaw_cos );
	//			float velx_ENU = map_BodyHeading2ENU_x( velx , vely , Yaw_sin , Yaw_cos );
	//			float vely_ENU = map_BodyHeading2ENU_y( velx , vely , Yaw_sin , Yaw_cos );
	target_velocity.x = velx;
	target_velocity.y = vely;
	HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
	VelCtrlMaxRoll = VelCtrlMaxPitch = -1;
	return true;
}
bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(float velx, float vely, float maxRoll, float maxPitch)
{
	if (Position_Control_Enabled == false)
		return false;

	//���ٶ���ת��ENU
	float Yaw = Quaternion_getYaw(get_Airframe_attitude());
	float Yaw_sin, Yaw_cos;
	arm_sin_cos_f32(rad2degree(Yaw), &Yaw_sin, &Yaw_cos);
	float velx_ENU = map_BodyHeading2ENU_x(velx, vely, Yaw_sin, Yaw_cos);
	float vely_ENU = map_BodyHeading2ENU_y(velx, vely, Yaw_sin, Yaw_cos);

	target_velocity.x = velx_ENU;
	target_velocity.y = vely_ENU;
	HorizontalPosition_ControlMode = Position_ControlMode_Velocity;
	VelCtrlMaxRoll = maxRoll;
	if (VelCtrlMaxRoll < 0.1f)
		VelCtrlMaxRoll = 0.1f;
	VelCtrlMaxPitch = maxPitch;
	if (VelCtrlMaxPitch < 0.1f)
		VelCtrlMaxPitch = 0.1f;
	return true;
}
bool Position_Control_set_XYLock()
{
	if (Position_Control_Enabled == false)
		return false;
	if (HorizontalPosition_ControlMode == Position_ControlMode_Velocity)
		HorizontalPosition_ControlMode = Position_ControlMode_Locking;
	return true;
}
/*ˮƽλ��*/

/*�Զ�����*/

/*�Զ�����*/

/*ģʽ������*/

void ctrl_Position()
{
	bool inFlight = get_is_inFlight();
	vector3_float Position = get_Position();
	vector3_float VelocityENU = get_VelocityENU();
	vector3_float AccelerationENU = get_AccelerationCtrl();
	//��ȡ�����ת����
	float MotorStartThrottle = Cfg_get_MotorStartingThrottle();

	float Ps = 1.0f;
	float Pv = 2.0f;
	if (Altitude_Control_Enabled)
	{ //�߶ȿ���

		switch (Altitude_ControlMode)
		{
		case Position_ControlMode_Position:
		{ //����λ��
			if (inFlight)
			{
				Target_tracker[2].r2n = Target_tracker[2].r2p = 200;
				TD4_track4(&Target_tracker[2], target_position.z, 1.0f / 200);

				//					Target_tracker[2].r2n = Target_tracker[2].r2p = 30;
				//					TD4_track4( &Target_tracker[2] , target_position.z , 1.0f / 200 );
			}
			else
			{
				//û���ǰ��λ�ÿ���ģʽ
				//��Ҫ���
				TD4_reset(&Target_tracker[2]);
				target_position.z = Target_tracker[2].x1 = Position.z;
				Attitude_Control_set_Throttle(0);
				goto AltCtrl_Finish;
			}
			break;
		}
		case Position_ControlMode_Velocity:
		{ //�����ٶ�
			if (inFlight || target_velocity.z > 0)
				TD4_track3(&Target_tracker[2], target_velocity.z, 1.0f / 200);
			else
			{
				//û����������ٶ�Ϊ��
				//��Ҫ���
				TD4_reset(&Target_tracker[2]);
				Target_tracker[2].x1 = Position.z;
				Attitude_Control_set_Throttle(0);
				goto AltCtrl_Finish;
			}
			break;
		}

		case Position_ControlMode_Takeoff:
		{ //���
			if (inFlight)
			{
				//�����
				//���ƴﵽĿ��߶�
				Target_tracker[2].r2p = 50;
				Target_tracker[2].r2n = 50;
				TD4_track4(&Target_tracker[2], target_position.z, 1.0f / 200);
				if (fabsf(target_position.z - Position.z) < 10 &&
					in_symmetry_range_float(VelocityENU.z, 10.0f) &&
					in_symmetry_range_float(AccelerationENU.z, 50.0f) &&
					in_symmetry_range_float(Target_tracker[2].x2, 0.1f) &&
					in_symmetry_range_float(Target_tracker[2].x3, 0.1f))
				{
					Altitude_ControlMode = Position_ControlMode_Velocity;
					Buzzer_flag = 2;
				}
				else
				{
					Buzzer_flag = 1;
				}
			}
			else
			{
				//δ���
				//�ȴ����
				//					if(IsInTakeoff==1){
				////						target_position.z =  Position.z + (Height_t265+TakeoffHeight-pos_vision.z);
				//						target_position.z =  Height + TakeoffHeight;
				//						Target_tracker[2].x1 = Position.z;
				//						TD4_track3( &Target_tracker[2] , 50 , 1.0f / 200 );
				//					}else{
				target_position.z = Position.z + TakeoffHeight;
				Target_tracker[2].x1 = Position.z;
				TD4_track3(&Target_tracker[2], 50, 1.0f / 200);
				//					}
			}
			break;
		}
		case Position_ControlMode_RouteLine:
		{ //�ɵ�ָ���߶�
			if (inFlight)
			{
				//�����
				//���ƴﵽĿ��߶�
				Target_tracker[2].r2n = Target_tracker[2].r2p = 200;
				TD4_track4(&Target_tracker[2], target_position.z, 1.0f / 200);
				if (fabsf(target_position.z - Position.z) < 10 &&
					in_symmetry_range_float(VelocityENU.z, 10.0f) &&
					in_symmetry_range_float(AccelerationENU.z, 50.0f) &&
					in_symmetry_range_float(Target_tracker[2].x2, 0.1f) &&
					in_symmetry_range_float(Target_tracker[2].x3, 0.1f))
					Altitude_ControlMode = Position_ControlMode_Position;
			}
			else
			{
				//δ���
				//��Ҫ���
				TD4_reset(&Target_tracker[2]);
				Target_tracker[2].x1 = Position.z;
				Attitude_Control_set_Throttle(0);
				goto AltCtrl_Finish;
			}
			break;
		}

		case Position_ControlMode_Locking:
		default:
		{ //��λ�ã����ٵ�0Ȼ����ס�߶ȣ�
			if (inFlight)
			{
				TD4_track3(&Target_tracker[2], 0, 1.0f / 200);
				if (in_symmetry_range_float(VelocityENU.z, 10.0f) && in_symmetry_range_float(AccelerationENU.z, 50.0f) &&
					in_symmetry_range_float(Target_tracker[2].x2, 0.1f) &&
					in_symmetry_range_float(Target_tracker[2].x3, 0.1f))
				{
					target_position.z = Target_tracker[2].x1 = Position.z;
					Altitude_ControlMode = Position_ControlMode_Position;
				}
			}
			else
			{
				Attitude_Control_set_Throttle(0);
				goto AltCtrl_Finish;
			}
			break;
		}
		}

		if (inFlight)
		{
			//���������ٶ�
			float target_velocity_z;
			//������ֱ�ٶȵĵ���
			float Tvz_1;
			if (Target_tracker[2].tracking_mode == 4)
			{
				target_velocity_z = Ps * (Target_tracker[2].x1 - Position.z) + Target_tracker[2].x2;
				Tvz_1 = Ps * (Target_tracker[2].x2 - VelocityENU.z) + Target_tracker[2].x3;
			}
			else
			{
				target_velocity_z = Target_tracker[2].x2;
				Tvz_1 = Target_tracker[2].x3;
			}

			//�����������ٶ�
			float target_acceleration_z = Pv * (target_velocity_z - VelocityENU.z) + Tvz_1;
			target_acceleration_z = Filter_Butter4_LP_float_run(&TargetVelocityFilter[2], target_acceleration_z);
			//���ٶ����
			float acceleration_z_error = target_acceleration_z - AccelerationENU.z;

			//��ȡ���cosin
			float lean_cosin = get_lean_angle_cosin();
			if (lean_cosin < 0.5f)
				lean_cosin = 0.5f;

			//��ȡ��ͣ���� - �����ת����
			float hover_throttle = get_hover_throttle() - MotorStartThrottle;

			//��ͣ����ʱ��
			//����� = mg
			//����Ta���ٶ�ʱ��
			//���������Ϊ��( 1 + Ta/G )
			//����������� =    ��ͣ���� * ( 1 + Ta / G )  +   P*( Ta - a )
			float throttle = hover_throttle * (1.0f + target_acceleration_z / constG) + 0.025f * acceleration_z_error;
			//��ǲ���
			throttle /= lean_cosin;

			//�����޷�
			throttle += MotorStartThrottle;
			if (throttle > 90)
				throttle = 90;
			if (inFlight)
			{
				if (throttle < MotorStartThrottle)
					throttle = MotorStartThrottle;
			}

			//���
			Attitude_Control_set_Throttle(throttle);
		}
		else
		{
			//û���
			//���������������
			float current_throttle = get_Target_Throttle();
			if (current_throttle < MotorStartThrottle)
				current_throttle = MotorStartThrottle;
			//			if (IsInTakeoff == 1){
			//				Attitude_Control_set_Throttle( current_throttle );
			//			}else{
			//				Attitude_Control_set_Throttle( current_throttle + 1.0f/200 * 10 );
			Attitude_Control_set_Throttle(current_throttle + 1.0f / 200 * 25);
			//			}
		}

	} //�߶ȿ���
AltCtrl_Finish:

	if (Position_Control_Enabled)
	{ //ˮƽλ�ÿ���
		if (get_Position_Measurement_System_Status() != Measurement_System_Status_Ready)
		{
			Position_Control_Enabled = false;
			goto PosCtrl_Finish;
		}

		//���ˮƽ�Ƕ����
		float max_angle_err_xy = 4.5f / 200;

		float Tvx_1 = 0, Tvy_1 = 0;
		switch (HorizontalPosition_ControlMode)
		{
		case Position_ControlMode_Position:
		{
			if (inFlight)
			{
				target_velocity.x = Ps * (target_position.x - Position.x) * 1.0f;
				target_velocity.y = Ps * (target_position.y - Position.y) * 1.0f;
				float vel = safe_sqrt_f(target_velocity.x * target_velocity.x + target_velocity.y * target_velocity.y);
				if (vel > 100)
				{
					float scale = 100 / vel;
					target_velocity.x *= scale;
					target_velocity.y *= scale;
				}
				else
				{
					Tvx_1 = Ps * (-VelocityENU.x);
					Tvy_1 = Ps * (-VelocityENU.y);
				}
				max_angle_err_xy = 3.5f / 200;
			}
			else
			{
				//					if (IsInTakeoff == 1){
				//						//glw
				//						target_velocity.x = Ps * ( target_position.x - Position.x );
				//						target_velocity.y = Ps * ( target_position.y - Position.y );
				//						float vel = safe_sqrt_f( target_velocity.x*target_velocity.x + target_velocity.y*target_velocity.y );
				//						if( vel > 100 )
				//						{
				//							float scale = 100 / vel;
				//							target_velocity.x *= scale;
				//							target_velocity.y *= scale;
				//						}
				//						else
				//						{
				//							Tvx_1 = Ps * ( -VelocityENU.x );
				//							Tvy_1 = Ps * ( -VelocityENU.y );
				//						}
				//						max_angle_err_xy = 3.5f / 200;
				//					}else{
				//û���ǰ��λ�ÿ���ģʽ
				//��������λ��
				target_position.x = Position.x;
				target_position.y = Position.y;
				Attitude_Control_set_Target_RollPitch(0, 0);
				return;
				//					}
			}
			break;
		}
		case Position_ControlMode_Velocity:
		{
			if (inFlight)
			{
				max_angle_err_xy = 6.5f / 200;
			}
			else
			{
				//û���ʱ���������ٶ�
				Attitude_Control_set_Target_RollPitch(0, 0);
				return;
			}
			break;
		}
		case Position_ControlMode_RouteLine:
		{
			if (inFlight)
			{
				vector3_float footpoint;
				footpoint.x = (LineB * LineB * Position.x - LineA * LineB * Position.y - LineA * LineC) * LineInvA2PB2;
				footpoint.y = (LineA * LineA * Position.y - LineA * LineB * Position.x - LineB * LineC) * LineInvA2PB2;
				//ѹ�߿���
				vector3_float target_anti_deviate_velocity;
				target_anti_deviate_velocity.x = Ps * (footpoint.x - Position.x);
				target_anti_deviate_velocity.y = Ps * (footpoint.y - Position.y);
				float vel = safe_sqrt_f(target_anti_deviate_velocity.x * target_anti_deviate_velocity.x + target_anti_deviate_velocity.y * target_anti_deviate_velocity.y);
				if (vel > 150)
				{
					float scale = 150 / vel;
					target_anti_deviate_velocity.x *= scale;
					target_anti_deviate_velocity.y *= scale;
				}
				else
				{
					Tvx_1 += Ps * (-target_anti_deviate_velocity.x);
					Tvy_1 += Ps * (-target_anti_deviate_velocity.y);
				}
				//Ŀ�����
				vector3_float target_route_velocity;
				target_route_velocity.x = Ps * (target_position.x - footpoint.x);
				target_route_velocity.y = Ps * (target_position.y - footpoint.y);
				float max_route_velocity = safe_sqrt_f(150 * 150 - vel * vel);
				vel = safe_sqrt_f(target_route_velocity.x * target_route_velocity.x + target_route_velocity.y * target_route_velocity.y);
				if (vel > max_route_velocity)
				{
					float scale = max_route_velocity / vel;
					target_route_velocity.x *= scale;
					target_route_velocity.y *= scale;
				}
				else
				{
					Tvx_1 += Ps * (-target_route_velocity.x);
					Tvy_1 += Ps * (-target_route_velocity.y);
				}
				//�жϵ���Ŀ���
				vector3_float pos_err;
				pos_err.x = target_position.x - Position.x;
				pos_err.y = target_position.y - Position.y;
				if (
					(pos_err.x * pos_err.x + pos_err.y * pos_err.y < 15 * 15) &&
					(VelocityENU.x * VelocityENU.x + VelocityENU.y * VelocityENU.y < 15 * 15) &&
					(AccelerationENU.x * AccelerationENU.x + AccelerationENU.y * AccelerationENU.y < 35 * 35))
					HorizontalPosition_ControlMode = Position_ControlMode_Position;

				target_velocity.x = target_anti_deviate_velocity.x + target_route_velocity.x;
				target_velocity.y = target_anti_deviate_velocity.y + target_route_velocity.y;
				max_angle_err_xy = 3.5f / 200;
			}
			else
			{
				//û���ʱ���������ٶ�
				Attitude_Control_set_Target_RollPitch(0, 0);
				return;
			}
			break;
		}

		case Position_ControlMode_Locking:
		default:
		{ //ɲ����λ��
			if (inFlight)
			{
				max_angle_err_xy = 2.0f / 200;
				target_velocity.x = 0;
				target_velocity.y = 0;
				if (VelocityENU.x * VelocityENU.x + VelocityENU.y * VelocityENU.y < 10 * 10)
				{
					target_position.x = Position.x;
					target_position.y = Position.y;
					HorizontalPosition_ControlMode = Position_ControlMode_Position;
				}
			}
			else
			{
				target_position.x = Position.x;
				target_position.y = Position.y;
				HorizontalPosition_ControlMode = Position_ControlMode_Position;
				Attitude_Control_set_Target_RollPitch(0, 0);
				return;
			}
			break;
		}
		}
		//�����������ٶ�
		float target_acceleration_x = Pv * (target_velocity.x - VelocityENU.x) + Tvx_1;
		target_acceleration_x = Filter_Butter4_LP_float_run(&TargetVelocityFilter[0], target_acceleration_x);
		float target_acceleration_y = Pv * (target_velocity.y - VelocityENU.y) + Tvy_1;
		target_acceleration_y = Filter_Butter4_LP_float_run(&TargetVelocityFilter[1], target_acceleration_y);
		//ȥ�������Ŷ�
		target_acceleration_x -= get_WindDisturbance_x();
		target_acceleration_y -= get_WindDisturbance_y();
		//��ת��Bodyheading
		float Yaw = Quaternion_getYaw(get_Airframe_attitude());
		float Yaw_sin, Yaw_cos;
		arm_sin_cos_f32(rad2degree(Yaw), &Yaw_sin, &Yaw_cos);
		float target_acceleration_x_bodyheading = map_ENU2BodyHeading_x(target_acceleration_x, target_acceleration_y, Yaw_sin, Yaw_cos);
		float target_acceleration_y_bodyheading = map_ENU2BodyHeading_y(target_acceleration_x, target_acceleration_y, Yaw_sin, Yaw_cos);
		//�����Ŷ��Ƕ�
		float WindDisturbance_Bodyheading_x = map_ENU2BodyHeading_x(get_WindDisturbance_x(), get_WindDisturbance_y(), Yaw_sin, Yaw_cos);
		float WindDisturbance_Bodyheading_y = map_ENU2BodyHeading_y(get_WindDisturbance_x(), get_WindDisturbance_y(), Yaw_sin, Yaw_cos);
		float AntiDisturbancePitch = atan2f(-WindDisturbance_Bodyheading_x, constG);
		float AntiDisturbanceRoll = atan2f(WindDisturbance_Bodyheading_y, constG);
		//����Ŀ��Ƕȼ��Ƕȱ仯
		float target_Roll = atan2f(-target_acceleration_y_bodyheading, constG);
		if (HorizontalPosition_ControlMode == Position_ControlMode_Velocity && VelCtrlMaxRoll > 0)
			target_Roll = constrain_range_float(target_Roll, AntiDisturbanceRoll + VelCtrlMaxRoll, AntiDisturbanceRoll - VelCtrlMaxRoll);
		float target_Pitch = atan2f(target_acceleration_x_bodyheading, constG);
		if (HorizontalPosition_ControlMode == Position_ControlMode_Velocity && VelCtrlMaxPitch > 0)
			target_Pitch = constrain_range_float(target_Pitch, AntiDisturbancePitch + VelCtrlMaxPitch, AntiDisturbancePitch - VelCtrlMaxPitch);
		float err_x = target_Roll - Attitude_Control_get_Target_Roll();
		float err_y = target_Pitch - Attitude_Control_get_Target_Pitch();
		//�Ƕȱ仯�޷�
		float VelocityBodyheading_x = map_ENU2BodyHeading_x(VelocityENU.x, VelocityENU.y, Yaw_sin, Yaw_cos);
		float VelocityBodyheading_y = map_ENU2BodyHeading_y(VelocityENU.x, VelocityENU.y, Yaw_sin, Yaw_cos);
		if (
			VelocityENU.x * VelocityENU.x + VelocityENU.y * VelocityENU.y > 150 * 150 &&
			-err_x * VelocityBodyheading_y + err_y * VelocityBodyheading_x < 0)
			max_angle_err_xy = 2.0f / 200;
		constrain_vector2_float(&err_x, &err_y, max_angle_err_xy);
		target_Roll = Attitude_Control_get_Target_Roll() + err_x;
		target_Pitch = Attitude_Control_get_Target_Pitch() + err_y;

		constrain_vector2_float(&target_Roll, &target_Pitch, 0.5f);
		Attitude_Control_set_Target_RollPitch(target_Roll, target_Pitch);

	} //ˮƽλ�ÿ���

PosCtrl_Finish:
	return;
}

void init_ctrl_Position()
{
	//��ʼ������TD4�˲���
	TD4_init(&Target_tracker[0], 15, 15, 15, 15);
	TD4_init(&Target_tracker[1], 15, 15, 15, 15);
	TD4_init(&Target_tracker[2], 2, 5, 50, 60);
	Target_tracker[2].r3p = 900;
	Target_tracker[2].r3n = 600;

	//��ʼ�������ٶȵ�ͨ�˲���
	Filter_Butter4_LP_float_init(&TargetVelocityFilter[0], 200, 10);
	Filter_Butter4_LP_float_init(&TargetVelocityFilter[1], 200, 10);
	Filter_Butter4_LP_float_init(&TargetVelocityFilter[2], 200, 10);
}