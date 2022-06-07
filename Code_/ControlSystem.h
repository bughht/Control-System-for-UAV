#pragma once

#include <stdbool.h>

/*观测器*/
	//获取悬停油门
	float get_hover_throttle();
	//获取是否在飞行
	bool get_is_inFlight();
	//取消飞行
	void set_inFlight_false();
	//获取ENU x风力（造成的加速度cm/s^2）
	float get_WindDisturbance_x();
	//获取ENU y风力（造成的加速度cm/s^2）
	float get_WindDisturbance_y();
/*观测器*/

/*姿态控制*/
	typedef enum
	{
		Attitude_ControlMode_Angle ,
		Attitude_ControlMode_AngularRate ,
		Attitude_ControlMode_Locking ,
	}Attitude_ControlMode;
	
	//打开关闭姿态控制器
	bool Attitude_Control_Enable();
	bool Attitude_Control_Disable();

	//获取当前油门
	float get_Target_Throttle();
	//设定油门
	bool Attitude_Control_set_Throttle( float thr );
	//获取目标Roll
	float Attitude_Control_get_Target_Roll();	//rad
	//获取目标Pitch
	float Attitude_Control_get_Target_Pitch();	//rad
	//获取目标Yaw
	float Attitude_Control_get_Target_Yaw();	//rad
	//设定目标Roll Pitch
	bool Attitude_Control_set_Target_RollPitch( float Roll , float Pitch ); //rad

	//设定目标Yaw
	bool Attitude_Control_set_Target_Yaw( float Yaw );
	bool Attitude_Control_set_Target_YawRelative( float Yaw );
	//设定目标Yaw速度
	bool Attitude_Control_set_Target_YawRate( float YawRate );
	//锁定Yaw（刹车后锁角度）
	bool Attitude_Control_set_YawLock();
/*姿态控制*/
	
/*位置控制*/
	typedef enum
	{				
		Position_ControlMode_Velocity = 11 ,	//速度控制模式
		Position_ControlMode_Locking = 10 ,	//刹车后锁位置
		
		Position_ControlMode_RouteLine = 20 ,	//巡线模式
		Position_ControlMode_Position = 21 ,	//位置锁定模式
		Position_ControlMode_Takeoff = 22 ,	//起飞模式
	}Position_ControlMode;
	#define Is_PositionControlMode(x) (x >=20 && x<=29)
	
	//打开关闭高度控制器
	bool Altitude_Control_Enable();
	bool Altitude_Control_Disable();
	
	/*高度*/
		//设定目标高度（直接进位置锁定模式）
		bool Position_Control_set_TargetPositionZ( float posz );
		bool Position_Control_set_TargetPositionZRelative( float posz );
		//设定目标垂直速度
		bool Position_Control_set_TargetVelocityZ( float velz );
		//刹车后锁高度（先减速再进位置锁定模式）
		bool Position_Control_set_ZLock();
	
		//获取当前高度控制模式
		Position_ControlMode get_Altitude_ControlMode();
	
		//起飞到当前高度上方的height高度
		bool Position_Control_Takeoff_HeightRelative( float height );
	/*高度*/
	
	/*水平位置*/
		//打开关闭水平位置控制
		bool get_Position_Control_Enabled();
		bool Position_Control_Enable();
		bool Position_Control_Disable();
		
		//获取当前水平位置控制模式
		Position_ControlMode get_Position_ControlMode();
	
		//设定目标水平位置 //ENU
		bool Position_Control_set_TargetPositionXY( float posx , float posy );
		//设定目标水平位置（相对当前坐标） //ENU
		bool Position_Control_set_TargetPositionXYRelative( float posx , float posy );
		bool Position_Control_move_PositionXYRelative( float posx , float posy );
		//设定目标水平位置（相对当前坐标且偏移在Bodyheading系下）
		bool Position_Control_set_TargetPositionXYRelativeBodyHeading( float posx , float posy );//FLU
		bool Position_Control_move_TargetPositionXYRelativeBodyHeading( float posx , float posy );
		//根据经纬度设定目标水平位置
		bool Position_Control_set_TargetPositionXY_LatLon( double Lat , double Lon );
		
		//设定目标水平速度（Bodyheading朝向）
		bool Position_Control_set_TargetVelocityBodyHeadingXY( float velx , float vely );
		bool Position_Control_set_TargetVelocityBodyHeadingXY_WXJ( float velx , float vely );
		//设定目标水平速度（Bodyheading朝向）并限制最大角度（补偿风力后的角度）
		bool Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( float velx , float vely , float maxRoll , float maxPitch );
		//刹车后锁定水平位置
		bool Position_Control_set_XYLock();
	/*水平位置*/
/*位置控制*/