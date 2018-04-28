#include "pid.h"
#include <math.h>
#include <stdio.h>
#include "dji_vehicle.hpp"
#include "dji_control.hpp"

using namespace DJI;
using namespace DJI::OSDK;
extern Vehicle  vehicle;
extern Vehicle* v;

#define DtoR (double)0.017453292519943
#define RtoD (double)57.295779513082321
#define a3 (v->broadcast)

static Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;
  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;
  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;
  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);
  return ans;
}

//									name     ep  ei  ed   kp     ki    kd  min max  output~tick
PID_t 
pid_vel_x 		={"vel_x PID",	0,	0,	0,	16.0f, 0.0075f,0.0f,-30,30,	0,0,0,0,0},		//输出的是target_vel
pid_vel_y 		={"vel_y PID",	0,	0,	0,	16.0f, 0.0075f,0.0f,-30,30,	0,0,0,0,0},		//输出的是target_vel
pid_vel_z 		={"vel_z PID",	0,	0,	0,	16.0f, 0.0075f,0.0f,-30,30,	0,0,0,0,0},		//输出的是target_vel
pid_angl_pitch={"angl pitch PID",	0,	0,	0,	5.0f,	0.0f,	0.0f,-60,60,	0,0,0,0,0},		//输出的是target_angle_vel
pid_angl_roll	={"angl roll PID",	0,	0,	0,	5.0f,	0.0f,	0.0f,-60,60,	0,0,0,0,0},		//输出的是target_angle_vel
pid_angl_yaw	={"angl yaw PID",	0,	0,	0,	5.0f,	0.0f,	0.0f,-60,60,	0,0,0,0,0};		//输出的是target_angle_vel

float pid_cal(PID_t *pid,float tgt_val,float cur_val)
{
	uint32_t tick = v->protocolLayer->getDriver()->getTimeStamp();
	float e=tgt_val-cur_val;
	if(pid->tick!=0)pid->ed=(e-pid->ep)/(tick-pid->tick)/1000.0f;
	pid->ep=e;
	pid->ei+=pid->ep;
	pid->tick=tick;
	pid->p_value=pid->kp*pid->ep;
	pid->i_value=pid->ki*pid->ei;
	pid->d_value=pid->kd*pid->ed;
	pid->output=pid->p_value+pid->i_value+pid->d_value;
	if(pid->output<pid->min)pid->output=pid->min;
	if(pid->output>pid->max)pid->output=pid->max;
	return pid->output;
}

/*
 *控xyz速度(ground坐标系),yaw控角速度
 *可当做替代v->control->velocityAndYawRateCtrl函数使用
 */
void ctrl_ground_velocity_and_yawrate(float tgt_vx,float tgt_vy,float tgt_vz,float yawrate)
{
			/*环A 通过速度差值计算目标角度(目标角度产生加速度)*/
			float cur_vel_x		= a3->getVelocity().x;
			float tgt_angl_x 	= -pid_cal(&pid_vel_x,tgt_vx,cur_vel_x);
			float cur_vel_y 	= a3->getVelocity().y;
			float tgt_angl_y 	= pid_cal(&pid_vel_y,tgt_vy,cur_vel_y);
			
			/*环B 通过加速度差值计算目标角度*/
			Telemetry::Quaternion broadcastQ = v->broadcast->getQuaternion();
			float cur_angl_x = toEulerAngle((static_cast<void*>(&broadcastQ))).x*RtoD;
			float tgt_rate_x = pid_cal(&pid_angl_pitch,tgt_angl_x,cur_angl_x);
			float cur_angl_y = toEulerAngle((static_cast<void*>(&broadcastQ))).y*RtoD;
			float tgt_rate_y = pid_cal(&pid_angl_roll,tgt_angl_y,cur_angl_y);
			
			/*调用API控制角速度*/
			
			uint8_t ctrl_flag =(DJI::OSDK::Control::VERTICAL_VELOCITY | 
													DJI::OSDK::Control::HORIZONTAL_ANGULAR_RATE |
													DJI::OSDK::Control::YAW_RATE | 
													DJI::OSDK::Control::HORIZONTAL_GROUND);
			//																						roll_rate		pitch_rate
			DJI::OSDK::Control::CtrlData data(ctrl_flag, tgt_rate_y, tgt_rate_x, tgt_vz, yawrate);
			v->control->flightCtrl(data);
			
}

