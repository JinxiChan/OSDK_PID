#ifndef PID_H
#define PID_H
#include <stdint.h>

typedef struct 
{
	const char* name;
	float ep;
	float ei;
	float ed;
	float kp;
	float ki;
	float kd;
	float min;
	float max;
	float output;
	float p_value,i_value,d_value;
	uint32_t tick;
}PID_t;

extern PID_t pid_vel_x,pid_vel_y,pid_vel_z,pid_angl_pitch,pid_angl_roll,pid_angl_yaw;
float pid_cal(PID_t *pid,float tgt_val,float cur_val);

/*
 *��xyz�ٶ�(ground����ϵ),yaw�ؽ��ٶ�
 *�ɵ������v->control->velocityAndYawRateCtrl����ʹ��
 */
void ctrl_ground_velocity_and_yawrate(float tgt_vx,float tgt_vy,float tgt_vz,float yawrate);
#endif //PID_H
