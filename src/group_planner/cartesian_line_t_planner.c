/*
 * cartesian_line_t_planner.c
 *
 *  Created on: 2020年9月7日
 *      Author: Eren
 */

#include "type_define.h"

static char cartesian_line_init_handle_unit(AXIS_GROUP_INFO *p_axis_group_info)
{
	double delta_abs = 0.0;
	unsigned char rotation_flag = 0;
	double delta_buf[6] = {0};
	double vel_buf = 0;
	double acc_buf = 0;
	unsigned char i;

	double tmax = 5.0;
	double t0max = 5.0;
	double timer = 0.0;
	double x_dis = power(p_axis_group_info->target_pos.trans.x - p_axis_group_info->start_pos.trans.x, 2);
	double y_dis = power(p_axis_group_info->target_pos.trans.y - p_axis_group_info->start_pos.trans.y, 2);
	double z_dis = power(p_axis_group_info->target_pos.trans.z - p_axis_group_info->start_pos.trans.z, 2);

	delta_abs = sqrt(x_dis + y_dis + z_dis);

	p_axis_group_info->delta_pos.roll = p_axis_group_info->target_pos.roll - p_axis_group_info->start_pos.roll;
	p_axis_group_info->delta_pos.pitch = p_axis_group_info->target_pos.pitch - p_axis_group_info->start_pos.pitch;
	p_axis_group_info->delta_pos.yaw = p_axis_group_info->target_pos.yaw - p_axis_group_info->start_pos.yaw;

	if (p_axis_group_info->delta_pos.roll< JT_EPS
	 && p_axis_group_info->delta_pos.pitch < JT_EPS
	 && p_axis_group_info->delta_pos.yaw < JT_EPS)
	{
		rotation_flag = 0;
	}
	else
	{
		rotation_flag = 1;
	}

	if (delta_abs < JT_EPS && rotation_flag == 0)
	{
		p_axis_group_info->delta_pos.trans.x = 0;
		p_axis_group_info->delta_pos.trans.y = 0;
		p_axis_group_info->delta_pos.trans.z = 0;
		p_axis_group_info->delta_pos.roll = 0;
		p_axis_group_info->delta_pos.pitch = 0;
		p_axis_group_info->delta_pos.yaw = 0;

		return 1;
	}
	else
	{
		delta_buf[0] = p_axis_group_info->delta_pos.trans.x = x_dis;
		delta_buf[1] = p_axis_group_info->delta_pos.trans.y = y_dis;
		delta_buf[2] = p_axis_group_info->delta_pos.trans.z = z_dis;
		delta_buf[3] = p_axis_group_info->delta_pos.roll;
		delta_buf[4] = p_axis_group_info->delta_pos.pitch;
		delta_buf[5] = p_axis_group_info->delta_pos.yaw;

		p_axis_group_info->move_flag = 1;
	}


	for (i = 0; i < 5; i++)
	{
		if (i < 3)
		{
			vel_buf = p_axis_group_info->max_dis_vel;
			acc_buf = p_axis_group_info->max_dis_acc;
		}
		else
		{
			vel_buf = p_axis_group_info->max_rot_vel;
			acc_buf = p_axis_group_info->max_rot_acc;
		}

		/*总体运行时间*/
		timer = delta_buf[0]/vel_buf;
		if (timer > tmax)
			tmax = timer;

		/*加速段运行时间*/
		if (timer > vel_buf/acc_buf)
			timer =vel_buf/acc_buf;

		if (timer > t0max) t0max = timer;
	}

	p_axis_group_info->plan_timer[0] = (long)(t0max + 0.999);
	p_axis_group_info->plan_timer[1] = (long)(tmax + 0.999);
	p_axis_group_info->plan_timer[2] = p_axis_group_info->plan_timer[0] + p_axis_group_info->plan_timer[1];

	return 0;
}


