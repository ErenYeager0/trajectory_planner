/*
 * cartesian_line_t_planner.c
 *
 *  Created on: 2020年9月7日
 *      Author: Eren
 */
#include <math.h>
#include <stdio.h>
#include "type_define.h"
#include "group_planner.h"

static char cartesian_line_init_handle_unit(AXIS_GROUP_INFO *p_axis_group_info);
static char cartesian_line_acc_handle_unit(AXIS_GROUP_INFO *p_axis_group_info, long count);
static char cartesian_line_slip_handle_unit(AXIS_GROUP_INFO *p_axis_group_info, long count);
static char cartesian_line_dec_handle_unit(AXIS_GROUP_INFO *p_axis_group_info, long count);
static char cartesian_line_pause_handle_unit(AXIS_GROUP_INFO *p_axis_group_info, long count);

static double si = 0.0;

const lpCallback cartesian_line_t_planner[] = {
/* HANDLE*/
		cartesian_line_init_handle_unit,
		cartesian_line_acc_handle_unit,
		cartesian_line_slip_handle_unit,
		cartesian_line_dec_handle_unit,
		cartesian_line_pause_handle_unit
};

void cartesian_line_planner_test()
{
	AXIS_GROUP_INFO axis_group_test;
	long count;
	unsigned char stage_num;
	unsigned char pause_flag = 0;
	char move_status = 0;

	axis_group_test.start_pos.trans.x = 0;
	axis_group_test.start_pos.trans.y = 0;
	axis_group_test.start_pos.trans.z = 0;
	axis_group_test.start_pos.roll    = 0;
	axis_group_test.start_pos.pitch   = 0;
	axis_group_test.start_pos.yaw     = 0;

	axis_group_test.target_pos.trans.x = 100;
	axis_group_test.target_pos.trans.y = 120;
	axis_group_test.target_pos.trans.z = 80;
	axis_group_test.target_pos.roll    = 30;
	axis_group_test.target_pos.pitch   = 20;
	axis_group_test.target_pos.yaw     = 10;

	axis_group_test.max_dis_acc = 0.2;
	axis_group_test.max_dis_vel = 5.0;
	axis_group_test.max_rot_acc = 0.1;
	axis_group_test.max_rot_vel = 5.0;

	if(cartesian_line_init_handle_unit(&axis_group_test) != 0)
	{
		return;
	}

	for (count = 1; count < axis_group_test.plan_timer[2]; count++)
	{
		//ACC STAGE
		if (count <= axis_group_test.plan_timer[0])
		{
			if (pause_flag ==1)
			{
				;
			}
			else
			{
				stage_num = (unsigned char)GROUP_ACC_STAGE;
			}
		}

		//VEL SLIP STAGE
		if ((count > axis_group_test.plan_timer[0])
		 && (count <= axis_group_test.plan_timer[1]))
		{
#ifdef DEBUG_SLIP_PAUSE
			if (count == (axis_test.plan_timer[0] + 5))
			{
				pause_flag = 1;
			}
#endif
			if (pause_flag == 1)
			{
				stage_num = (unsigned char)GROUP_PAUSE_STAGE;
				count = axis_group_test.plan_timer[1] + 1;
			}
			else
			{
				stage_num = (unsigned char)GROUP_SLIP_STAGE;
			}

		}

		//DEC STAGE
		if ((count > axis_group_test.plan_timer[1])
		 && (count < axis_group_test.plan_timer[2]))
		{
			stage_num = (unsigned char)GROUP_DEC_STAGE;
		}

		move_status = cartesian_line_t_planner[stage_num](&axis_group_test,count);

		printf("count:%d,stage:%d,%f,%f,%f,%f,%f,%f\n",
				count,
				stage_num,
				axis_group_test.traj_pos.trans.x,
				axis_group_test.traj_pos.trans.y,
				axis_group_test.traj_pos.trans.z,
				axis_group_test.traj_pos.roll,
				axis_group_test.traj_pos.pitch,
				axis_group_test.traj_pos.yaw);

		if (move_status == 1)
		{
			pause_flag = 0;
			return;
		}
	}


}

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
	double x_dis, x_dis_square;
	double y_dis, y_dis_square;
	double z_dis, z_dis_square;


	//FIXME:why power function is unreference????
	x_dis = p_axis_group_info->target_pos.trans.x - p_axis_group_info->start_pos.trans.x;
	y_dis = p_axis_group_info->target_pos.trans.y - p_axis_group_info->start_pos.trans.y;
	z_dis = p_axis_group_info->target_pos.trans.z - p_axis_group_info->start_pos.trans.z;
	x_dis_square = x_dis*x_dis;
	y_dis_square = y_dis*y_dis;
	z_dis_square = z_dis*z_dis;

	delta_abs = sqrt(x_dis_square + y_dis_square + z_dis_square);

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

		p_axis_group_info->move_flag = 0;

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
		timer = delta_buf[i]/vel_buf;
		if (timer > tmax)
			tmax = timer;

		/*加速段运行时间*/
		if (timer > vel_buf/acc_buf)
			timer = vel_buf/acc_buf;

		if (timer > t0max) t0max = timer;

		printf("%d--delta:%f, t0max:%f, tmax:%f\n", i, delta_buf[i], t0max, tmax);
	}

	p_axis_group_info->plan_timer[0] = (long)(t0max + 0.999);
	p_axis_group_info->plan_timer[1] = (long)(tmax + 0.999);
	p_axis_group_info->plan_timer[2] = p_axis_group_info->plan_timer[0] + p_axis_group_info->plan_timer[1];

	printf("[%s]t0:%d, t1:%d, t2:%d\n",
			__func__,
			p_axis_group_info->plan_timer[0],
			p_axis_group_info->plan_timer[1],
			p_axis_group_info->plan_timer[2]);

	return 0;
}

static char cartesian_line_acc_handle_unit(AXIS_GROUP_INFO *p_axis_group_info, long count)
{
	double percent_delta = 0.0;

	si = si + (double)count/p_axis_group_info->plan_timer[0];
	percent_delta = 1 - si/p_axis_group_info->plan_timer[1];

	p_axis_group_info->traj_pos.trans.x = p_axis_group_info->target_pos.trans.x - p_axis_group_info->delta_pos.trans.x*percent_delta;
	p_axis_group_info->traj_pos.trans.y = p_axis_group_info->target_pos.trans.y - p_axis_group_info->delta_pos.trans.y*percent_delta;
	p_axis_group_info->traj_pos.trans.z = p_axis_group_info->target_pos.trans.z - p_axis_group_info->delta_pos.trans.z*percent_delta;

	p_axis_group_info->traj_pos.roll = p_axis_group_info->target_pos.roll - p_axis_group_info->delta_pos.roll*percent_delta;
	p_axis_group_info->traj_pos.pitch = p_axis_group_info->target_pos.pitch - p_axis_group_info->delta_pos.pitch*percent_delta;
	p_axis_group_info->traj_pos.yaw = p_axis_group_info->target_pos.yaw - p_axis_group_info->delta_pos.yaw*percent_delta;

	return 0;
}


static char cartesian_line_slip_handle_unit(AXIS_GROUP_INFO *p_axis_group_info, long count)
{
	double percent_delta = 0.0;

	si = si + 1;
	percent_delta = 1 - si/p_axis_group_info->plan_timer[1];

	p_axis_group_info->traj_pos.trans.x = p_axis_group_info->target_pos.trans.x - p_axis_group_info->delta_pos.trans.x*percent_delta;
	p_axis_group_info->traj_pos.trans.y = p_axis_group_info->target_pos.trans.y - p_axis_group_info->delta_pos.trans.y*percent_delta;
	p_axis_group_info->traj_pos.trans.z = p_axis_group_info->target_pos.trans.z - p_axis_group_info->delta_pos.trans.z*percent_delta;

	p_axis_group_info->traj_pos.roll = p_axis_group_info->target_pos.roll - p_axis_group_info->delta_pos.roll*percent_delta;
	p_axis_group_info->traj_pos.pitch = p_axis_group_info->target_pos.pitch - p_axis_group_info->delta_pos.pitch*percent_delta;
	p_axis_group_info->traj_pos.yaw = p_axis_group_info->target_pos.yaw - p_axis_group_info->delta_pos.yaw*percent_delta;

	return 0;
}

static char cartesian_line_dec_handle_unit(AXIS_GROUP_INFO *p_axis_group_info, long count)
{
	double percent_delta = 0.0;

	si = si + 1 - (double)(count - p_axis_group_info->plan_timer[1])/p_axis_group_info->plan_timer[0];
	percent_delta = 1 - si/p_axis_group_info->plan_timer[1];

	p_axis_group_info->traj_pos.trans.x = p_axis_group_info->target_pos.trans.x - p_axis_group_info->delta_pos.trans.x*percent_delta;
	p_axis_group_info->traj_pos.trans.y = p_axis_group_info->target_pos.trans.y - p_axis_group_info->delta_pos.trans.y*percent_delta;
	p_axis_group_info->traj_pos.trans.z = p_axis_group_info->target_pos.trans.z - p_axis_group_info->delta_pos.trans.z*percent_delta;

	p_axis_group_info->traj_pos.roll = p_axis_group_info->target_pos.roll - p_axis_group_info->delta_pos.roll*percent_delta;
	p_axis_group_info->traj_pos.pitch = p_axis_group_info->target_pos.pitch - p_axis_group_info->delta_pos.pitch*percent_delta;
	p_axis_group_info->traj_pos.yaw = p_axis_group_info->target_pos.yaw - p_axis_group_info->delta_pos.yaw*percent_delta;

	return 0;

}

static char cartesian_line_pause_handle_unit(AXIS_GROUP_INFO *p_axis_group_info, long count)
{
	return 0;
}
