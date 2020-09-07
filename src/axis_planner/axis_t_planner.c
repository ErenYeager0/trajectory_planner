#include "type_define.h"
#include "axis_planner.h"
#include <math.h>

static char axis_init_handle_unit(AXIS_INFO *p_axis_info);
static char axis_acc_handle_unit(AXIS_INFO *p_axis_info, long count);
static char axis_slip_handle_unit(AXIS_INFO *p_axis_info, long count);
static char axis_dec_handle_unit(AXIS_INFO *p_axis_info, long count);
static char axis_pause_handle_unit(AXIS_INFO *p_axis_info, long count);


static double si = 0.0;


const lpCallback axis_t_planner[] = {
/* HANDLE*/
axis_init_handle_unit,
axis_acc_handle_unit,
axis_slip_handle_unit,
axis_dec_handle_unit,
axis_pause_handle_unit,
};

#define DEBUG_SLIP_PAUSE

void axis_planner_test()
{
	AXIS_INFO axis_test;
	long count = 0;
	unsigned char stage_num;
	unsigned char pause_flag = 0;
	char move_status = 0;

	axis_test.max_vel = 5.0;
	axis_test.max_acc = 1.0;

	axis_test.start_pos = 100.0;
	axis_test.target_pos = 10.0;

	axis_init_handle_unit(&axis_test);

	printf("t0:%d,t1:%d,t2:%d\n", axis_test.plan_timer[0],axis_test.plan_timer[1],axis_test.plan_timer[2]);

	for (count = 1; count < axis_test.plan_timer[2]; count++)
	{
		if (count <= axis_test.plan_timer[0])
		{
			if (pause_flag ==1)
			{
				;
			}
			else
			{
				stage_num = (unsigned char)ACC_STAGE;
			}
		}

		if ((count > axis_test.plan_timer[0])
		 && (count <= axis_test.plan_timer[1]))
		{
#ifdef DEBUG_SLIP_PAUSE
			if (count == (axis_test.plan_timer[0] + 5))
			{
				pause_flag = 1;
			}
#endif
			if (pause_flag == 1)
			{
				stage_num = (unsigned char)PAUSE_STAGE;
				count = axis_test.plan_timer[1] + 1;
			}
			else
			{
				stage_num = (unsigned char)SLIP_STAGE;
			}

		}

		if ((count > axis_test.plan_timer[1])
		 && (count < axis_test.plan_timer[2]))
		{
			stage_num = (unsigned char)DEC_STAGE;
		}

		move_status = axis_t_planner[stage_num](&axis_test,count);

		printf("count:%d,stage:%d,%f\n", count, stage_num, axis_test.traj_pos);

		if (move_status == 1)
		{
			pause_flag = 0;
			return;
		}

	}
}

static char axis_init_handle_unit(AXIS_INFO *p_axis_info)
{
	double delta_abs = 0.0;
	double tmax = 5.0;
	double t0max = 5.0;
	double timer = 0.0;

	p_axis_info->delta_pos = p_axis_info->target_pos - p_axis_info->start_pos;
	delta_abs = fabs(p_axis_info->delta_pos);

	if (delta_abs < JT_EPS)
	{
		p_axis_info->delta_pos = 0.0;
		p_axis_info->move_flag = 0;

		return 1;
	}

	if (p_axis_info->delta_pos > 0)
	{
		p_axis_info->move_flag = 1;
	}
	else
	{
		p_axis_info->move_flag = -1;
	}

	/*总体运行时间*/
	timer = delta_abs/(p_axis_info->max_vel);
	if (timer > tmax)
		tmax = timer;

	/*加速段运行时间*/
	if (timer > p_axis_info->max_vel/p_axis_info->max_acc)
		timer = p_axis_info->max_vel/p_axis_info->max_acc;

	if (timer > t0max) t0max = timer;

	p_axis_info->plan_timer[0] = (long)(t0max + 0.999);
	p_axis_info->plan_timer[1] = (long)(tmax + 0.999);
	p_axis_info->plan_timer[2] = p_axis_info->plan_timer[0] + p_axis_info->plan_timer[1];

	return 0;
}

static char axis_acc_handle_unit(AXIS_INFO *p_axis_info, long count)
{
	double percent_delta = 0.0;

	si = si + (double)count/p_axis_info->plan_timer[0];
	percent_delta = 1 - si/p_axis_info->plan_timer[1];
	p_axis_info->traj_pos = p_axis_info->target_pos - p_axis_info->delta_pos*percent_delta;

	return 0;
}

static char axis_slip_handle_unit(AXIS_INFO *p_axis_info, long count)
{
	double percent_delta = 0.0;

	si = si + 1;
	percent_delta = 1 - si/p_axis_info->plan_timer[1];
	p_axis_info->traj_pos = p_axis_info->target_pos - p_axis_info->delta_pos*percent_delta;

	return 0;
}

static char axis_dec_handle_unit(AXIS_INFO *p_axis_info, long count)
{
	double percent_delta = 0.0;

	si = si + 1 - (double)(count - p_axis_info->plan_timer[1])/p_axis_info->plan_timer[0];
	percent_delta = 1 - si/p_axis_info->plan_timer[1];
	p_axis_info->traj_pos = p_axis_info->target_pos - p_axis_info->delta_pos*percent_delta;

	return 0;

}

static char axis_pause_handle_unit(AXIS_INFO *p_axis_info, long count)
{
	double percent_delta = 0.0;

	si = si + 1 - (double)(count - p_axis_info->plan_timer[1])/p_axis_info->plan_timer[0];
	percent_delta = 1 - si/p_axis_info->plan_timer[1];
	p_axis_info->traj_pos = p_axis_info->target_pos - p_axis_info->delta_pos*percent_delta;

	printf("count:%d, percent_delta:%f\n", count, percent_delta);
	if (count == p_axis_info->plan_timer[0])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
