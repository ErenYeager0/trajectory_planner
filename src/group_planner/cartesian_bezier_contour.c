/*
 * cartesian_line_t_planner.c
 *
 *  Created on: 2020Äê9ÔÂ11ÈÕ
 *      Author: Eren
 *      description: only plan contour without posture,Example 8.20
 */
#include <path_planner.h>
#include "type_define.h"
#include "group_planner.h"

static void target_init(AXIS_GROUP_INFO *p_axis_group_info);

void bezier_contour_test()
{
	AXIS_GROUP_INFO axis_group_test;
	VECTOR_3D join_point1,join_point2;
	VECTOR_3D traj_point;
	static VECTOR_3D last_traj_point;
	double u =0.0;
	const int step_max = 100;
	int i;

	target_init(&axis_group_test);

	/*
	 * start -> join_point1 -> target_pos[0] -> join_point2 -> target_pos[1]
	 *
	 * */
	get_track_join(&(axis_group_test.start_pos.trans),
			       &(axis_group_test.target_pos[0].trans),
				   1,
				   &join_point1);

	get_track_join(&(axis_group_test.target_pos[1].trans),
			       &(axis_group_test.target_pos[0].trans),
				   1,
				   &join_point2);


	for (i = 0; i < step_max; i++)
	{
		u += 1.0/step_max;

		line_path(&(axis_group_test.start_pos),
				  &join_point1,
				  &traj_point,
				  u);
		printf("%f,%f,%f\n", traj_point.x, traj_point.y, traj_point.z);
	}
	u = 0;

	for (i = 0; i < step_max; i++)
	{
		u += 1.0/step_max;
		beizier_path_4degree(&join_point1,
							 &(axis_group_test.target_pos[0]),
							 &join_point2,
							 0.5,
							 &traj_point,
							 u);

		printf("%f,%f,%f\n", traj_point.x, traj_point.y, traj_point.z);
	}
	u = 0;

	for (i = 0; i < step_max; i++)
	{
		u += 1.0/step_max;

		line_path(&join_point2,
				  &(axis_group_test.target_pos[1]),
				  &traj_point,
				  u);
		printf("%f,%f,%f\n", traj_point.x, traj_point.y, traj_point.z);
	}
}

static void target_init(AXIS_GROUP_INFO *p_axis_group_info)
{
	p_axis_group_info->start_pos.trans.x = 0;
	p_axis_group_info->start_pos.trans.y = 0;
	p_axis_group_info->start_pos.trans.z = 0;

	p_axis_group_info->target_pos[0].trans.x = 1;
	p_axis_group_info->target_pos[0].trans.y = 2;
	p_axis_group_info->target_pos[0].trans.z = 1;

	p_axis_group_info->target_pos[1].trans.x = 2;
	p_axis_group_info->target_pos[1].trans.y = 3;
	p_axis_group_info->target_pos[1].trans.z = 0;

	p_axis_group_info->target_pos[2].trans.x = 4;
	p_axis_group_info->target_pos[2].trans.y = 3;
	p_axis_group_info->target_pos[2].trans.z = 0;

	p_axis_group_info->target_pos[3].trans.x = 5;
	p_axis_group_info->target_pos[3].trans.y = 2;
	p_axis_group_info->target_pos[3].trans.z = 2.345;

	p_axis_group_info->target_pos[4].trans.x = 6;
	p_axis_group_info->target_pos[4].trans.y = 0;
	p_axis_group_info->target_pos[4].trans.z = 2;
}

