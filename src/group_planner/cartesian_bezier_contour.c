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
static void calculate_vel_rate(AXIS_GROUP_INFO *p_axis_group_info, double* p_line_len, double* p_time);

void bezier_contour_test_T_traj()
{
	AXIS_GROUP_INFO axis_group_test;
	VECTOR_3D join_point1,join_point2;
	VECTOR_3D traj_point;
	VECTOR_3D beizier_param[5];
	double u = 0;
	double curve_len,line1_len,line2_len,all_len;
	const double vel_test = 0.01;
	const double acc_test = 0.001;
	int t[10];
	int count = 0;
	double vel,dis;

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

	get_beizier_4degree_param(&join_point1,
							  &(axis_group_test.target_pos[0]),
							  &join_point2,
							  0.5,
							  beizier_param);

	get_bezier_path_4degree_length(beizier_param, 2, &curve_len);
	printf("cureve length : %f\n", curve_len);

	get_line_length(&(axis_group_test.start_pos),
					&join_point1,
					&line1_len);
	printf("line1 length : %f\n", line1_len);

	get_line_length(&join_point2,
					&(axis_group_test.target_pos[1]),
					&line2_len);
	printf("line2 length : %f\n", line2_len);

	all_len = curve_len + line1_len + line2_len;

	printf("all length : %f\n", all_len);

	t[0] = (int)(vel_test/acc_test);
	t[1] = (int)(all_len/vel_test);
	t[2] = t[0] + t[1];

	t[3] = (line1_len - (0.5*t[0]*vel_test))/vel_test + t[0];
	t[4] = curve_len / vel_test;

	printf("t0 = %d, t1 = %d, t3 = %d\n", t[0], t[1], t[3]);

	vel = dis = 0;
	while(1)
	{
		count++;
		if (count <= t[0])
		{
			vel += acc_test;
			dis = 0.5*acc_test*pow(count, 2);
			u = dis/line1_len;
			line_path(&(axis_group_test.start_pos),
					  &join_point1,
					  &traj_point,
					  u);
			printf("L1,%d, %f, %f,%f,%f\n", count, vel, traj_point.x, traj_point.y, traj_point.z);
		}
		else if (count > t[0] && count <= t[3])
		{
			dis += vel;
			u = dis/line1_len;
			line_path(&(axis_group_test.start_pos),
					  &join_point1,
					  &traj_point,
					  u);
			if (count == t[3])
			{
				dis = 0;
			}

			printf("L2,%d, %f, %f,%f,%f\n", count, vel, traj_point.x, traj_point.y, traj_point.z);
		}
		else if (count > t[3] && count <= (t[0] + t[3] + t[4]))
		{


			dis += vel;
			u = dis/curve_len;
			beizier_4degree_path(beizier_param,
								 u,
								 &traj_point);
			if (count == (t[0] + t[3] + t[4]))
			{
				dis = 0;
			}
			printf("C, %d, %f, %f,%f,%f\n", count, vel, traj_point.x, traj_point.y, traj_point.z);
		}
		else if (count > (t[0] + t[3] + t[4]) && count <= t[1])
		{

			dis += vel;
			u = dis/line2_len;
			line_path(&join_point2,
					  &(axis_group_test.target_pos[1]),
					  &traj_point,
					  u);
			printf("L3, %d, %f, %f,%f,%f\n", count, vel, traj_point.x, traj_point.y, traj_point.z);
		}
		else if (count > t[1] && count <= (t[1] + t[0]))
		{
			vel -= acc_test;
			dis += vel;
			u = dis/line2_len;
			line_path(&join_point2,
					  &(axis_group_test.target_pos[1]),
					  &traj_point,
					  u);

			printf("L4, %d, %f, %f,%f,%f\n", count, vel, traj_point.x, traj_point.y, traj_point.z);

		}
		else
		{
			break;

		}

	}
}

void bezier_contour_test_path()
{
	AXIS_GROUP_INFO axis_group_test;
	VECTOR_3D join_point1,join_point2;
	VECTOR_3D traj_point;
	VECTOR_3D beizier_param[5];
	const int step_max = 100;
	double u = 0;
	double curve_len,line1_len,line2_len,all_len;
	const double vel_test = 0.01;
	const double acc_test = 0.001;
	int t[10];
	int i = 0;
	int count = 0;
	double vel,dis;

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

	get_beizier_4degree_param(&join_point1,
							  &(axis_group_test.target_pos[0]),
							  &join_point2,
							  0.5,
							  beizier_param);

	/*
	 * line segment -> curve segment -> line segment
	 *
	 * */
	for (i = 0; i < step_max; i++)
	{
		u += 1.0/step_max;

		line_path(&(axis_group_test.start_pos),
				  &join_point1,
				  &traj_point,
				  u);
		printf("line:%f,%f,%f\n", traj_point.x, traj_point.y, traj_point.z);
	}
	u = 0;

	for (i = 0; i < step_max; i++)
	{
		u += 1.0/step_max;
		beizier_4degree_path(beizier_param,
							 u,
							 &traj_point);

		printf("curv:%f,%f,%f\n", traj_point.x, traj_point.y, traj_point.z);
	}
	u = 0;

	for (i = 0; i < step_max; i++)
	{
		u += 1.0/step_max;

		line_path(&join_point2,
				  &(axis_group_test.target_pos[1]),
				  &traj_point,
				  u);
		printf("line:%f,%f,%f\n", traj_point.x, traj_point.y, traj_point.z);
	}
}

void bezier_contour_test_820()
{
	AXIS_GROUP_INFO axis_group_test;
	VECTOR_3D join_point[8];

	target_init(&axis_group_test);

	get_track_join(&(axis_group_test.target_pos[0].trans),
			       &(axis_group_test.start_pos.trans),
				   0.5,
				   join_point);
	printf("q_1' : %f, %f, %f\n", join_point[0].x, join_point[0].y,join_point[0].z);

	get_track_join(&(axis_group_test.target_pos[0].trans),
			       &(axis_group_test.target_pos[1].trans),
				   0.5,
				   join_point+1);
	printf("q_1'' : %f, %f, %f\n", join_point[1].x, join_point[1].y,join_point[1].z);


}
/*
 * point set from example 8.20
 * */
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
	p_axis_group_info->target_pos[3].trans.z = 2;

	p_axis_group_info->target_pos[4].trans.x = 6;
	p_axis_group_info->target_pos[4].trans.y = 0;
	p_axis_group_info->target_pos[4].trans.z = 2;
}

