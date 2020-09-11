/*
 * bezier_curves.c
 *
 *  Created on: 2020Äê9ÔÂ8ÈÕ
 *      Author: Eren
 *      <trajectory planning for automatic machines and robots>
 */
#include <stdio.h>
#include <math.h>
#include "type_define.h"

static int factorial(int n);
static double get_basis_function(unsigned char m, unsigned char j, double u);

// Example B.9 in page 484
void bezier_curves_test()
{
	VECTOR_2D p0,p1,p2,p3;
	VECTOR_2D curve_traject;
	double u = 0.0;
	const unsigned int step_max = 100;
	unsigned int i;


	p0.x = 0; p0.y = 0;
	p1.x = 0; p1.y = 1;
	p2.x = 1; p2.y = 2.5;
	p3.x = 2; p3.y = 3;

	for (i = 0; i < step_max; i++)
	{
		u += 1.0/ (double)step_max;

		curve_traject.x = get_basis_function(3, 0, u)*p0.x\
						+ get_basis_function(3, 1, u)*p1.x\
						+ get_basis_function(3, 2, u)*p2.x\
						+ get_basis_function(3, 3, u)*p3.x;

		curve_traject.y = get_basis_function(3, 0, u)*p0.y\
						+ get_basis_function(3, 1, u)*p1.y\
						+ get_basis_function(3, 2, u)*p2.y\
						+ get_basis_function(3, 3, u)*p3.y;

		printf("%d,%f,%f\n", i, curve_traject.x, curve_traject.y);
	}
}


/*
 * m£º m-th degree Bernstein polynomials
 * j: j = 0, .... m, form the rows of Pascal's triangle
 *
 * */
double get_binomial_coefficients(unsigned char m, unsigned char j)
{
	return (double)factorial(m)/(double)(factorial(j)*factorial(m-j));
}

static double get_basis_function(unsigned char m, unsigned char j, double u)
{
	double binomial_coefficient = get_binomial_coefficients(m, j);

	return binomial_coefficient*pow(u,j)*pow(1-u, m-j);
}

static int factorial(int n)
{
	if (n == 0)
	{
		return 1;
	}
	else
	{
		return n*factorial(n-1);
	}
}

int get_track_join(const VECTOR_3D *p_start, const VECTOR_3D *p_target, double delta, VECTOR_3D *p_join)
{
	VECTOR_3D t;
	double normal;

	//printf("start:%f,%f,%f\n", p_start->x,  p_start->y, p_start->z);
	//printf("target:%f,%f,%f\n", p_target->x,  p_target->y, p_start->z);

	t.x = p_target->x - p_start->x;
	t.y = p_target->y - p_start->y;
	t.z = p_target->z - p_start->z;

	normal = sqrt(t.x*t.x + t.y*t.y +t.z*t.z);
	t.x /= normal;	t.y /= normal; 	t.z /= normal;

	p_join->x = p_start->x + t.x*delta;
	p_join->y = p_start->y + t.y*delta;
	p_join->z = p_start->z + t.z*delta;
	printf("join:%f,%f,%f\n", p_join->x,  p_join->y, p_join->z);
}

int beizier_function_planner_4degree(const VECTOR_3D *p_start, const VECTOR_3D *p_middle, const VECTOR_3D *p_target, double delta, VECTOR_3D *p_trajectory, double u)
{
	VECTOR_3D first_join, second_join;
	static VECTOR_3D a0,a1,a2,a3,a4;
	static int init_flag = 0;

	if (init_flag == 0)
	{
		init_flag = 1;
		get_track_join(p_middle, p_start, delta, &first_join);
		get_track_join(p_middle, p_target, delta, &second_join);

		a0.x =p_start->x;a0.y =p_start->y;a0.z =p_start->z;

		a1.x = -4*p_start->x+4*first_join.x;
		a1.y = -4*p_start->y+4*first_join.y;
		a1.z = -4*p_start->z+4*first_join.z;

		a2.x = 6*p_start->x-12*first_join.x+6*p_middle->x;
		a2.y = 6*p_start->y-12*first_join.y+6*p_middle->y;
		a2.z = 6*p_start->z-12*first_join.z+6*p_middle->z;

		a3.x = -4*p_start->x+12*first_join.x-12*p_middle->x+4*second_join.x;
		a3.y = -4*p_start->y+12*first_join.y-12*p_middle->y+4*second_join.y;
		a3.z = -4*p_start->z+12*first_join.z-12*p_middle->z+4*second_join.z;

		a4.x = 5*p_start->x-4*first_join.x+6*p_middle->x-4*second_join.x+p_target->x;
		a4.y = 5*p_start->y-4*first_join.y+6*p_middle->y-4*second_join.y+p_target->y;
		a4.z = 5*p_start->z-4*first_join.z+6*p_middle->z-4*second_join.z+p_target->z;

	}

	p_trajectory->x = a0.x + a1.x*u+ a2.x*pow(u,2)+a3.x*pow(u,3)+a4.x*pow(u,4);
	p_trajectory->y = a0.y + a1.y*u+ a2.y*pow(u,2)+a3.y*pow(u,3)+a4.y*pow(u,4);
	p_trajectory->z = a0.z + a1.z*u+ a2.z*pow(u,2)+a3.z*pow(u,3)+a4.z*pow(u,4);
}
