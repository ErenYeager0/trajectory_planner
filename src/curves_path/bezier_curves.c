/*
 * bezier_curves.c
 *
 *  Created on: 2020��9��8��
 *      Author: Eren
 *      <trajectory planning for automatic machines and robots>
 */
#include <stdio.h>
#include <math.h>
#include "type_define.h"


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

int get_beizier_4degree_param(const VECTOR_3D *p_start, const VECTOR_3D *p_middle, const VECTOR_3D *p_target, double delta, VECTOR_3D *p_param)
{
	VECTOR_3D first_join, second_join;

	get_track_join(p_middle, p_start, delta, &first_join);
	get_track_join(p_middle, p_target, delta, &second_join);

	p_param[0].x = p_start->x;
	p_param[0].y = p_start->y;
	p_param[0].z = p_start->z;

	p_param[1].x = -4*p_start->x+4*first_join.x;
	p_param[1].y = -4*p_start->y+4*first_join.y;
	p_param[1].z = -4*p_start->z+4*first_join.z;

	p_param[2].x = 6*p_start->x-12*first_join.x+6*p_middle->x;
	p_param[2].y = 6*p_start->y-12*first_join.y+6*p_middle->y;
	p_param[2].z = 6*p_start->z-12*first_join.z+6*p_middle->z;

	p_param[3].x = -4*p_start->x+12*first_join.x-12*p_middle->x+4*second_join.x;
	p_param[3].y = -4*p_start->y+12*first_join.y-12*p_middle->y+4*second_join.y;
	p_param[3].z = -4*p_start->z+12*first_join.z-12*p_middle->z+4*second_join.z;

	//the book about this equation is wrong
	p_param[4].x = p_start->x-4*first_join.x+6*p_middle->x-4*second_join.x+p_target->x;
	p_param[4].y = p_start->y-4*first_join.y+6*p_middle->y-4*second_join.y+p_target->y;
	p_param[4].z = p_start->z-4*first_join.z+6*p_middle->z-4*second_join.z+p_target->z;
}

int beizier_4degree_path( const VECTOR_3D *p_param, double u, VECTOR_3D *p_trajectory)
{
	p_trajectory->x = p_param[0].x + p_param[1].x*u+ p_param[2].x*pow(u,2)+p_param[3].x*pow(u,3)+p_param[4].x*pow(u,4);
	p_trajectory->y = p_param[0].y + p_param[1].y*u+ p_param[2].y*pow(u,2)+p_param[3].y*pow(u,3)+p_param[4].y*pow(u,4);
	p_trajectory->z = p_param[0].z + p_param[1].z*u+ p_param[2].z*pow(u,2)+p_param[3].z*pow(u,3)+p_param[4].z*pow(u,4);

	return 0;
}

/*
 *  from <A Primer on Bezier Curves> --> arc length
 *  and <Gaussian Quadrature Weights and Abscissae>
 *
 * */
static int get_bezier_4degree_seg(const VECTOR_3D *p_param, double u, double *p_len);
int get_bezier_path_4degree_length(const VECTOR_3D *p_param, int n, double *p_curve_len)
{

	double u;
	double len_seg[100];

	switch (n)
	{
		case 2:
			u = 1.0/2*(-1)/sqrt(3) + 1.0/2;
			get_bezier_4degree_seg(p_param, u, len_seg);
			u = 1.0/2/sqrt(3) + 1.0/2;
			get_bezier_4degree_seg(p_param, u, len_seg + 1);
			*p_curve_len = 0.5*(len_seg[0] + len_seg[1]);
			break;
		default:
			break;
	}


}

int line_path(const VECTOR_3D *p_start, const VECTOR_3D *p_target, VECTOR_3D *p_trajectory, double u)
{
	p_trajectory->x = p_start->x + (p_target->x - p_start->x)*u;
	p_trajectory->y = p_start->y + (p_target->y - p_start->y)*u;
	p_trajectory->z = p_start->z + (p_target->z - p_start->z)*u;

	return 0;
}

int get_line_length(const VECTOR_3D *p_start, const VECTOR_3D *p_target,double *p_length)
{
	VECTOR_3D delta;

	delta.x = p_target->x - p_start->x;
	delta.y = p_target->y - p_start->y;
	delta.z = p_target->z - p_start->z;

	*p_length = sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);

	return 0;
}

static int get_bezier_4degree_seg(const VECTOR_3D *p_param, double u, double *p_len)
{
	VECTOR_3D path_dot;

	path_dot.x = p_param[1].x + 2*p_param[2].x*u + 3*p_param[3].x*pow(u,2) + 4*p_param[4].x*pow(u,3);
	path_dot.y = p_param[1].y + 2*p_param[2].y*u + 3*p_param[3].y*pow(u,2) + 4*p_param[4].y*pow(u,3);
	path_dot.z = p_param[1].z + 2*p_param[2].z*u + 3*p_param[3].z*pow(u,2) + 4*p_param[4].z*pow(u,3);

	*p_len = sqrt(pow(path_dot.x, 2) + pow(path_dot.y, 2) + pow(path_dot.z, 2));
}


/****************************************************************************************************/
// Example B.9 in page 484
static int factorial(int n);
static double get_basis_function(unsigned char m, unsigned char j, double u);

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
 * m�� m-th degree Bernstein polynomials
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
