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

