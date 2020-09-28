
#ifndef MATH_FUNCTION_DEFINE
#define MATH_FUNCTION_DEFINE

#include "type_define.h"

int get_track_join(const VECTOR_3D *p_start, const VECTOR_3D *p_target, double delta, VECTOR_3D *p_join);
int get_beizier_4degree_param(const VECTOR_3D *p_start, const VECTOR_3D *p_middle, const VECTOR_3D *p_target, double delta, VECTOR_3D *p_param);
int get_bezier_path_4degree_length(const VECTOR_3D *p_param, int n, double *p_curve_len);
int beizier_4degree_path( const VECTOR_3D *p_param, double u, VECTOR_3D *p_trajectory);

int line_path(const VECTOR_3D *p_start, const VECTOR_3D *p_target, VECTOR_3D *p_trajectory, double u);
int get_line_length(const VECTOR_3D *p_start, const VECTOR_3D *p_target,double *p_length);
#endif
