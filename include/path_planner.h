
#ifndef MATH_FUNCTION_DEFINE
#define MATH_FUNCTION_DEFINE

#include "type_define.h"

int get_track_join(const VECTOR_3D *p_start, const VECTOR_3D *p_target, double delta, VECTOR_3D *p_join);
int beizier_path_4degree(const VECTOR_3D *p_start, const VECTOR_3D *p_middle, const VECTOR_3D *p_target, double delta, VECTOR_3D *p_trajectory, double u);
int beizier_path_tangent(const VECTOR_3D *p_start, const VECTOR_3D *p_target, double time_delta, VECTOR_3D *p_tangent);
int line_path(const VECTOR_3D *p_start, const VECTOR_3D *p_target, VECTOR_3D *p_trajectory, double u);

#endif
