#include <stdio.h>
#include "type_define.h"

extern void axis_planner_test();
extern void cartesian_line_planner_test();
int main()
{
	printf("hello planner\n\n");

	//axis_planner_test();
	cartesian_line_planner_test();


	printf("goodbye planner\n");
	return 0;
}
