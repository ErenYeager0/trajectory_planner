#include <stdio.h>
#include "type_define.h"

extern void axis_planner_test();
extern void cartesian_line_planner_test_one_seg();
extern void bezier_curves_test();
extern void bezier_contour_test_path();
extern void bezier_contour_test_T_traj();
extern void bezier_contour_test_820();

const lpCallback planner_tester[] = {
/* HANDLE*/
		axis_planner_test,                    //demo0:�����ٶ�T�͹滮
		cartesian_line_planner_test_one_seg,  //demo1:�ѿ����ռ�ֱ�߹滮����̬ΪRPY
		bezier_contour_test_path,             //demo2:����������
		bezier_contour_test_T_traj            //demo3:����������T���ٶȹ滮
};

int main(void *argc)
{
	int test_demo_num = 0;
	printf("************************************************\n");
	printf("*                                              *\n");
	printf("*              trajectory planner              *\n");
	printf("*                                              *\n");
	printf("************************************************\n\n");

	printf("please input test demo number(0~2):\n");
	printf("		0:axis_planner_test\n");
	printf("		1:cartesian_line_planner_test_one_seg\n");
	printf("		2:bezier_contour_test_path\n");
	printf("		3:bezier_contour_test_T_traj\n");
	scanf("%d", &test_demo_num);
	planner_tester[test_demo_num](argc);

	printf("goodbye planner\n");
	return 0;
}
