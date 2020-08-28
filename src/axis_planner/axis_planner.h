

typedef enum axis_plan_stage
{
	INIT_STAGE,
	ACC_STAGE,
	SLIP_STAGE,
	DEC_STAGE,
	PAUSE_STAGE
}AXIS_PLAN_STAGE;

typedef struct
{
	//AXIS_PLAN_STAGE p_stage;
	lpCallback p_handle;
}AXIS_PLANNER_UNIT;


void axis_planner_test();
