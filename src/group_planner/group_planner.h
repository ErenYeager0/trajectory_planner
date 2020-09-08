

typedef enum group_plan_stage
{
	GROUP_INIT_STAGE,
	GROUP_ACC_STAGE,
	GROUP_SLIP_STAGE,
	GROUP_DEC_STAGE,
	GROUP_PAUSE_STAGE
}GROUP_PLAN_STAGE;

typedef struct
{
	//AXIS_PLAN_STAGE p_stage;
	lpCallback p_handle;
}GROUP_PLANNER_UNIT;

