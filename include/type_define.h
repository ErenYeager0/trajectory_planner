
#ifndef TYPE_DEFINE
#define TYPE_DEFINE

typedef void* (*lpCallback)(void*);

#define MAX_AXIS_PER_GROUP    6
#define MAX_GROUP_PER_CNANNEL 2

typedef struct aixs_info
{
	unsigned char axis_id;
	unsigned char ctrl_mod;
	unsigned char plan_mod; //T or S

	double max_vel;
	double max_acc;
	double max_jerk;
	double max_pos;
	double min_pos;

	double target_pos;
	double target_vel;
	double target_acc;

	double current_pos;
	double current_vel;
	double current_acc;
}AXIS_INFO;

typedef struct aixs_group_info
{
	AXIS_INFO *paxis_info;
	unsigned char axis_num;

	lpCallback forward_kinematics;
	lpCallback inverse_kinematics;
}AXIS_GROUP_INFO;

typedef struct channel_info
{
	AXIS_GROUP_INFO *pgroup_info;
}CHANNEL_INFO;

#endif
