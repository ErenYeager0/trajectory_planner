
#ifndef TYPE_DEFINE
#define TYPE_DEFINE

typedef void* (*lpCallback)(void*,...);
#ifdef PRINT_DEBUG

#else

#endif

#define MAX_AXIS_PER_GROUP    6
#define MAX_GROUP_PER_CNANNEL 2
#define JT_EPS 0.0005

typedef struct aixs_info
{
	unsigned char axis_id;
	unsigned char ctrl_mod;
	unsigned char plan_mod; //T or S
	unsigned char move_flag;//0--stop,1--positive, -1--negative

	double max_vel;
	double max_acc;
	double max_jerk;
	double max_pos;
	double min_pos;

	double target_pos; //du
	double target_vel; //du/s
	double target_acc; //du/s^2

	double start_pos;
	double start_vel;
	double start_acc;

	double traj_pos; //du
	double traj_vel; //du/s
	double traj_acc; //du/s^2

	double delta_pos;

	long plan_timer[7];
	//long counter;
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
