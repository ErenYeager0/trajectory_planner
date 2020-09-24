
#ifndef TYPE_DEFINE
#define TYPE_DEFINE

typedef void* (*lpCallback)(void*,...);
#ifdef PRINT_DEBUG

#else

#endif

#define MAX_AXIS_PER_GROUP    6
#define MAX_GROUP_PER_CNANNEL 2
#define JT_EPS 0.0005
#define TARGET_POS_MAX        10

typedef enum rotation_type
{
	NO_ROT,
	RPY,
	QUATER,
}ROTATION_TYPE;

typedef struct vector_2D
{
	double x;
	double y;
}VECTOR_2D;

typedef struct vector_3D
{
	double x;
	double y;
	double z;
}VECTOR_3D;

typedef struct matrix_rotation
{
	VECTOR_3D n;
	VECTOR_3D o;
	VECTOR_3D a;
}MATRIX_ROTATION;

typedef struct matrix_pose
{
	VECTOR_3D trans;

	//type RPY
	double roll,pitch,yaw;

	//type quaternion


}MATRIX_POSE;

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
	unsigned char move_flag;//0--stop,1--positive, -1--negative
	ROTATION_TYPE rotation_type;
	unsigned char current_target;

	lpCallback forward_kinematics;
	lpCallback inverse_kinematics;

	double max_dis_vel;
	double max_dis_acc;
	double max_dis_jerk;

	double max_rot_vel;
	double max_rot_acc;
	double max_rot_jerk;

	MATRIX_POSE target_pos[TARGET_POS_MAX]; //mm & du
	double target_vel; //mm/s && du/s
	double target_acc; //mm/s^2 du/s^2

	MATRIX_POSE start_pos;
	double start_vel;
	double start_acc;

	MATRIX_POSE traj_pos; //mm & du
	double traj_vel; //mm/s && du/s
	double traj_acc; //mm/s^2 du/s^2

	MATRIX_POSE delta_pos;

	long plan_timer[7];
}AXIS_GROUP_INFO;

typedef struct channel_info
{
	AXIS_GROUP_INFO *pgroup_info;
}CHANNEL_INFO;

#endif
