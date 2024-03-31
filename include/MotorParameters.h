// Author @HugoPernet


/// Motor values limits ///

//position
#define P_MIN -12.5f
#define P_MAX 12.5f
//velocity
#define V_MIN -5.0f
#define V_MAX 5.0f
//proportional factor
#define KP_MIN 0.0f
#define KP_MAX 500.0f
//derivative factor
#define KD_MIN 0.0f
#define KD_MAX 5.0f
//torque
#define T_MIN 0.0f
#define T_MAX 5.0f

/// CAN parameters for motors ///

//Left Motor
#define CAN_ID_Motor_L 0x01
#define dlc -1