// Author @HugoPernet


/// Motor values limits ///


#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -20.0f
#define V_MAX 20.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -8.0f
#define T_MAX 8.0f

//displacement step
#define Step 0.01

/// CAN parameters for motors ///

//CAN Parameters
#define MY_PACKET_ID 0x01
int dlc = -1;