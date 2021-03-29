/* sumantra sharma, 25th March 2021*/
#include<sys/time.h>
#include <math.h>
#include<stdio.h>	
#include<stdlib.h>
#include "libpcanfd.h"
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <sys/mman.h>
#include "pcanfd.h"
#include<inttypes.h>
#include <lcm/lcm.h>
#include "spi_command_t.h" /* lcm msg type for spi command*/
#include "spi_data_t.h" /* lcm msg type for spi data*/

#define TASK_PERIOD 2000000 /* Joint pd controller task period */
#define DELAY 100000 /*Wait time after sending request on canbus*/

#define PI 3.142

#define MOTOR_SPEED_MAX 250 /* max speed of gyems motor in rpm for safety, can be increased till 300 */
#define RPM_TO_RPS 0.10472 /* constant for converting rpm to radian per sec*/
#define DEG_TO_RADIAN 0.01745 /* constant for converting degrees to radians*/
#define MOTOR_Kt 3.30 /* torque const of the gyems motor, adjust this experimentally */

/* joint max and min values, used for safety check*/
 #define P_MIN -1.57f /* -90 degrees*/
 #define P_MAX 1.57f  /*+90 degrees*/
 #define V_MIN -(MOTOR_SPEED_MAX*RPM_TO_RPS)
 #define V_MAX (MOTOR_SPEED_MAX*RPM_TO_RPS)
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -15.0f
 #define T_MAX 15.0f
 
/*Joint Soft Stops */  
 #define A_LIM_P 1.5f
 #define A_LIM_N -1.5f
 #define H_LIM_P 5.0f
 #define H_LIM_N -5.0f
 #define K_LIM_P 0.2f
 #define K_LIM_N 7.7f
 #define KP_SOFTSTOP 100.0f
 #define KD_SOFTSTOP 0.4f

/*maximum size of read CANBUS read buffer*/
#define MAX_READ 35

struct motor_torq_cmd{
	float torq_setpoint_knee[4];
	float torq_setpoint_hip[4];
	float torq_setpoint_abad[4];
};

/*data struct for storing all the joint setpoints in the PCAN driver format */
struct setpoint_can_msgs{
	struct pcanfd_msg knee_setpoints[4];
	struct pcanfd_msg hip_setpoints[4];
	struct pcanfd_msg abad_setpoints[4];
};

/*data struct for passing data from main to real-time thread*/
struct rt_task_args{
  /* RT Task using the Alchemy API*/
  RT_TASK write_task;   
  /*mutex for accesing shared memory*/
  RT_MUTEX mutex;
  /*  PCAN msgs for sending requests, storing responses*/
  struct pcanfd_msg req_msgs_vel[3];
  struct pcanfd_msg req_msgs_pos[3];
  struct pcanfd_msg res_msgs[MAX_READ];
  /* for storing setpoints in PCAN driver format */
  struct setpoint_can_msgs setpoint_msgs;
  /* for storing SPI commands*/
  spi_command_t *setpoints;
  spi_data_t leg_data;
  /* for storing setpoints leg-wise*/
  struct motor_torq_cmd motor_torq_setpoints;
  /* fiel descriptor for the CANPORTS*/
  int fd[4];
  /* ovrruns in the RT task*/
  unsigned long overruns;
  /* flag for  while loop*/
  int isRunning_rt;

};



const char* ports[4] = {"pcan0", "pcan1", "pcan6", "pcan7"};
static volatile sig_atomic_t isRunning = 1;
struct timespec timeout;
