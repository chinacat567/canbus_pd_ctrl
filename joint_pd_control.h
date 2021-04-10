/* sumantra sharma, 25th March 2021*/
#include<sys/time.h>
#include <math.h>
#include<stdio.h>	
#include<stdlib.h>
#include "libpcanfd.h"
// #include <alchemy/task.h>
// #include <alchemy/timer.h>
// #include <alchemy/mutex.h>
#include <sys/mman.h>
#include "pcanfd.h"
#include<inttypes.h>
#include <lcm/lcm.h>
#include "spi_command_t.h" /* lcm msg type for spi command*/
#include "spi_data_t.h" /* lcm msg type for spi data*/
#include <alchemy/pipe.h>
#include <unistd.h>
#include<fcntl.h>
// #include <alchemy/queue.h>
#include "mqueue.h"
#include<signal.h>
#include<pthread.h>
#include<errno.h>
#include<sys/syscall.h>
#ifndef SYS_gettid
#error "SYS_gettid unavaiable in this system"
#endif
#define gettid() ((pid_t)syscall (SYS_gettid))


#define TASK_PERIOD_MOTOR 2000000 /* Joint pd controller task period */
#define TASK_PERIOD_IMU 10000000 /* IMU  task period */
#define DELAY 100000 /*Wait time after sending request on canbus*/
#define PI 3.142f

#define MOTOR_SPEED_MAX 100 /* max speed of gyems motor in rpm for safety, can be increased till 300 */
#define RPM_TO_RPS 0.10472 /* constant for converting rpm to radian per sec*/
#define DEG_TO_RADIAN 0.01745 /* constant for converting degrees to radians*/
#define MOTOR_Kt 3.30 /* torque const of the gyems motor, adjust this experimentally */
#define CURRENT_SCALING 62.06

/* joint max and min values, used for safety check*/
 #define P_MIN -PI /* -90 degrees*/
 #define P_MAX PI  /*+90 degrees*/
 #define V_MIN -(MOTOR_SPEED_MAX*RPM_TO_RPS)
 #define V_MAX (MOTOR_SPEED_MAX*RPM_TO_RPS)
 #define KP_MIN 0.0f
 #define KP_MAX 100.0f
 #define KD_MIN 0.0f
 #define KD_MAX 100.0f
 #define T_MIN -15.0f
 #define T_MAX 15.0f
 
/*Joint Soft Stops */  
 #define A_LIM_P 1.57f
 #define A_LIM_N -1.57f
 #define H_LIM_P PI
 #define H_LIM_N -PI
 #define K_LIM_P 30.927f
 #define K_LIM_N -30.92f
 #define KP_SOFTSTOP 15.0f
 #define KD_SOFTSTOP 0.1f 

/*maximum size of read CANBUS read buffer*/
#define MAX_READ 35
#define MAX_READ_IMU 10

    union float_bytes {
    float fval;
    uint32_t ival;
    uint8_t bvals[4];
};

struct period_info {
  struct timespec next_period;
  long period_ns;
};
/*!
 * Mini Cheetah's IMU
 */
struct imu_data {
  float accel[3];
  float gyro[3]; /*[gyro_x,gyro_y,gyro_z]*/
  float quat[4]; /* [i,j,k,real]*/
  // todo is there status for the vectornav?
};

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
  // RT_TASK write_task; 
  /* Msg queue object*/
  // RT_QUEUE msg_q_setpoints; 
  mqd_t msg_q_cmd;
  /*Msg queue object */ 
  // RT_QUEUE msg_q_status;  
  mqd_t msg_q_status;
  /* Msg queue object*/
  // RT_QUEUE msg_q_data;
  mqd_t msg_q_data;
  /*  PCAN msgs for sending requests, storing responses*/
  struct pcanfd_msg req_msgs_vel[3];
  struct pcanfd_msg req_msgs_pos[3];
  struct pcanfd_msg res_msgs[MAX_READ];
  /* for storing setpoints in PCAN driver format */
  struct setpoint_can_msgs setpoint_msgs;
  /* for storing SPI commands*/
  spi_command_t setpoints;
  spi_data_t leg_data;
  /* for storing setpoints leg-wise*/
  struct motor_torq_cmd motor_torq_setpoints;
  /* fiel descriptor for the CANPORTS*/
  int fd[4];
  /* ovrruns in the RT task*/
  unsigned long overruns;
  /* buffer for storing q data*/
  // void * data_msg_buf;

};


/*data struct for passing data from main to real-time thread for imu task*/
struct imu_rt_task_args{
  /* RT Task using the Alchemy API*/
  // RT_TASK write_task; 
  /* Msg queue object*/
  // RT_QUEUE msg_q_imu;  
  mqd_t msg_q_imu;
  /*  PCAN msgs for sending requests, storing responses*/
  struct pcanfd_msg get_quat;
  struct pcanfd_msg get_gyro;
  struct pcanfd_msg get_accel;
  struct pcanfd_msg res_msgs_imu[MAX_READ];
  struct imu_data data;
  int fd;
  /* ovrruns in the RT task*/
  // char* imu_data_msg_buf;
  unsigned long overruns;
 

};


const char* ports[4] = {"pcan4", "pcan5", "pcan2", "pcan3"};
const char* imu_port = "pcan6";
const char* devname_data = "/dev/rtp0";
const char* devname_cmd = "/dev/rtp1";
const char* devname_imu = "/dev/rtp2";
const char* devname_status = "/dev/rtp3";


/*for normalizing the zero positions*/
float offset_hip[4] = {-PI/2, PI/2, -PI/2, PI/2};
float offset_knee[4] = {PI, -PI, PI, -PI};

// float offset_hip[4] = {0,0,0,0,};
// float offset_knee[4] = {0,0,0,0};

/*starting stand position*/
// float hip_stand[4] = {-PI/4, PI/4, -PI/4, PI/4};
// float knee_stand[4] = {PI/2, -PI, PI/2, -PI/2};
/*status of higher controller*/
int status = 0;
/*mutex lock*/
// RT_MUTEX mutex;
pthread_mutex_t mutex;
pthread_mutexattr_t attr;
int isRunning = 1;

