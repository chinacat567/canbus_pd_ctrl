/* sumantra sharma, 25th March 2021*/
#include<sys/time.h>
#include <math.h>
#include<stdio.h>	
#include<stdlib.h>
#include "libpcanfd.h"
#include <sys/mman.h>
#include "pcanfd.h"
#include<inttypes.h>
#include "spi_command_t.h" /* lcm msg type for spi command*/
#include "spi_data_t.h" /* lcm msg type for spi data*/
#include <unistd.h>
#include<fcntl.h>
#include "mqueue.h"
#include<signal.h>
#include<pthread.h>
#include<errno.h>
#include<sys/syscall.h>
#ifndef SYS_gettid
#error "SYS_gettid unavaiable in this system"
#endif
#define gettid() ((pid_t)syscall (SYS_gettid))
#define TASK_PERIOD_IMU 10000000 /* IMU  task period */
#define MAX_READ_IMU 15

struct period_info {
  struct timespec next_period;
  long period_ns;
};


union float_bytes 
{
	float fval;
    uint32_t ival;
    uint8_t bvals[4];
};

struct imu_data {
  float accel[3];
  float gyro[3]; /*[gyro_x,gyro_y,gyro_z]*/
  float quat[4]; /* [i,j,k,real]*/
  // todo is there status for the vectornav?
};

/*data struct for passing data from main to real-time thread for imu task*/
struct imu_rt_task_args{
 
  mqd_t msg_q_imu;
  mqd_t msg_q_status;
  struct pcanfd_msg get_quat;
  struct pcanfd_msg get_gyro;
  struct pcanfd_msg get_accel;
  struct pcanfd_msg res_msgs_imu[MAX_READ_IMU];
  struct imu_data data;
  int fd;

};

const char* imu_port = "pcan6";
const char* devname_status = "/dev/rtp3";
const char* devname_imu = "/dev/rtp2";

/*status of higher controller*/
int status = 0;
pthread_mutex_t mutex;
int isRunning = 1;


