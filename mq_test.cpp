#include "mqueue.h"
#include<thread>
#include<errno.h>
#include<stdio.h>
using namespace std;
#include<unistd.h>
#include<sys/syscall.h>
#ifndef SYS_gettid
#error "SYS_gettid unavaiable in this system"
#endif
#define gettid() ((pid_t)syscall (SYS_gettid))
#include"helper.hpp"
#include<thread>
mqd_t msg_q_status;

void* task(void* msg_q_arg)
{
	mqd_t* msg_q;
	msg_q = (mqd_t *) msg_q_arg;
	int err,status=0;
	int count = 0;
	while(count < 100)
	{
		err = mq_send(msg_q_status, (char*) (&status), sizeof(int), 1);
		if(err < 0)
		{   
		  if (errno == EAGAIN)
		     printf("Cmd Message queue full, skiping send\n");
		  else if (errno == EBADF)
		    printf("Failed to write setopints to msg q becauuse of bad fd\n");
		  else 
		    printf("Failed to write setopints to msg q \n");

		}
		if(err == 0)
			printf("Send successful\n");
		count++;
	}
}


int main()
{

	pthread_t thread;
	struct mq_attr status_q_attr;
		status_q_attr.mq_flags = 0;
		status_q_attr.mq_maxmsg = 100;
		status_q_attr.mq_msgsize = sizeof(int);
		msg_q_status = mq_open("/dev/rtp1", O_WRONLY | O_NONBLOCK | O_CREAT, 0666, &status_q_attr);
	  	if(msg_q_status < 0)
	   	printf("Failed to open data msg q\n");

printf("[Init] Setup RT Scheduler...\n");
  struct sched_param params;
  params.sched_priority = 99;
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    printf("sched_setscheduler failed.\n");
  }
  cpu_set_t cpu_motors;;
  CPU_ZERO(&cpu_motors);
  CPU_SET(3,&cpu_motors);
  if (sched_setaffinity(0, sizeof(cpu_set_t), &cpu_motors) == -1) {
    printf("sched_setaffinity failed.\n");
  }

  pthread_create(&thread, NULL, &task, NULL);
  pthread_join(thread, NULL);
  // helper(msg_q_status);

  // std::thread msg_q_thread(task, &msg_q_status);
  // msg_q_thread.join();

	return 0;
}