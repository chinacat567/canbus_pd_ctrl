#include "mqueue.h"
#include "stdio.h"
#include<errno.h>
#include<spi_data_t.h>
#include<spi_command_t.h>
#include<joint_pd_control.h>
#include<signal.h>
int isrunning = 1;
void *thread_func(void* argp)
{
	int err;
	int status = 1;
	mqd_t * msg_q_status;
	msg_q_status = (mqd_t *)argp;

	while(isrunning)
	{
		
		err = mq_send(*msg_q_status, (char*) (&status), sizeof(int), 1);
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
	}
}
void handle_sigint(int sig)
{
	isrunning = 0;
}

int main()
{
	/* open the msg queue objects*/
	signal(SIGINT, handle_sigint);
	pthread_t thread;
	struct mq_attr data_q_attr;
	data_q_attr.mq_flags = 0;
	data_q_attr.mq_maxmsg = 100;
	data_q_attr.mq_msgsize = sizeof(spi_data_t);
	mqd_t msg_q_data;
	msg_q_data = mq_open("/dev/rtp0", O_RDONLY | O_NONBLOCK | O_CREAT, 0666, &data_q_attr);
  	if(msg_q_data < 0)
   	printf("Failed to open data msg q\n");

    struct mq_attr msg_q_attr;
	msg_q_attr.mq_flags = 0;
	msg_q_attr.mq_maxmsg = 100;
	msg_q_attr.mq_msgsize = sizeof(spi_command_t);
	mqd_t msg_q_cmd;
	msg_q_cmd = mq_open("/dev/rtp1", O_WRONLY | O_NONBLOCK | O_CREAT, 0666, &msg_q_attr);
  	if(msg_q_cmd < 0)
   	printf("Failed to open data msg q\n");

   	struct mq_attr imu_q_attr;
	imu_q_attr.mq_flags = 0;
	imu_q_attr.mq_maxmsg = 100;
	imu_q_attr.mq_msgsize = sizeof(struct imu_data);
	mqd_t msg_q_imu;
	msg_q_imu = mq_open("/dev/rtp2", O_WRONLY | O_NONBLOCK | O_CREAT, 0666, &imu_q_attr);
  	if(msg_q_imu < 0)
   	printf("Failed to open data msg q\n");

   	struct mq_attr status_q_attr;
	status_q_attr.mq_flags = 0;
	status_q_attr.mq_maxmsg = 1000;
	status_q_attr.mq_msgsize = sizeof(int);
	mqd_t msg_q_status;
	msg_q_status = mq_open("/dev/rtp3", O_WRONLY | O_NONBLOCK | O_CREAT, 0666, &status_q_attr);
  	if(msg_q_status < 0)
   	printf("Failed to open data msg q\n");
	int err;
	spi_command_t cmd;
	memset(&cmd, 0, sizeof(spi_command_t));
	pthread_create(&thread, NULL, thread_func, &msg_q_status);
	pthread_join(thread, NULL);
	/*cleanup*/
	mq_close(msg_q_data);
	mq_close(msg_q_status);
	mq_close(msg_q_cmd);
	mq_close(msg_q_imu);
	return 0;

}