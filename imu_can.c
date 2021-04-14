#include "imu_can.h"

void print_message_single(const char *prompt, const struct pcanfd_msg *msg)
{
	
	rt_printf("%s: %d 0x%08x %1d ",
			prompt,
			 msg->type,
			 msg->id,
			 msg->data_len);
	
	for (int j = 0; j < msg->data_len; j++)
	{
		rt_printf("%02x ", msg->data[j]);
	}

	rt_printf("\n");
	
}


float bytes_to_float(const uint8_t *byte_array)
{
    char float_byte_array[4];
    float retval;

    float_byte_array[0] = byte_array[0];
    float_byte_array[1] = byte_array[1];
    float_byte_array[2] = byte_array[2];
    float_byte_array[3] = byte_array[3];

    memcpy(&retval, float_byte_array, 4);

    return retval;
}

uint16_t bytes_to_uint16(const uint8_t *byte_array)
{
	 
	char uint_byte_array[2];
	uint16_t retval;
	uint_byte_array[0] = byte_array[0];
	uint_byte_array[1] = byte_array[1];
	memcpy(&retval, uint_byte_array, 2);
	return retval;


}


static void inc_period(struct period_info *pinfo)	
{
  pinfo->next_period.tv_nsec += pinfo->period_ns;

  while (pinfo->next_period.tv_nsec >= 1000000000) {
    /* timespec nsec overflow */
    pinfo->next_period.tv_sec++;
    pinfo->next_period.tv_nsec -= 1000000000;
  }
}

static void wait_rest_of_period(struct period_info *pinfo)
{
  inc_period(pinfo);
  int err;
  /* for simplicity, ignoring possibilities of signal wakes */
  err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
  if(err)
  {
  	rt_printf("clock_nanosleep failed\n");
  }
}


static void periodic_task_init(struct period_info *pinfo, long int setperiod)
{
  /* for simplicity, default is a 100ms period */
	printf("period %ld set\n", setperiod);
  pinfo->period_ns = setperiod; 
  clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{

	float qFloat = fixedPointValue;
	qFloat *= pow(2.0, qPoint * -1);
	return (qFloat);
}

void print_imu_data(struct imu_data* msg)
{
	rt_printf("\n");
	rt_printf("quat_I = %f, quat_J = %f, quat_K = %f, a = %f\n" , msg->quat[0], msg->quat[1], \
	msg->quat[2], msg->quat[3]);
	rt_printf("gyro_x = %f, gyro_y = %f, gyro_z = %f\n", msg->gyro[0], msg->gyro[1], \
	msg->gyro[2]);
	rt_printf("accel_x = %f, accel_y = %f, accel_z = %f\n", msg->accel[0], msg->accel[1], \
	msg->accel[2]);
		
	
}

void* write_task_func_imu(void * arg)
{
	int err, id= 0, flag = 0;
	struct imu_rt_task_args* args;
	args = (struct imu_rt_task_args*)arg;
	int isRunning_local=1;
	uint prio = 1;

	struct period_info pinfo;
	periodic_task_init(&pinfo,TASK_PERIOD_IMU);
	
	while(isRunning_local)
	{
		
		/* read status from q*/
		err = mq_receive(args->msg_q_status, (char *)(&status), sizeof(int), &prio);
		if(err < 0)
		{
			if(errno == EAGAIN)
			{
				#ifdef DEBUG
				rt_printf("[IMU-RT-TASK] :No msg available in status msg q, skipping\n");
				#endif 	
			}
			else if(errno == EBADF)
			{
				rt_printf("[IMU-RT-TASK] :Invalid fd for status msg q\n");
				return NULL;
			}
			else if(errno == EMSGSIZE)
			{
				rt_printf("[IMU-RT-TASK] :Invalid msg size for status msg q\n");
				return NULL;
			}
		}
		else if (err == sizeof(int))
		{
			#ifdef DEBUG
			rt_printf("[IMU-RT-TASK] : Read status msg from higher controller\n");	
			#endif	
		}
		else
		{
			rt_printf("[IMU-RT-TASK]: DANGER : Read incomplete status from higher controller %d\n");
			return NULL;
		}


		/*send reqs for imu data*/
		/*139 - Quaternions, 142 - Accel, 145 - Gyro*/
		err = pcanfd_send_msg(args->fd, &args->get_quat);
		if (err)
		{
			if(err == -EWOULDBLOCK)
			{
				rt_printf("[IMU-RT-TASK] :DANGER :Failed to write msg to CANBUS because queue is full, increase loop time\n");

			}
			else
			{
				return NULL;	
			}
						
		}

		err = pcanfd_send_msg(args->fd, &args->get_gyro);
		if (err)
		{
			if(err == -EWOULDBLOCK)
			{
				rt_printf("[IMU-RT-TASK] :DANGER :Failed to write msg to CANBUS because queue is full, increase loop time\n");

			}
			else
			{
				return NULL;	
			}
						
		}
		err = pcanfd_send_msg(args->fd, &args->get_accel);
		if (err)
		{
			if(err == -EWOULDBLOCK)
			{
				rt_printf("[IMU-RT-TASK] :DANGER :Failed to write msg to CANBUS because queue is full, increase loop time\n");

			}
			else
			{
				return NULL;	
			}
						
		}
	
		/*read responses*/
		int msg_count = 0;
		while(1)
		{
			err = pcanfd_recv_msg(args->fd, &args->res_msgs_imu[msg_count]);
			if (err)
			{
				/* if msg queue is empty, break from the loop, else return*/
				if(err == -EWOULDBLOCK)
				{
					break;
				}
				else
				{
					rt_printf("[IMU-RT-TASK] :DANGER : Recv msg failed because of error no. %d\n", err);	
					return NULL;
				}
					
			}
			else
			{
			 	#ifdef DEBUG
				print_message_single(imu_port, &args->res_msgs_imu[msg_count]);	
				print_imu_data(&args->data);
				#endif
				msg_count++;
			} 
			
		}
		/* parse the data*/
		for (int i = 0; i < msg_count; ++i)
		{
			id = args->res_msgs_imu[i].id;
			switch(id)
			{
				case 140:
					args->data.quat[0] = bytes_to_float(&args->res_msgs_imu[i].data[0]);
					args->data.quat[1] = bytes_to_float(&args->res_msgs_imu[i].data[4]);
				case 141:
					args->data.quat[2] = bytes_to_float(&args->res_msgs_imu[i].data[0]);
					args->data.quat[3] = bytes_to_float(&args->res_msgs_imu[i].data[4]);
				case 146:
					args->data.gyro[0] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[0]), 9);
					args->data.gyro[1] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[2]), 9);
					args->data.gyro[2] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[4]), 9);
				case 143:
					args->data.accel[0] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[0]), 8);
					args->data.accel[1] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[2]), 8);
					args->data.accel[1] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[4]), 8);
			}


		}
		
		/* acquire mutex*/
		err = pthread_mutex_trylock(&mutex);
		if(err)
		{
			if(errno != EBUSY)
			{
				rt_printf("[IMU-RT-TASK] : Invalid context, failed to lock mutex\n");
				return NULL;
			}	
		}
		else
		{
			
			isRunning_local = isRunning;
			
		}
		
		/*release mutex*/
		err = pthread_mutex_unlock(&mutex);
		if(err)
		{
			if(errno == EPERM)
			{
				rt_printf("[IMU-RT-TASK] : Invalid context, failed to unlock mutex\n");
				return NULL;
			}
			else if (errno == EINVAL)
			{
				rt_printf("[IMU-RT-TASK] : Invalid mutex, failed to unlock\n");
				return NULL;
			}
			else
			{
				rt_printf("[IMU-RT-TASK] : Failed to release mutex\n");
				return NULL;
			}

		}
			
		if (status)
		{
		 	err = mq_send(args->msg_q_imu, (char *) &args->data, sizeof(struct imu_data), 1);
		  	if(err < 0)
			{
				if(errno == EAGAIN)
				{
					#ifdef DEBUG
					rt_printf("[IMU-RT-TASK] : Imu data q full, skipping\n");
					#endif 	
				}
				else if(errno == EBADF)
				{
					rt_printf("[IMU-RT-TASK] : Invalid fd for imu msg q\n");
					return NULL;
				}
				else if(errno == EMSGSIZE)
				{
					rt_printf("[IMU-RT-TASK] : Invalid msg size for imu msg q\n");
					return NULL;
				}
			}
		  
		}

		else
		{
			#ifdef DEBUG
			rt_printf("[IMU-RT-TASK] : IMU TASK : Waiting for higher controller\n");
			#endif
		}

		wait_rest_of_period(&pinfo);	
		
	}
	rt_printf("[IMU-RT-TASK] : Exiting pthread\n");
}


int main()
{
	int err;
	cpu_set_t cpu_imu;
	CPU_ZERO(&cpu_imu);
	CPU_SET(2,&cpu_imu);
	pthread_t imu_thread;

	struct imu_rt_task_args args_imu;

	memset(&args_imu,0,sizeof(struct imu_rt_task_args));

	struct pcanfd_msg_filter msg_filter;

	/*open imu can port*/
	args_imu.fd = pcanfd_open(imu_port, OFD_BITRATE  | PCANFD_INIT_STD_MSG_ONLY | OFD_NONBLOCKING, 1000000);
	if (args_imu.fd < 0)
	{
		rt_printf("[IMU-RT-TASK]: Main:Pcanfd open failed with err %d on port no.\n",args_imu.fd);
		return 0;
	}

	/* set options to allow only STD-CAN msgs */
	uint32_t allowed_msg = PCANFD_ALLOWED_MSG_CAN;
	err = pcanfd_set_option(args_imu.fd, PCANFD_OPT_ALLOWED_MSGS, &allowed_msg, sizeof(allowed_msg));
	if (err) 
	{
		rt_printf("[IMU-RT-TASK]: Main: Failed to set_option on fd %d with errno %d\n", args_imu.fd, err);
		return 0;

	}

	/* add filter for imu canbus*/
	msg_filter.id_from = 139;
	msg_filter.id_to = 151;
	err = pcanfd_add_filter(args_imu.fd, &msg_filter);
	if (err) 
	{
		rt_printf("[IMU-RT-TASK]: Main: Failed to add_filter on fd %d with errno %d\n", args_imu.fd, err);
		return 0;
	}
	/* imu quat req msg*/
		
	args_imu.get_quat.type = PCANFD_TYPE_CAN20_MSG;
	args_imu.get_quat.data_len = 8;
	args_imu.get_quat.id = 139;
	for (int j = 0; j < 8; ++j)
	{
		args_imu.get_quat.data[j] = 0;	
	}
	/*imu gyro req msg*/
	args_imu.get_gyro.type = PCANFD_TYPE_CAN20_MSG;
	args_imu.get_gyro.data_len = 8;
	args_imu.get_gyro.id = 145;
	for (int j = 0; j < 8; ++j)
	{
		args_imu.get_gyro.data[j] = 0;	
	}
	/*imu gyro req msg*/
	args_imu.get_accel.type = PCANFD_TYPE_CAN20_MSG;
	args_imu.get_accel.data_len = 8;
	args_imu.get_accel.id = 142;
	for (int j = 0; j < 8; ++j)
	{
		args_imu.get_accel.data[j] = 0;	
	}

	struct mq_attr imu_q_attr;
	imu_q_attr.mq_flags = 0;
	imu_q_attr.mq_maxmsg = 100;
	imu_q_attr.mq_msgsize = sizeof(struct imu_data);
	args_imu.msg_q_imu = mq_open(devname_imu, O_WRONLY | O_NONBLOCK | O_CREAT, 0666, &imu_q_attr);
	if(args_imu.msg_q_imu < 0)
	{
		rt_printf("Failed to open data msg q\n");
		return 0;
	}
	 
	struct mq_attr status_q_attr;
	status_q_attr.mq_flags = 0;
	status_q_attr.mq_maxmsg = 100;
	status_q_attr.mq_msgsize = sizeof(int);
	mqd_t msg_q_status;
	args_imu.msg_q_status = mq_open(devname_status, O_RDONLY | O_NONBLOCK | O_CREAT, 0666, &status_q_attr);
	 if(args_imu.msg_q_status < 0)
	 {
	 	rt_printf("Failed to open data msg q\n");
	 	return 0;
	 }

	/* flush the open qs*/
	uint prio;
	spi_command_t data_temp;
	int status_temp;


	while(1)
	{
		err = mq_receive(args_imu.msg_q_status, (char *)(&status_temp), sizeof(int), &prio);
		if (err < 0)
		{
			if(errno == EAGAIN)
			{	
				rt_printf("[IMU-TASK-Main] :No msg available in status msg q, flushed successfully\n");
				break;
			}
			else
			{
				rt_printf("[IMU-TASK-Main] :Failed to flush status msg q, exiting\n");
				return 0;
			}
		}
		else
		{
			rt_printf("[IMU-TASK-Main]: Msg flushed from status msg q %d\n");
		}
	}
		
	rt_printf("Cmd msg q flush complete\n");

	/* init mutex lock*/
	err = pthread_mutex_init(&mutex, NULL);
	if (err) 
	{
		rt_printf("[IMU-TASK-Main] : Failed to init mutex lock, exiting\n");;
		return 0;
	}

	/* set sched policy*/
	pid_t id = gettid();
	struct sched_param params;
	params.sched_priority = 99;
	/* set cpu*/
	err =  sched_setaffinity(id, sizeof(cpu_set_t), &cpu_imu);
	if (err)
	{
    	rt_printf("[Init] sched_setaffinity failed.\n");
    	return 0;
	}

	err = sched_setscheduler(id, SCHED_FIFO, &params);
	if (err)
	{
	   rt_printf("[Init] sched_setscheduler failed.\n");
	   return 0;
	}

	/*for avoiding page faults*/	
	mlockall(MCL_CURRENT | MCL_FUTURE);

	err = pthread_create(&imu_thread, NULL, write_task_func_imu, &args_imu);
	if (err)
	{
		rt_printf("[IMU-RT-TASK]: Main: Failed to start imu pthread, code %d\n", err);
		return 0;
	}

	rt_printf("Press any key + ENTER to exit\n");
	getchar();

	err = pthread_mutex_lock(&mutex);
	if(err)
	{
		if(errno == EPERM)
		{
			rt_printf("[IMU-RT-TASK]: Main : Invalid context, failed to lock mutex\n");
	
		}
		else if (errno == EINVAL)
		{
			rt_printf("[IMU-RT-TASK]: Main : Invalid mutex, failed to lock\n");
	
		}
		else
		{
			rt_printf("[IMU-RT-TASK]: Main :  Failed to acquire mutex\n");
	
		}

	}
		
	rt_printf("[IMU-RT-TASK] : Main: Acquired mutex\n");
	isRunning = 0;
	err = pthread_mutex_unlock(&mutex);
	if(err)
	{
		if(errno == EPERM)
		{
			rt_printf("[IMU-RT-TASK] : Main :  Invalid context, failed to unlock mutex\n");
		}
		else if (errno == EINVAL)
		{
			rt_printf("[IMU-RT-TASK] : Main :  Invalid mutex, failed to unlock\n");
		}
		else
		{
			rt_printf("[IMU-RT-TASK] : Main :  Failed to release mutex\n");
		}

	}
	/* wait for thread to exit*/
	pthread_join(imu_thread, NULL);
	/*close msg qs*/
	mq_close(args_imu.msg_q_status);
	mq_close(args_imu.msg_q_imu);
	/* close can port*/
	err = pcanfd_close(args_imu.fd);
	if (err != -1)
	{
		rt_printf("[IMU-RT-TASK] : Main ::Pcanfd close failed with err %d\n",err);
	}

	return 0;

}

	
