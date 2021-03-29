/*sumantra sharma 25th March 2021*/
/* 28th March - added mutex locks between LCM and RT thread*/
/* 29th March - mutex sharing between RT and non RT not possible. Use msg pipes instead*/
#include "joint_pd_control.h"

void control_comp(struct rt_task_args *args)
{
	int err;
	/* lock mutex*/
	err = rt_mutex_acquire_timed(&args->mutex, NULL);
	if (err)
	{
		printf("Joint PD Controller: Failed to acquire mutex lock, code %d\n", err);	
		args->isRunning_rt = 0;
		return;
	}
		/* tau = tau_ff + kp * (theta-theta_r) + kd * (v-v_r)*/
		for (int i = 0; i < 4; ++i)
		{
			if(args->leg_data.q_knee[i] > K_LIM_P)
			{
				args->setpoints->q_des_knee[i] = K_LIM_P;
				args->setpoints->qd_des_knee[i] = 0.0f;
				args->setpoints->kp_knee[i] = 0;
				args->setpoints->kd_knee[i] = KD_SOFTSTOP;
				args->setpoints->tau_knee_ff[i] += KP_SOFTSTOP*(K_LIM_P - args->leg_data.q_knee[i]);
				args->leg_data.flags[i] |= 1;

			}
			else if (args->leg_data.q_knee[i] < K_LIM_N)
			{
				args->setpoints->q_des_knee[i] = K_LIM_N;
				args->setpoints->qd_des_knee[i] = 0.0f;
				args->setpoints->kp_knee[i] = 0;
				args->setpoints->kd_knee[i] = KD_SOFTSTOP;
				args->setpoints->tau_knee_ff[i] += KP_SOFTSTOP*(K_LIM_N - args->leg_data.q_knee[i]);
				args->leg_data.flags[i] = 1;

			}

			args->motor_torq_setpoints.torq_setpoint_knee[i] = \
			args->setpoints->tau_knee_ff[i]	+ (args->setpoints->kp_knee[i]) * (args->setpoints->q_des_knee[i] - args->leg_data.q_knee[i]) + \
			(args->setpoints->kd_knee[i]) * (args->setpoints->qd_des_knee[i] - args->leg_data.qd_knee[i]);

			if(args->leg_data.q_hip[i] > K_LIM_P)
			{
				args->setpoints->q_des_hip[i] = K_LIM_P;
				args->setpoints->qd_des_hip[i] = 0.0f;
				args->setpoints->kp_hip[i] = 0;
				args->setpoints->kd_hip[i] = KD_SOFTSTOP;
				args->setpoints->tau_hip_ff[i] += KP_SOFTSTOP*(K_LIM_P - args->leg_data.q_hip[i]);
				args->leg_data.flags[i] = 2;

			}
			else if (args->leg_data.q_hip[i] < K_LIM_N)
			{
				args->setpoints->q_des_hip[i] = K_LIM_N;
				args->setpoints->qd_des_hip[i] = 0.0f;
				args->setpoints->kp_hip[i] = 0;
				args->setpoints->kd_hip[i] = KD_SOFTSTOP;
				args->setpoints->tau_hip_ff[i] += KP_SOFTSTOP*(K_LIM_N - args->leg_data.q_hip[i]);
				args->leg_data.flags[i] = 2;

			}

			args->motor_torq_setpoints.torq_setpoint_hip[i] = \
			args->setpoints->tau_hip_ff[i]	+ (args->setpoints->kp_hip[i]) * (args->setpoints->q_des_hip[i] - args->leg_data.q_hip[i]) + \
			(args->setpoints->kd_hip[i]) * (args->setpoints->qd_des_hip[i] - args->leg_data.qd_hip[i]);

			if(args->leg_data.q_abad[i] > K_LIM_P)
			{
				args->setpoints->q_des_abad[i] = K_LIM_P;
				args->setpoints->qd_des_abad[i] = 0.0f;
				args->setpoints->kp_abad[i] = 0;
				args->setpoints->kd_abad[i] = KD_SOFTSTOP;
				args->setpoints->tau_abad_ff[i] += KP_SOFTSTOP*(K_LIM_P - args->leg_data.q_abad[i]);
				args->leg_data.flags[i] = 3;

			}
			else if (args->leg_data.q_abad[i] < K_LIM_N)
			{
				args->setpoints->q_des_abad[i] = K_LIM_N;
				args->setpoints->qd_des_abad[i] = 0.0f;
				args->setpoints->kp_abad[i] = 0;
				args->setpoints->kd_abad[i] = KD_SOFTSTOP;
				args->setpoints->tau_abad_ff[i] += KP_SOFTSTOP*(K_LIM_N - args->leg_data.q_abad[i]);
				args->leg_data.flags[i] = 3;

			}


			args->motor_torq_setpoints.torq_setpoint_abad[i] = \
			args->setpoints->tau_abad_ff[i]	+ (args->setpoints->kp_abad[i]) * (args->setpoints->q_des_abad[i] - args->leg_data.q_abad[i]) + \
			(args->setpoints->kd_abad[i]) * (args->setpoints->qd_des_abad[i] - args->leg_data.qd_abad[i]);
	
		}
		/* release mutex*/
		err = rt_mutex_release(&args->mutex);
		if (err)
		{
			printf("Joint PD Controller: Failed to release mutex lock, code %d\n", err);
			args->isRunning_rt = 0;
			return;
		} 
			
}

static void lcm_recv_handler(const lcm_recv_buf_t *rbuf, const char* channel, const spi_command_t *msg, void* args)
{
	/*copy msg to args->setpoints after applying safety limits*/
	struct rt_task_args* args_t;
	args_t =  (struct rt_task_args*)args;
	int err;
	/*lock mutex*/
	err = rt_mutex_acquire_timed(&args_t->mutex, NULL);
	if (err)
	{	
		printf("Joint PD Controller: Failed to acquire mutex lock in lcm callback, code %d\n", err);
		isRunning = 0;
		return;
	}
	/* apply safety limits*/
	for (int i = 0; i < 4; ++i)
	{	
		args_t->setpoints->q_des_knee[i] = fminf(fmaxf(P_MIN, msg->q_des_knee[i]), P_MAX);
		args_t->setpoints->q_des_abad[i] = fminf(fmaxf(P_MIN, msg->q_des_abad[i]), P_MAX);
		args_t->setpoints->q_des_hip[i] = fminf(fmaxf(P_MIN, msg->q_des_hip[i]), P_MAX);

		args_t->setpoints->qd_des_knee[i] = fminf(fmaxf(V_MIN, msg->qd_des_knee[i]), V_MAX);
		args_t->setpoints->qd_des_abad[i] = fminf(fmaxf(V_MIN, msg->qd_des_abad[i]), V_MAX);
		args_t->setpoints->qd_des_hip[i] = fminf(fmaxf(V_MIN, msg->qd_des_hip[i]), V_MAX);
		
		args_t->setpoints->kp_knee[i] = fminf(fmaxf(KP_MIN, msg->kp_knee[i]), KP_MAX);
		args_t->setpoints->kp_abad[i] = fminf(fmaxf(KP_MIN, msg->kp_abad[i]), KP_MAX);
		args_t->setpoints->kp_hip[i] = fminf(fmaxf(KP_MIN, msg->kp_hip[i]), KP_MAX);

		args_t->setpoints->kd_knee[i] = fminf(fmaxf(KD_MIN, msg->kd_knee[i]), KD_MAX);
		args_t->setpoints->kd_abad[i] = fminf(fmaxf(KD_MIN, msg->kd_abad[i]), KD_MAX);
		args_t->setpoints->kd_hip[i] = fminf(fmaxf(KD_MIN, msg->kd_hip[i]), KD_MAX);

		args_t->setpoints->tau_knee_ff[i] = fminf(fmaxf(T_MIN, msg->tau_knee_ff[i]), T_MAX);
		args_t->setpoints->tau_abad_ff[i] = fminf(fmaxf(T_MIN, msg->tau_abad_ff[i]), T_MAX);
		args_t->setpoints->tau_hip_ff[i] = fminf(fmaxf(T_MIN, msg->tau_hip_ff[i]), T_MAX);
	}
	/*release mutex*/
	err = rt_mutex_release(&args_t->mutex);
	if (err) 
	{
		printf("Joint PD Controller: Failed to release mutex lock in lcm callback, code %d\n", err);
		isRunning = 0;
		return;
	}
}

void pack_torque_cmd(struct rt_task_args* args){
     //range :-2000~2000, corresponding to the actual torque current range -32A~32A
     /// pack ints into the can buffer ///
    int16_t iqControl;
     for (int j = 0; j < 4; ++j)
     {


       
     	/* knee*/

     	for (int i = 0; i < 3; ++i)
     	{
     	     switch(i)
     	     {
     	     	case 0: 	/*knee*/

     	     		iqControl = MOTOR_Kt * args->motor_torq_setpoints.torq_setpoint_knee[j]; 		
     	     		args->setpoint_msgs.knee_setpoints[j].id = 0x141+ i;
     	     		args->setpoint_msgs.knee_setpoints[j].data[4] = iqControl&0xff;                
     	     		args->setpoint_msgs.knee_setpoints[j].data[5] = (iqControl>>8)&0xff;
     	     	    args->setpoint_msgs.knee_setpoints[j].type =  PCANFD_TYPE_CAN20_MSG;
     	     		args->setpoint_msgs.knee_setpoints[j].data_len= 8;
     	     		args->setpoint_msgs.knee_setpoints[j].data[0] = 0xA1;                                              
     	     	    args->setpoint_msgs.knee_setpoints[j].data[1] = 0;                
     	     	    args->setpoint_msgs.knee_setpoints[j].data[2] = 0;                   
     	     	    args->setpoint_msgs.knee_setpoints[j].data[3] = 0; 
     	     	    args->setpoint_msgs.knee_setpoints[j].data[6] = 0;  
     	     	    args->setpoint_msgs.knee_setpoints[j].data[7] = 0;                 
     	     		break;
     	     	case 1: 	/*hip*/
     	     		iqControl = MOTOR_Kt * args->motor_torq_setpoints.torq_setpoint_hip[j];  		
     	     		args->setpoint_msgs.hip_setpoints[j].id = 0x141+ i;
     	     		args->setpoint_msgs.hip_setpoints[j].data[4] = iqControl&0xff;                
     	     		args->setpoint_msgs.hip_setpoints[j].data[5] = (iqControl>>8)&0xff;  
     	     	    args->setpoint_msgs.hip_setpoints[j].type =  PCANFD_TYPE_CAN20_MSG;
     	     		args->setpoint_msgs.hip_setpoints[j].data_len= 8;
     	     		args->setpoint_msgs.hip_setpoints[j].data[0] = 0xA1;                                              
     	     	    args->setpoint_msgs.hip_setpoints[j].data[1] = 0;                
     	     	    args->setpoint_msgs.hip_setpoints[j].data[2] = 0;                   
     	     	    args->setpoint_msgs.hip_setpoints[j].data[3] = 0; 
     	     	    args->setpoint_msgs.hip_setpoints[j].data[6] = 0;  
     	     	    args->setpoint_msgs.hip_setpoints[j].data[7] = 0;
     	     		break;
     	     	case 2:		/*abad*/ 
     	     		iqControl = MOTOR_Kt * args->motor_torq_setpoints.torq_setpoint_abad[j]; 		
     	     		args->setpoint_msgs.abad_setpoints[j].id = 0x141+ i;
     	     		args->setpoint_msgs.abad_setpoints[j].data[4] = iqControl&0xff;                
     	     		args->setpoint_msgs.abad_setpoints[j].data[5] = (iqControl>>8)&0xff;  
     	     	    args->setpoint_msgs.abad_setpoints[j].type =  PCANFD_TYPE_CAN20_MSG;
     	     		args->setpoint_msgs.abad_setpoints[j].data_len= 8;
     	     		args->setpoint_msgs.abad_setpoints[j].data[0] = 0xA1;                                              
     	     	    args->setpoint_msgs.abad_setpoints[j].data[1] = 0;                
     	     	    args->setpoint_msgs.abad_setpoints[j].data[2] = 0;                   
     	     	    args->setpoint_msgs.abad_setpoints[j].data[3] = 0; 
     	     	    args->setpoint_msgs.abad_setpoints[j].data[6] = 0;  
     	     	    args->setpoint_msgs.abad_setpoints[j].data[7] = 0;
     	     		break;
     	     	
     	     }
   
     	}
         
     }            
    
    }

void unpack_reply(struct pcanfd_msg rx_msgs[], int n, spi_data_t* leg_data, int leg_no)
{
//1. Motor temperatureï¼ˆ 1â„ƒ/LSBï¼‰
//2. Motor torque current(Iq)ï¼ˆRange:-2048~2048,real torque current range:-33A~33Aï¼‰
//3. Motor speedï¼ˆ1dps/LSBï¼‰
//4. Encoder position valueï¼ˆ14bit encoder value range 0~16383ï¼‰
	int64_t pposition;
	int16_t pspeed;


	for (int i = 0; i < n; ++i)
	{
		if(rx_msgs[i].data[0] == 0x9c)
		{
			pspeed = (rx_msgs[i].data[5]<<8)|(rx_msgs[i].data[4]);
			switch (rx_msgs[i].id){
				case 0x141:					/* knee*/
					leg_data->qd_knee[leg_no] = (pspeed*0.01)* DEG_TO_RADIAN;
					break;
				case 0x142:
					leg_data->qd_hip[leg_no] = (pspeed*0.01)* DEG_TO_RADIAN;
					break;
				case 0x143:	
					leg_data->qd_abad[leg_no] = (pspeed*0.01)* DEG_TO_RADIAN;
					break;
			} 
		}
		else if(rx_msgs[i].data[0] == 0x92)
		{
			pposition = ( rx_msgs[i].data[1] | rx_msgs[i].data[2] << 8| \
			rx_msgs[i].data[3] << 16 | rx_msgs[i].data[4] << 24 | \
			(uint64_t)rx_msgs[i].data[5] << 32 |(uint64_t)rx_msgs[i].data[6] << 40 | \
			(uint64_t)rx_msgs[i].data[7] << 48 );

			switch (rx_msgs[i].id){
				/*knee*/
				case 0x141:					
					leg_data->q_knee[leg_no] = (pposition*60*PI)/(16383*180); 
					break;
				/*hip*/
				case 0x142:
					leg_data->q_hip[leg_no] = (pposition*60*PI)/(16383*180);
					break;
				/*abad*/
				case 0x143:	
					leg_data->q_abad[leg_no] = (pposition*60*PI)/(16383*180);
					break;

		}	
		}    
  
	}
}


void print_message(const char *prompt, const struct pcanfd_msgs *msg)
{
	
	for (int i = 0; i < msg->count; ++i)
	{
		rt_printf("%s: %d 0x%08x %1d ",
				prompt,
				 msg->list[i].type,
				 msg->list[i].id,
				 msg->list[i].data_len);
	
		for (int j = 0; j < msg->list[i].data_len; j++)
		{
			rt_printf("%02x ", msg->list[i].data[j]);
		}

		rt_printf("\n");
	}

	
}


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

static void write_task_func(void * arg)
{
	int err_1 = 0, err_2 = 0, err = 0,count = 0;;
	struct rt_task_args* args;
	args = (struct rt_task_args*)arg;
	// clock_gettime(CLOCK_MONOTONIC, &start);

	/*enter infinite loop*/
	while(1)
	{
		/* wait for task release*/
		err = rt_task_wait_period(&args->overruns);
		if(err)
		{
			rt_printf("RT task wait failed with err, exiting=%d\n", err);
			/* lock mutex*/
			err = rt_mutex_acquire_timed(&args->mutex, NULL);
			if (err)
				printf("Joint PD Controller: Failed to acquire mutex lock, code %d\n", err);		
			args->isRunning_rt = 0;
			err = rt_mutex_release(&args->mutex);
			if (err)
				printf("Joint PD Controller: Failed to release mutex lock, code %d\n", err);
			return;
		}

		/* send  0x9c requests on CANBUS*/
		for (int i = 0; i < 4; ++i) /* legs*/
		{
			for (int j = 0; j < 3; ++j) /* joints*/
			{
				/* send request for position*/
				err_1 = pcanfd_send_msg(args->fd[i], &args->req_msgs_pos[j]);
				/* if txq is full then exit the for loop*/
				if (err_1)
				{
					rt_printf("Send msg failed because of errors=%d on port %d\n. Exiting the task", err_1, i);
					/* lock mutex*/
					err = rt_mutex_acquire_timed(&args->mutex, NULL);
					if (err)
						printf("Joint PD Controller: Failed to acquire mutex lock, code %d\n", err);		
					args->isRunning_rt = 0;
					err = rt_mutex_release(&args->mutex);
					if (err) 
						printf("Joint PD Controller: Failed to release mutex lock, code %d\n", err);
					return;		
	
				}
				
			}
			
		}
		
		/* send 0x92 requests on CANBUS*/
		for (int i = 0; i < 4; ++i) /* legs*/
		{
			for (int j = 0; j < 3; ++j) /* joints*/
			{
				/* send request for position*/
				err_2 = pcanfd_send_msg(args->fd[i], &args->req_msgs_vel[j]);
				/* if txq is full then exit the for loop*/
				if (err_2)
				{
					rt_printf("Send msg failed because of errors=%d, %d\n. Exiting the task", err_2);
					/* lock mutex*/
					err = rt_mutex_acquire_timed(&args->mutex, NULL);
					if (err)
						printf("Joint PD Controller: Failed to acquire mutex lock, code %d\n", err);		
					args->isRunning_rt = 0;
					err = rt_mutex_release(&args->mutex);
					if (err) 
						printf("Joint PD Controller: Failed to release mutex lock, code %d\n", err);
					return;		
				}
			}
		}

		/* read responses*/
		for (int i = 0; i < 4; ++i)
		{
				int msg_count = 0;
				while(1)
				{
					err = pcanfd_recv_msg(args->fd[i], &args->res_msgs[msg_count]);
					if(err == -EWOULDBLOCK) /* no msgs left to read*/
						break;
					else if(err)
					{
						rt_printf("Recv msg failed because of error=%d\n. Exiting the task", err);
						/* lock mutex*/
						err = rt_mutex_acquire_timed(&args->mutex, NULL);
						if (err)
							printf("Joint PD Controller: Failed to delete rt task, code %d\n", err);		
						args->isRunning_rt = 0;
						err = rt_mutex_release(&args->mutex);
						if (err)
							printf("Joint PD controler : release mutex lock, code %d\n", err);
						return;
					} 
				
				 // else
				// 	print_message_single(ports[i], &args->res_msgs[msg_count]);	
					msg_count++;

				}

			/* parse responses leg-wise*/
			unpack_reply(args->res_msgs, msg_count, &args->leg_data, i);

		}
	
		/* do control computations*/
		control_comp(args);
		/* stuff data to can msg*/
		pack_torque_cmd(args);
		/* publish on lcm before sending over CANBUS*/

		
		/* send msgs*/
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				switch(j)
				{
					case 0:		/*knee*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.knee_setpoints[i]);
						if (err)
						{
							rt_printf("Send msg failed because of errno=%d\n, exiting", err);
							/* lock mutex*/
							err = rt_mutex_acquire_timed(&args->mutex, NULL);
							if (err)
								printf("Joint PD Controller: Failed to lock mutex, code %d\n", err);		
							args->isRunning_rt = 0;
							/* release mutex*/
							err = rt_mutex_release(&args->mutex);
							if (err)
								printf("Joint PD Controller : Failed to release mutex lock, code %d\n", err);		
							return;
						}
						break;
					case 1:		/*hip*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.hip_setpoints[i]);
						if (err)
						{
							rt_printf("Send msg failed because of err=%d\n, exiting", err);
							/* lock mutex*/
							err = rt_mutex_acquire_timed(&args->mutex, NULL);
							if (err)
								printf("Joint PD Controller: Failed to lock mutex, code %d\n", err);		
							args->isRunning_rt = 0;
							err = rt_mutex_release(&args->mutex);
							if (err)
								printf("Joint PD Controller : Failed to release mutex lock, code %d\n", err);		
							return;
						}
						break;
					case 2:		/*abad*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.abad_setpoints[i]);
						if (err)
						{
							rt_printf("Send msg failed because of err=%d\n, exiting", err);
							/* lock mutex*/
							err = rt_mutex_acquire_timed(&args->mutex, NULL);
							if (err)
								printf("Joint PD Controller: Failed to delete rt task, code %d\n", err);		
							args->isRunning_rt = 0;
							err = rt_mutex_release(&args->mutex);
							if (err)
								printf("Joint PD Controller : Failed to release mutex lock, code %d\n", err);		
							return;
						}
						break;
				}
				

			}
			
		}


	}

}



static void signal_handler(int _)
{
	(void)_;
	isRunning = 0;
}

int main()
{
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);
	int err = 0;
	int lcm_fd;
	fd_set fds;
	/*timeout for lcm select */
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;
	/* for isolating CPUs*/
	cpu_set_t cpus;
	CPU_ZERO(&cpus);
	CPU_SET(1,&cpus);
	CPU_SET(2,&cpus);
	CPU_SET(3,&cpus);


	struct rt_task_args* args = malloc(sizeof(struct rt_task_args));
	if(!args)
	{
		printf("Joint PD controller (main): Malloc for args failed, exiting program\n");
		return 0;
	}

	args->setpoints = (spi_command_t*)malloc(sizeof(spi_command_t));
	if(!args->setpoints)
	{
		printf("Joint PD controller (main):Malloc for args->setpoints failed, exiting program\n");
		return 0;
	}
	args->isRunning_rt = 1;

	struct pcanfd_msg_filter* msg_filter =  malloc(sizeof(struct pcanfd_msg_filter));
	if(!msg_filter)
	{
		printf("Joint PD controller (main):Malloc for msg_filter failed, exiting program\n");
		return 0;
	}

	/*init lcm*/
	lcm_t *lcm = lcm_create(NULL);
	if(!lcm)
	{
		printf("Joint PD controller (main):Malloc for lcm failed, exiting program\n");
		return 0;
	}

	/* declare lcm subscriber*/
	spi_command_t_subscription_t *sub = spi_command_t_subscribe(lcm, "spine_command", &lcm_recv_handler, &args);

	

	/* open can ports*/
	for (int i = 0; i < 4; ++i)
	{
		args->fd[i] = pcanfd_open(ports[i], OFD_BITRATE  | PCANFD_INIT_STD_MSG_ONLY | OFD_NONBLOCKING, 1000000);
		if (args->fd[i] < 0)
		{
			printf("Joint PD controller (main):Pcanfd open failed with err %d on port no. %d\n",args->fd[i], i);
			return 0;
		}
	}

	/* set options to allow only STD-CAN msgs */
	uint32_t allowed_msg = PCANFD_ALLOWED_MSG_CAN;
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_set_option(args->fd[i], PCANFD_OPT_ALLOWED_MSGS, &allowed_msg, sizeof(allowed_msg));
		if (err) 
		{
			printf("Joint PD Controller (main): Failed to set_option on fd %d with errno %d\n", args->fd[i], err);
			return 0;

		}
				
	}

	/* add filters on CANID for safety, allow only CANIDS 141 - 143 */
	msg_filter->id_from = 0x141;
	msg_filter->id_to = 0x143;
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_add_filter(args->fd[i], msg_filter);
		if (err) 
		{
			printf("Joint PD Controller (main): Failed to add_filter on fd %d with errno %d\n", args->fd[i], err);
			return 0;

		}
				
	}
	
	/* init req msgs*/
	/* 0x9c is the command for getting the velocity*/
	for (int i = 0; i < 3; ++i)
	{
		args->req_msgs_vel[i].type = PCANFD_TYPE_CAN20_MSG;
		args->req_msgs_vel[i].data_len = 8;
		args->req_msgs_vel[i].id = (0x141 +i);
		for (int j = 1; j < 8; ++j)
		{
			args->req_msgs_vel[i].data[j] = 0;	
		}
		
		args->req_msgs_vel[i].data[0] = 0x9c;
			
	}
	/*0x92 is the command for getting the multi turns angle*/
	for (int i = 0; i < 3; ++i)
	{
		args->req_msgs_pos[i].type = PCANFD_TYPE_CAN20_MSG;
		args->req_msgs_pos[i].data_len = 8;
		args->req_msgs_pos[i].id = (0x141 + i);
		for (int j = 1; j < 8; ++j)
		{
			args->req_msgs_pos[i].data[j] = 0;	
		}
		
		args->req_msgs_pos[i].data[0] = 0x92;
			
	}

	/*for avoiding page faults*/	
	mlockall(MCL_CURRENT | MCL_FUTURE);

	/*create real_time task*/
	err = rt_task_create(&args->write_task, "write_task", 0, 99, 0);
	if (err) {
		printf("Joint PD Controller (main): Failed to create rt task, code %d\n", err);
		return err;
	}
	/* set affinity to isolate real-time task to only 2 cpus*/
	err = rt_task_set_affinity(&args->write_task, &cpus);
	if (err) {
		printf("Joint PD Controller (main): Failed to create rt task, code %d\n", err);
		return err;
	}
	/* set peridic task*/

	err = rt_task_set_periodic(&args->write_task, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
	if (err) {
		printf("Joint PD Controller (main): Failed set periodic task, code %d\n" ,err);
		return err;
	}
	/* create mutex object*/
	err = rt_mutex_create(&args->mutex, "mutex_lock");
	if (err) {
		printf("Joint PD Controller (main): Failed to create mutex object code %d\n" ,err);
		return err;
	}

	err = rt_task_start(&args->write_task, write_task_func, args);
	if (err) {
		printf("Joint PD Controller (main): Failed to start rt task, code %d\n", err);
		return err;
	}


	/*do nothing, blocking call*/
	int isRunning_rt_copy = 1;
	while(isRunning && isRunning_rt_copy)
	{
		/* setup  up lcm callback in non-blocking mode*/
		lcm_fd = lcm_get_fileno(lcm);
		FD_ZERO(&fds);
		FD_SET(lcm_fd, &fds);
		/* do select() on linux fd*/
		int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

		if (0 == status) {
		    // no messages
		    printf("waiting for message\n");
		} else if (FD_ISSET(lcm_fd, &fds)) {
		    // LCM has events ready to be processed.
		    lcm_handle(lcm);
		}
		
		/*lock mutex*/
		err = rt_mutex_acquire_timed(&args->mutex,NULL);
		if (err == -EPERM ) {
			printf("Joint PD Controller (main): Failed to acquire mutex, code %d\n", err);
			return err;
		}
		isRunning_rt_copy = args->isRunning_rt;
		/*unlock mutex*/
		rt_mutex_release(&args->mutex);
		if (err) {
			printf("Joint PD Controller (main): Failed to release mutex, code %d\n", err);
			return err;
		}
	}


	/*cleanup*/
	lcm_destroy(lcm);
			err = rt_task_delete(&args->write_task);
	if (err) {
		printf("Joint PD Controller (main): Failed to delete rt task, code %d\n", err);		
		return err;
	}
	free(args->setpoints);
	free(args);
	/* switch off all motors*/
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_close(args->fd[i]);
		if (err != -1)
			printf("Joint PD Controller (main):Pcanfd close failed with err %d\n",err);
	}

	return 0;

}
