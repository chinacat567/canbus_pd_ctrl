/*sumantra sharma 25th March 2021*/
/* 28th March - added mutex locks between LCM and RT thread*/
/* 29th March - mutex sharing between RT and non RT not possible. Use msg pipes instead*/
/* 8th April - moved to pthreads (posix skin) from alchemy */
/*12th April - moved imu task to seperate process*/
#include "motors_can.h"

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
  /* for simplicity, ignoring possibilities of signal wakes */
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}


static void periodic_task_init(struct period_info *pinfo, long int setperiod)
{
  /* for simplicity, default is a 100ms period */
	printf("period %ld set\n", setperiod);
  pinfo->period_ns = setperiod; 
  clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

void print_spi_command(spi_command_t* msg)
{
	for (int i = 0; i < 1; ++i)
	{
		rt_printf("\n");
		// rt_printf("Leg no. %d :\n", i);
		// rt_printf("Knee setpoints:\n");
		rt_printf("q_des_knee[%d] = %f\n",i, msg->q_des_knee[i]);
		rt_printf("qd_des_knee[%d] = %f\n",i, msg->qd_des_knee[i]);
		rt_printf("kp_knee[%d] = %f\n",i, msg->kp_knee[i]);
		rt_printf("kd_knee[%d] = %f\n",i, msg->kd_knee[i]);
		rt_printf("tau_ff_knee[%d] = %f\n",i, msg->tau_knee_ff[i]);

		// rt_printf("Hip setpoints:\n");
		rt_printf("q_des_hip[%d] = %f\n",i,  msg->q_des_hip[i]);
		rt_printf("qd_des_hip[%d] = %f\n",i,  msg->qd_des_hip[i]);
		rt_printf("kp_hip[%d] = %f\n",i,  msg->kp_hip[i]);
		rt_printf("kd_hip[%d] = %f\n",i,  msg->kd_hip[i]);
		rt_printf("tau_ff_hip[%d] = %f\n",i,  msg->tau_hip_ff[i]);

		// // rt_printf("Abad setpoints:\n");
		rt_printf("q_des_abad[%d] = %f\n",i, msg->q_des_abad[i]);
		rt_printf("qd_des_abad[%d] = %f\n",i, msg->qd_des_abad[i]);
		rt_printf("kp_abad[%d] = %f\n",i, msg->kp_abad[i]);
		rt_printf("kd_abad[%d] = %f\n",i, msg->kd_abad[i]);
		rt_printf("tau_ff_abad[%d] = %f\n",i, msg->tau_abad_ff[i]);



	}
}

void print_spi_data(spi_data_t* msg)
{
	for (int i = 0; i < 2; ++i)
	{
		rt_printf("\n");
		rt_printf("Leg no. %d :\n", i);
		rt_printf("Knee data:\n");
		rt_printf("q_knee[%d] = %f\n",i, msg->q_knee[i]);
		rt_printf("qd_knee[%d] = %f\n",i, msg->qd_knee[i]);
		
		// rt_printf("Hip data:\n");
		rt_printf("q_hip[%d] = %f\n",i, msg->q_hip[i]);
		rt_printf("qd_hip[%d] = %f\n",i, msg->qd_hip[i]);
	
		// rt_printf("Abad data:\n");
		rt_printf("q_abad[%d] = %f\n",i, msg->q_abad[i]);
		rt_printf("qd_abad[%d] = %f\n",i, msg->qd_abad[i]);
		
	}
}



void print_torq_data(struct motor_torq_cmd* msg)
{
	for (int i = 0; i < 1; ++i)
	{
		rt_printf("\n");
		rt_printf("torq_knee[%d] = %f\n",i, msg->torq_setpoint_knee[i]);
		rt_printf("torq_hip[%d] = %f\n",i, msg->torq_setpoint_hip[i]);		
		rt_printf("torq_abad[%d] = %f\n",i, msg->torq_setpoint_abad[i]);
		
	}
}


void control_comp(struct rt_task_args *args)
{
	int err;

	
		/* tau = tau_ff + kp * (theta-theta_r) + kd * (v-v_r)*/
		for (int i = 0; i < 4; ++i)
		{
			#ifdef ESTOP
			if(args->leg_data.q_knee[i] > K_LIM_P)
			{
				rt_printf("IMPORTANT: ESTOP APPLIED to knee[%d]\n" , i);
				args->setpoints.q_des_knee[i] = K_LIM_P;
				args->setpoints.qd_des_knee[i] = 0.0f;
				args->setpoints.kp_knee[i] = 0;
				args->setpoints.kd_knee[i] = KD_SOFTSTOP;
				args->setpoints.tau_knee_ff[i] += KP_SOFTSTOP*(K_LIM_P - args->leg_data.q_knee[i]);
				args->leg_data.flags[i] |= 1;

			}
				else if (args->leg_data.q_knee[i] < K_LIM_N)
			{
				rt_printf("IMPORTANT: ESTOP APPLIED to knee[%d]\n", i);
				args->setpoints.q_des_knee[i] = K_LIM_N;
				args->setpoints.qd_des_knee[i] = 0.0f;
				args->setpoints.kp_knee[i] = 0;
				args->setpoints.kd_knee[i] = KD_SOFTSTOP;
				args->setpoints.tau_knee_ff[i] += KP_SOFTSTOP*(K_LIM_N - args->leg_data.q_knee[i]);
				args->leg_data.flags[i] = 1;

			}
			#endif

			args->motor_torq_setpoints.torq_setpoint_knee[i] = \
			args->setpoints.tau_knee_ff[i]	+ (args->setpoints.kp_knee[i]) * (args->setpoints.q_des_knee[i] - args->leg_data.q_knee[i]) + \
			(args->setpoints.kd_knee[i]) * (args->setpoints.qd_des_knee[i] - args->leg_data.qd_knee[i]);

			#ifdef ESTOP
			if(args->leg_data.q_hip[i] > H_LIM_P)
			{
				rt_printf("IMPORTANT: ESTOP APPLIED to hip[%d]\n", i);
				args->setpoints.q_des_hip[i] = H_LIM_P;
				args->setpoints.qd_des_hip[i] = 0.0f;
				args->setpoints.kp_hip[i] = 0;
				args->setpoints.kd_hip[i] = KD_SOFTSTOP;
				args->setpoints.tau_hip_ff[i] += KP_SOFTSTOP*(H_LIM_P - args->leg_data.q_hip[i]);
				args->leg_data.flags[i] = 2;

			}
			else if (args->leg_data.q_hip[i] < H_LIM_N)
			{
				rt_printf("IMPORTANT: ESTOP APPLIED to hip[%d]\n", i);
				args->setpoints.q_des_hip[i] = H_LIM_N;
				args->setpoints.qd_des_hip[i] = 0.0f;
				args->setpoints.kp_hip[i] = 0;
				args->setpoints.kd_hip[i] = KD_SOFTSTOP;
				args->setpoints.tau_hip_ff[i] += KP_SOFTSTOP*(H_LIM_N - args->leg_data.q_hip[i]);
				args->leg_data.flags[i] = 2;

			}
			#endif

			args->motor_torq_setpoints.torq_setpoint_hip[i] = \
			args->setpoints.tau_hip_ff[i]	+ (args->setpoints.kp_hip[i]) * (args->setpoints.q_des_hip[i] - args->leg_data.q_hip[i]) + \
			(args->setpoints.kd_hip[i]) * (args->setpoints.qd_des_hip[i] - args->leg_data.qd_hip[i]);

			#ifdef ESTOP
			if(args->leg_data.q_abad[i] > A_LIM_P)
			{
				rt_printf("IMPORTANT: ESTOP APPLIED to abad[%d]\n", i);
				args->setpoints.q_des_abad[i] = A_LIM_P;
				args->setpoints.qd_des_abad[i] = 0.0f;
				args->setpoints.kp_abad[i] = 0;
				args->setpoints.kd_abad[i] = KD_SOFTSTOP;
				args->setpoints.tau_abad_ff[i] += KP_SOFTSTOP*(A_LIM_P - args->leg_data.q_abad[i]);
				args->leg_data.flags[i] = 3;

			}
			else if (args->leg_data.q_abad[i] < A_LIM_N)
			{
				rt_printf("IMPORTANT: ESTOP APPLIED to abad[%d]\n", i);
				args->setpoints.q_des_abad[i] = A_LIM_N;
				args->setpoints.qd_des_abad[i] = 0.0f;
				args->setpoints.kp_abad[i] = 0;
				args->setpoints.kd_abad[i] = KD_SOFTSTOP;
				args->setpoints.tau_abad_ff[i] += KP_SOFTSTOP*(A_LIM_N - args->leg_data.q_abad[i]);
				args->leg_data.flags[i] = 3;

			}
			#endif


			args->motor_torq_setpoints.torq_setpoint_abad[i] = \
			args->setpoints.tau_abad_ff[i]	+ (args->setpoints.kp_abad[i]) * (args->setpoints.q_des_abad[i] - args->leg_data.q_abad[i]) + \
			(args->setpoints.kd_abad[i]) * (args->setpoints.qd_des_abad[i] - args->leg_data.qd_abad[i]);
	
		}

}

void apply_safey_limits(spi_command_t *setpoints)
{
	
	/* apply safety limits*/
	for (int i = 0; i < 4; ++i)
	{	
		setpoints->q_des_knee[i] = fminf(fmaxf(P_MIN, setpoints->q_des_knee[i]), P_MAX);
		if (setpoints->q_des_knee[i] == P_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to q_des_knee[%d]\n",i);
			#endif

		}
		setpoints->q_des_abad[i] = fminf(fmaxf(P_MIN, setpoints->q_des_abad[i]), P_MAX);
		if (setpoints->q_des_abad[i] == P_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to q_des_abad[%d]\n",i);
			#endif

		}
		setpoints->q_des_hip[i] = fminf(fmaxf(P_MIN, setpoints->q_des_hip[i]), P_MAX);
		if (setpoints->q_des_hip[i] == P_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to q_des_hip[%d]\n",i);
			#endif

		}

		setpoints->qd_des_knee[i] = fminf(fmaxf(V_MIN, setpoints->qd_des_knee[i]), V_MAX);
		if (setpoints->qd_des_knee[i] == V_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to qd_des_knee[%d]\n",i);
			#endif

		}
		setpoints->qd_des_abad[i] = fminf(fmaxf(V_MIN, setpoints->qd_des_abad[i]), V_MAX);
		if (setpoints->qd_des_abad[i] == V_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to qd_des_abad[%d]\n",i);
			#endif

		}
		setpoints->qd_des_hip[i] = fminf(fmaxf(V_MIN, setpoints->qd_des_hip[i]), V_MAX);
		if (setpoints->qd_des_hip[i] == V_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to qd_des_hip[%d]\n",i);
			#endif

		}
		
		setpoints->kp_knee[i] = fminf(fmaxf(KP_MIN, setpoints->kp_knee[i]), KP_MAX);
		if (setpoints->kp_knee[i] == KP_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to kp_knee[%d]\n",i);
			#endif

		}
		setpoints->kp_abad[i] = fminf(fmaxf(KP_MIN, setpoints->kp_abad[i]), KP_MAX);
		if (setpoints->kp_abad[i] == KP_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to kp_abad[%d]\n",i);
			#endif

		}
		setpoints->kp_hip[i] = fminf(fmaxf(KP_MIN, setpoints->kp_hip[i]), KP_MAX);
		if (setpoints->kp_hip[i] == KP_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to kp_hip[%d]\n",i);
			#endif

		}

		setpoints->kd_knee[i] = fminf(fmaxf(KD_MIN, setpoints->kd_knee[i]), KD_MAX);
		if (setpoints->kd_knee[i] == KD_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to kd_knee[%d]\n",i);
			#endif

		}
		setpoints->kd_abad[i] = fminf(fmaxf(KD_MIN, setpoints->kd_abad[i]), KD_MAX);
		if (setpoints->kd_abad[i] == KD_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to kd_abad[%d]\n",i);
			#endif

		}
		setpoints->kd_hip[i] = fminf(fmaxf(KD_MIN, setpoints->kd_hip[i]), KD_MAX);
		if (setpoints->kd_hip[i] == KD_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to kd_hip[%d]\n",i);
			#endif

		}

		setpoints->tau_knee_ff[i] = fminf(fmaxf(T_MIN, setpoints->tau_knee_ff[i]), T_MAX);
		if (setpoints->tau_knee_ff[i] == T_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to tau_knee_ff[%d]\n",i);
			#endif

		}
		setpoints->tau_abad_ff[i] = fminf(fmaxf(T_MIN, setpoints->tau_abad_ff[i]), T_MAX);
		if (setpoints->tau_abad_ff[i] == T_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to tau_abad_ff[%d]\n",i);
			#endif

		}
		setpoints->tau_hip_ff[i] = fminf(fmaxf(T_MIN, setpoints->tau_hip_ff[i]), T_MAX);
		if (setpoints->tau_hip_ff[i] == T_MAX)
		{
			#ifdef DEBUG
			rt_printf("Applied safety limit to tau_hip_ff[%d]\n",i);
			#endif

		}
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

     	     		iqControl = ((args->motor_torq_setpoints.torq_setpoint_knee[j])/ MOTOR_Kt) * CURRENT_SCALING; 
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
     	     		iqControl = ((args->motor_torq_setpoints.torq_setpoint_hip[j])/ MOTOR_Kt) * CURRENT_SCALING;  
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
     	     		iqControl = ((args->motor_torq_setpoints.torq_setpoint_abad[j])/ MOTOR_Kt) * CURRENT_SCALING; 	
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

void unpack_reply(struct pcanfd_msg *rx_msgs, spi_data_t* leg_data, int leg_no)
{
//1. Motor temperatureï¼ˆ 1â„ƒ/LSBï¼‰
//2. Motor torque current(Iq)ï¼ˆRange:-2048~2048,real torque current range:-33A~33Aï¼‰
//3. Motor speedï¼ˆ1dps/LSBï¼‰
//4. Encoder position valueï¼ˆ14bit encoder value range 0~16383ï¼‰
	int64_t pposition;
	int16_t pspeed;

	if(rx_msgs->data[0] == 0x9c)
	{
		pspeed = (rx_msgs->data[5]<<8)|(rx_msgs->data[4]);
		switch (rx_msgs->id){
			case 0x141:					/* knee*/
				leg_data->qd_knee[leg_no] = (pspeed/6)* DEG_TO_RADIAN;
				break;
			case 0x142:
				leg_data->qd_hip[leg_no] = (pspeed/6)* DEG_TO_RADIAN;
				break;
			case 0x143:	
				leg_data->qd_abad[leg_no] = (pspeed/6)* DEG_TO_RADIAN;
				break;
		} 
	}
	else if(rx_msgs->data[0] == 0x92)
	{
		pposition = ( rx_msgs->data[1] | rx_msgs->data[2] << 8| \
		rx_msgs->data[3] << 16 | rx_msgs->data[4] << 24 | \
		(uint64_t)rx_msgs->data[5] << 32 |(uint64_t)rx_msgs->data[6] << 40 | \
		(uint64_t)rx_msgs->data[7] << 48 );

		switch (rx_msgs->id)
		{
			/*knee*/
			case 0x141:					
				leg_data->q_knee[leg_no] = offset_knee[leg_no] + (pposition/600)* DEG_TO_RADIAN;
				break;
			/*hip*/
			case 0x142:
				leg_data->q_hip[leg_no] = offset_hip[leg_no] + (pposition/600)* DEG_TO_RADIAN;

				break;
			/*abad*/
			case 0x143:	
				leg_data->q_abad[leg_no] = (pposition/600)* DEG_TO_RADIAN;
				break;

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


void* write_task_func(void * arg)
{
	int err = 0,count = 0, isReturn = 0;
	struct rt_task_args* args;
	args = (struct rt_task_args*)arg;
	int prio = 1;
	struct period_info pinfo;
	periodic_task_init(&pinfo,TASK_PERIOD_MOTOR);
	int isRunning_local=1;
	/*enter infinite loop*/
	while(isRunning_local)
	{

		/* read setpoints from q*/
		err = mq_receive(args->msg_q_cmd, (char *) &args->setpoints, sizeof(spi_command_t), &prio);
		/*debug*/
		memset(&args->setpoints, 0 , sizeof(spi_command_t));
		if(err < 0)
		{
			if(errno == EAGAIN)
			{
				#ifdef DEBUG
				rt_printf("[MOTOR-RT-TASK] :No msg available in cmd msg q, skipping\n");
				#endif 	
			}
			else if(errno == EBADF)
			{
				rt_printf("[MOTOR-RT-TASK] :Invalid fd for cmd msg q\n");
				return NULL;
			}
			else if(errno == EMSGSIZE)
			{
				rt_printf("[MOTOR-RT-TASK] :Invalid msg size for cmd msg q\n");
				return NULL;
			}
		}
		else if (err == sizeof(spi_command_t))
		{
			#ifdef DEBUG
			rt_printf("[MOTOR-RT-TASK] : Read setpoints from higher controller\n");	
			#endif	
	
		}
		else
		{
			rt_printf("[MOTOR-RT-TASK]: DANGER : Read incomplete setpoints from higher controller %d\n");
			return NULL;
		}
		/* read status from q*/
		err = mq_receive(args->msg_q_status, (char *)(&status), sizeof(int), &prio);
		if(err < 0)
		{
			if(errno == EAGAIN)
			{
				#ifdef DEBUG
				rt_printf("[MOTOR-RT-TASK] :No msg available in status msg q, skipping\n");
				#endif 	
			}
			else if(errno == EBADF)
			{
				rt_printf("[MOTOR-RT-TASK] :Invalid fd for status msg q\n");
				return NULL;
			}
			else if(errno == EMSGSIZE)
			{
				rt_printf("[MOTOR-RT-TASK] :Invalid msg size for status msg q\n");
				return NULL;
			}
		}
		else if (err == sizeof(int))
		{
			#ifdef DEBUG
			rt_printf("[MOTOR-RT-TASK] : Read status msg from higher controller\n");	
			#endif	
		}
		else
		{
			rt_printf("[MOTOR-RT-TASK]: DANGER : Read incomplete status from higher controller %d\n");
			return NULL;
		}

		/*acquire mutex*/	
		err = pthread_mutex_trylock(&mutex);

		if(err)
		{
			if(errno != EBUSY)
			{
				rt_printf("[MOTOR-RT-TASK] : Invalid context, failed to lock mutex\n");
				return NULL;
			}	
		}

		else
		{

			isRunning_local = isRunning;
		}

		err = pthread_mutex_unlock(&mutex);
		if(err)
		{
			if(errno == EPERM)
			{
				rt_printf("[MOTOR-RT-TASK] : Invalid context, failed to unlock mutex\n");
				return NULL;
			}
			else if (errno == EINVAL)
			{
				rt_printf("[MOTOR-RT-TASK] : Invalid mutex, failed to unlock\n");
				return NULL;
			}
			else
			{
				rt_printf("[MOTOR-RT-TASK] : Failed to release mutex\n");
				return NULL;
			}

		}
			
		#ifdef DEBUG
		print_spi_command(&args->setpoints);
		#endif 
		apply_safey_limits(&args->setpoints);
		
		/* send  0x9c requests on CANBUS*/
		for (int i = 0; i < 4; ++i) /* legs*/
		{
			for (int j = 0; j < 3; ++j) /* joints*/
			{
				/* send request for position*/
				err = pcanfd_send_msg(args->fd[i], &args->req_msgs_pos[j]);
				/* if failed to send req then exit*/
				if (err)
				{
					if(err == -EWOULDBLOCK)
					{
		
						rt_printf("[MOTOR-RT-TASK] :DANGER :Failed to write msg to CANBUS, Tx Queue is full\n");

					}
					else
					{
						return NULL;			

					}
				}
				
			}
			
		}
		
		/* send 0x92 requests on CANBUS*/
		for (int i = 0; i < 4; ++i) /* legs*/
		{
			for (int j = 0; j < 3; ++j) /* joints*/
			{
				/* send request for position*/
				err = pcanfd_send_msg(args->fd[i], &args->req_msgs_vel[j]);
				/* if failed to send req then exit*/
				if (err)
				{
					if(err == -EWOULDBLOCK)
					{
		
						rt_printf("[MOTOR-RT-TASK] :DANGER :Failed to write msg to CANBUS, Tx Queue is full\n");

					}
					else
					{
						return NULL;	
					}
							
				}
			}

		}

		/* read responses*/
		for (int i = 0; i < 4; ++i)
		{
			/* get state*/
			err = pcanfd_get_state(args->fd[i], &state[i]);
			if(err)
			{
				rt_printf("[MOTOR-RT-TASK] :Failed to get status, invalid fd. Exiting\n");
				return NULL;
			}
			/* check for buss errors*/
			if (state[i].tx_error_counter || state[i].rx_error_counter)
			{
				rt_printf("[MOTOR-RT-TASK] : Tx/Rx errors detected on CANBUS port %d. Check BUS connection\n", i);
				return NULL;

			}	

			count = state[i].rx_pending_msgs;
			/* read pending msgs*/
			for (int j = 0; j < count; ++j)
			{
				err = pcanfd_recv_msg(args->fd[i], &args->res_msgs[j]);
				/* if msg queue is empty, break from the loop, else return*/
				if (err)
				{
					if(err == -EWOULDBLOCK)
					{
						rt_printf("[MOTOR-RT-TASK] :DANGER : Read lesser msgs than avail. in q %d\n", err);	
						break;
					}
					else
					{
						rt_printf("[MOTOR-RT-TASK] :DANGER : Recv msg failed because of error no. %d\n", err);	
						return NULL;
					}
				}
				
				else
				{
				 	#ifdef DEBUG
					print_message_single(ports[i], &args->res_msgs[j]);	
					#endif
					unpack_reply(&args->res_msgs[j], &args->leg_data, i);				
				} 

			}
			

		}
				

		/* do control computations*/
		control_comp(args);
		/* stuff data to can msg*/
		pack_torque_cmd(args);
		#ifdef DEBUG
		print_spi_data(&args->leg_data);
		print_torq_data(&args->motor_torq_setpoints);
		#endif 
		/* publish on msg q before sending over CANBUS if higher controller is running*/
		if (status)
		{
			err = mq_send(args->msg_q_data, (char *) (&args->leg_data), sizeof(spi_data_t), 1);
			if(err < 0)
			{
				if(errno == EAGAIN)
				{
					#ifdef DEBUG
					rt_printf("[MOTOR-RT-TASK] :Data msg q full, skipping\n");
					#endif 	
				}
				else if(errno == EBADF)
				{
					rt_printf("[MOTOR-RT-TASK] :Invalid fd for data msg q\n");
					return NULL;
				}
				else if(errno == EMSGSIZE)
				{
					rt_printf("[MOTOR-RT-TASK] :Invalid msg size for data msg q\n");
					return NULL;
				}
			}

			else
			{
				#ifdef DEBUG
				rt_printf("[MOTOR-RT-TASK] : Sent motor data to higher controller\n");
				#endif
			}
		}

		else
		{
			#ifdef DEBUG
			rt_printf("[MOTOR-RT-TASK] :Motors task : Waiting for higher controller\n ");
			#endif
		}
		
		
		/* send msgs*/
		#ifdef CAN_WRITE
		for (int i = 0; i < 4; ++i)
		{		

			for (int j = 0; j < 3; ++j)
			{
				switch(j)
				{
					case 0:		/*knee*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.knee_setpoints[i]);
						/* if failed to send req then exit*/
						if (err)
						{
							if(err == -EWOULDBLOCK)
							{
								rt_printf("[MOTOR-RT-TASK] :DANGER :Failed to write msg to CANBUS\n");
								return NULL;
							}
							else
							{
								return NULL;	
							}
									
						}
						break;
					case 1:		/*hip*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.hip_setpoints[i]);
						/* if failed to send req then exit*/
						if (err)
						{
							if(err == -EWOULDBLOCK)
							{
								rt_printf("[MOTOR-RT-TASK] :DANGER :Failed to write msg to CANBUS\n");
								return NULL;
							}
							else
							{
								return NULL;
							}
										
						}
						break;
					case 2:		/*abad*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.abad_setpoints[i]);
						/* if failed to send req then exit*/
						if (err)
						{
							if(err == -EWOULDBLOCK)
							{
				
								rt_printf("[MOTOR-RT-TASK] :DANGER :Failed to write msg to CANBUS\n");
								return NULL;

							}
							else
							{
								return NULL;
							}
										
						}
						break;
				}
				

			}
			
		}
		#endif 

		wait_rest_of_period(&pinfo);
	}
	rt_printf("[MOTOR-RT-TASK]: Exiting pthread");

}



int main()
{
	

	int err;
	/* for isolating CPUs*/
	cpu_set_t cpu_motors;
	CPU_ZERO(&cpu_motors);
	CPU_SET(1,&cpu_motors);
	/*pthread tasks*/
	pthread_t motor_thread;
	struct rt_task_args args;
	memset(&args, 0, sizeof(struct rt_task_args));

	// // DEBUG
	// memset(args.setpoints, 0 , sizeof(spi_command_t));
	// args.setpoints->q_des_hip[0] = -0.78;
	// args.setpoints->kp_hip[0] = 70;
	// args.setpoints->kd_hip[0] = 10;
	
	/* for settingf canbus filters*/
	struct pcanfd_msg_filter msg_filter;

	/* open can ports*/
	for (int i = 0; i < 4; ++i)
	{
		args.fd[i] = pcanfd_open(ports[i], OFD_BITRATE  | PCANFD_INIT_STD_MSG_ONLY | OFD_NONBLOCKING, 1000000);
		if (args.fd[i] < 0)
		{
			rt_printf("[MOTOR-RT-TASK] : Main :Pcanfd open failed with err %d on port no. %d\n",args.fd[i], i);
			return 0;
		}
	}


	/* set options to allow only STD-CAN msgs */
	uint32_t allowed_msg = PCANFD_ALLOWED_MSG_CAN;
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_set_option(args.fd[i], PCANFD_OPT_ALLOWED_MSGS, &allowed_msg, sizeof(allowed_msg));
		if (err) 
		{
			rt_printf("[MOTOR-RT-TASK] : Main : Failed to set_option on fd %d with errno %d\n", args.fd[i], err);
			return 0;

		}
				
	}

	/* add filters on CANID for safety, allow only CANIDS 141 - 143 */
	msg_filter.id_from = 0x141;
	msg_filter.id_to = 0x143;
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_add_filter(args.fd[i], &msg_filter);
		if (err) 
		{
			rt_printf("[MOTOR-RT-TASK] : Main : Failed to add_filter on fd %d with errno %d\n", args.fd[i], err);
			return 0;

		}
				
	}

			
	/* init req msgs*/
	/* 0x9c is the command for getting the velocity*/
	for (int i = 0; i < 3; ++i)
	{
		args.req_msgs_vel[i].type = PCANFD_TYPE_CAN20_MSG;
		args.req_msgs_vel[i].data_len = 8;
		args.req_msgs_vel[i].id = (0x141 +i);
		for (int j = 1; j < 8; ++j)
		{
			args.req_msgs_vel[i].data[j] = 0;	
		}
		
		args.req_msgs_vel[i].data[0] = 0x9c;
			
	}
	/*0x92 is the command for getting the multi turns angle*/
	for (int i = 0; i < 3; ++i)
	{
		args.req_msgs_pos[i].type = PCANFD_TYPE_CAN20_MSG;
		args.req_msgs_pos[i].data_len = 8;
		args.req_msgs_pos[i].id = (0x141 + i);
		for (int j = 1; j < 8; ++j)
		{
			args.req_msgs_pos[i].data[j] = 0;	
		}
		
		args.req_msgs_pos[i].data[0] = 0x92;
			
	}

	/* send some test msgs on the bus and check the bus state*/
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_send_msg(args.fd[i], &args.req_msgs_vel[i]);
		if (err)
		{
			
			if(err == -EWOULDBLOCK)
			{
				rt_printf("[MOTOR-RT-TASK] : Main : Failed to send test msg on CANBUS, exiting\n", err);
				return 0;		
			}
			else
			{
				return 0;
			}

			
		
		}
		
	}
	/*wait*/
	usleep(1000);
	/* check canbus state*/
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_get_state(args.fd[i], &state[i]);
		if(err)
		{
			rt_printf("[MOTOR-RT-TASK] :Failed to get status, invalid fd. Exiting\n");
			return 0;
		}
		if (state[i].tx_error_counter)
		{
			rt_printf("[MOTOR-RT-TASK] : Tx errors detected on CANBUS port %d. Check BUS connection\n", i);
			return 0;

		}
		
	}


	/* open msg queus*/
	struct mq_attr data_q_attr;
	data_q_attr.mq_flags = 0;
	data_q_attr.mq_maxmsg = 100;
	data_q_attr.mq_msgsize = sizeof(spi_data_t);
	args.msg_q_data = mq_open(devname_data, O_WRONLY | O_NONBLOCK | O_CREAT, 0666, &data_q_attr);
  	if(args.msg_q_data < 0)
  	{
   		rt_printf("[MOTOR-RT-TASK : Main : Failed to open data msg q\n");
   		return 0;
  	}

    struct mq_attr msg_q_attr;
	msg_q_attr.mq_flags = 0;
	msg_q_attr.mq_maxmsg = 100;
	msg_q_attr.mq_msgsize = sizeof(spi_command_t);
	args.msg_q_cmd = mq_open(devname_cmd, O_RDONLY| O_NONBLOCK | O_CREAT, 0666, &msg_q_attr);
  	if(args.msg_q_cmd < 0)
  	{
   		rt_printf("[MOTOR-RT-TASK : Main : Failed to open data msg q\n");
   		return 0;
  	}

   	struct mq_attr status_q_attr;
	status_q_attr.mq_flags = 0;
	status_q_attr.mq_maxmsg = 100;
	status_q_attr.mq_msgsize = sizeof(int);
	mqd_t msg_q_status;
	args.msg_q_status = mq_open(devname_status, O_RDONLY | O_NONBLOCK | O_CREAT, 0666, &status_q_attr);
  	if(args.msg_q_status < 0)
  	{
   		rt_printf("[MOTOR-RT-TASK : Main : Failed to open data msg q\n");
   		return 0;
  	}

   	/* flush the open qs*/
   	uint prio;
   	spi_command_t data_temp;
   	int status_temp;
   	while(1)
   	{
   		err = mq_receive(args.msg_q_cmd, (char *) &data_temp, sizeof(spi_command_t), &prio);
   		if (err < 0)
   		{
   			if(errno == EAGAIN)
   			{	
   				rt_printf("[MOTOR-RT-TASK]  Main :No msg available in cmd msg q, flushed successfully\n");
   				break;
   			}
   			else
   			{
   				rt_printf("[MOTOR-RT-TASK]  Main ::Failed to flush cmd msg q, exiting\n");
   				return 0;
   			}
   		}
   		else
   		{
   			rt_printf("[MOTOR-RT-TASK]: Main : Msg flushed from cmd msg q %d\n");
   		}

   	}
   	rt_printf("[MOTOR-RT-TASK] : Main : Flushed cmd msg q\n");
   	while(1)
   	{
   		err = mq_receive(args.msg_q_status, (char *)(&status_temp), sizeof(int), &prio);
   		if (err < 0)
   		{
   			if(errno == EAGAIN)
   			{	
   				rt_printf("[MOTOR-RT-TASK]  Main :No msg available in status msg q, flushed successfully\n");
   				break;
   			}
   			else
   			{
   				rt_printf("[MOTOR-RT-TASK]  Main ::Failed to flush status msg q, exiting\n");
   				return 0;
   			}
   		}
   		else
   		{
   			rt_printf("[MOTOR-RT-TASK]: Main : Msg flushed from status msg q %d\n");
   		}
   		
   		

   	}
   	rt_printf("[MOTOR-RT-TASK : Main : Flushed status msg q\n");


	err = pthread_mutex_init(&mutex, NULL);
	if (err) 
	{
		rt_printf("[MOTOR-RT-TASK] : Main : Failed to init mutex lock %d\n", err);
		return 0;
	}

	/* set sched policy*/
	pid_t id = gettid();
	struct sched_param params;
	params.sched_priority = 99;
	err = sched_setscheduler(id, SCHED_FIFO, &params);
	if (err)
	{
    	rt_printf("[Init] sched_setscheduler failed.\n");
    	return 0;
	}


	/*for avoiding page faults*/	
	mlockall(MCL_CURRENT | MCL_FUTURE);
  
	err = pthread_create(&motor_thread, NULL, write_task_func, &args);
	if (err) 
	{
		rt_printf("[MOTOR-RT-TASK] : Main : Failed to start motor pthread, code %d\n", err);
		return 0;
	}


	/*blocking call*/
	rt_printf("Press any key + ENTER to exit\n");
	/*avoid signal hanlder, use getchar instead*/
	getchar();
	/* exit the infinite loops in the threads*/
	err = pthread_mutex_lock(&mutex);
	if(err)
	{
		if(errno == EPERM)
		{
			rt_printf("[MAIN-RT-TASK] : Invalid context, failed to lock mutex\n");
			return 0;
		}
		else if (errno == EINVAL)
		{
			rt_printf("[MAIN-RT-TASK] : Invalid mutex, failed to lock\n");
			return 0;
		}
		else
		{
			rt_printf("[MAIN-RT-TASK] : Failed to acquire mutex\n");
			return 0;
		}

	}
	else
	{
		rt_printf("[MAIN-RT-TASK] : Acquired mutex\n");
		isRunning = 0;
	
	}
	
	err = pthread_mutex_unlock(&mutex);
	if(err)
	{
		if(errno == EPERM)
		{
			rt_printf("[IMU-RT-TASK] : Invalid context, failed to unlock mutex\n");
			return 0;
		}
		else if (errno == EINVAL)
		{
			rt_printf("[IMU-RT-TASK] : Invalid mutex, failed to unlock\n");
			return 0;
		}
		else
		{
			rt_printf("[IMU-RT-TASK] : Failed to release mutex\n");
			return 0;
		}

	}
	/*wait for thread to cleanup*/
	pthread_join(motor_thread, NULL);
	/*close msg qs*/
	mq_close(args.msg_q_data);
	mq_close(args.msg_q_status);
	mq_close(args.msg_q_cmd);
	/*set all setpoints to zerro before exiting*/
	memset(&args.setpoints, 0 , sizeof(spi_command_t));
	/* do control computations*/
	control_comp(&args);
	/* pack cmd*/
	pack_torque_cmd(&args);
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			switch(j)
			{
				case 0:		/*knee*/
					err = pcanfd_send_msg(args.fd[i], &args.setpoint_msgs.knee_setpoints[i]);
					if (err)
					{
						rt_printf("DANGER: Send msg failed because of errno %d, \n", err);		
					
					}
					break;
				case 1:		/*hip*/
					err = pcanfd_send_msg(args.fd[i], &args.setpoint_msgs.hip_setpoints[i]);
					if (err)
					{
						rt_printf("DANGER: Send msg failed because of err=%d, \n", err);		
					
					}
					break;
				case 2:		/*abad*/
					err = pcanfd_send_msg(args.fd[i], &args.setpoint_msgs.abad_setpoints[i]);
					if (err)
					{
						rt_printf("DANGER: Send msg failed because of err=%d, \n", err);	
					
					}
					break;
			}
			

		}
		
	}

	/* switch off all motors*/
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_close(args.fd[i]);
		if (err != -1)
		{
			rt_printf("[MOTOR-RT-TASK] : Main :Pcanfd close failed with err %d\n",err);
		}
	}

	
	return 0;

}
