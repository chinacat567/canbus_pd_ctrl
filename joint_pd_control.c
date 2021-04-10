/*sumantra sharma 25th March 2021*/
/* 28th March - added mutex locks between LCM and RT thread*/
/* 29th March - mutex sharing between RT and non RT not possible. Use msg pipes instead*/
/* 8th April - moved to pthreads (posix skin) from alchemy */
#include "joint_pd_control.h"

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
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

float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{

	float qFloat = fixedPointValue;
	qFloat *= pow(2.0, qPoint * -1);
	return (qFloat);
}

void print_spi_command(spi_command_t* msg)
{
	for (int i = 0; i < 1; ++i)
	{
		rt_printf("\n");
		// rt_printf("Leg no. %d :\n", i);
		// rt_printf("Knee setpoints:\n");
		rt_printf("q_des_knee[%d] = %f\n",i, msg->q_des_knee[i]);
		// rt_printf("qd_des_knee[%d] = %f\n",i, msg->qd_des_knee[i]);
		rt_printf("kp_knee[%d] = %f\n",i, msg->kp_knee[i]);
		rt_printf("kd_knee[%d] = %f\n",i, msg->kd_knee[i]);
		// rt_printf("tau_ff_knee[%d] = %f\n",i, msg->tau_knee_ff[i]);

		// rt_printf("Hip setpoints:\n");
		rt_printf("q_des_hip[%d] = %f\n",i,  msg->q_des_hip[i]);
		// rt_printf("qd_des_hip[%d] = %f\n",i,  msg->qd_des_hip[i]);
		rt_printf("kp_hip[%d] = %f\n",i,  msg->kp_hip[i]);
		rt_printf("kd_hip[%d] = %f\n",i,  msg->kd_hip[i]);
		// rt_printf("tau_ff_hip[%d] = %f\n",i,  msg->tau_hip_ff[i]);

		// // rt_printf("Abad setpoints:\n");
		// rt_printf("q_des_abad[%d] = %f\n",i, msg->q_des_abad[i]);
		// rt_printf("qd_des_abad[%d] = %f\n",i, msg->qd_des_abad[i]);
		// rt_printf("kp_abad[%d] = %f\n",i, msg->kp_abad[i]);
		// rt_printf("kd_abad[%d] = %f\n",i, msg->kd_abad[i]);
		// rt_printf("tau_ff_abad[%d] = %f\n",i, msg->tau_abad_ff[i]);



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
		// rt_printf("qd_knee[%d] = %f\n",i, msg->qd_knee[i]);
		
		// rt_printf("Hip data:\n");
		rt_printf("q_hip[%d] = %f\n",i, msg->q_hip[i]);
		// rt_printf("qd_hip[%d] = %f\n",i, msg->qd_hip[i]);
	
		// rt_printf("Abad data:\n");
		// rt_printf("q_abad[%d] = %f\n",i, msg->q_abad[i]);
		// rt_printf("qd_abad[%d] = %f\n",i, msg->qd_abad[i]);
		
	}
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

void print_torq_data(struct motor_torq_cmd* msg)
{
	for (int i = 0; i < 1; ++i)
	{
		rt_printf("\n");
		rt_printf("torq_knee[%d] = %f\n",i, msg->torq_setpoint_knee[i]);
		// rt_printf("torq_hip[%d] = %f\n",i, msg->torq_setpoint_hip[i]);		
		// rt_printf("torq_abad[%d] = %f\n",i, msg->torq_setpoint_abad[i]);
// 
		
	}
}


void control_comp(struct rt_task_args *args)
{
	int err;

	
		/* tau = tau_ff + kp * (theta-theta_r) + kd * (v-v_r)*/
		for (int i = 0; i < 4; ++i)
		{
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

			args->motor_torq_setpoints.torq_setpoint_knee[i] = \
			args->setpoints.tau_knee_ff[i]	+ (args->setpoints.kp_knee[i]) * (args->setpoints.q_des_knee[i] - args->leg_data.q_knee[i]) + \
			(args->setpoints.kd_knee[i]) * (args->setpoints.qd_des_knee[i] - args->leg_data.qd_knee[i]);

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

			args->motor_torq_setpoints.torq_setpoint_hip[i] = \
			args->setpoints.tau_hip_ff[i]	+ (args->setpoints.kp_hip[i]) * (args->setpoints.q_des_hip[i] - args->leg_data.q_hip[i]) + \
			(args->setpoints.kd_hip[i]) * (args->setpoints.qd_des_hip[i] - args->leg_data.qd_hip[i]);

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
		/*even odd for normalizing the zero pos*/

	
		if(rx_msgs[i].data[0] == 0x9c)
		{
			pspeed = (rx_msgs[i].data[5]<<8)|(rx_msgs[i].data[4]);
			switch (rx_msgs[i].id){
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
		else if(rx_msgs[i].data[0] == 0x92)
		{
			pposition = ( rx_msgs[i].data[1] | rx_msgs[i].data[2] << 8| \
			rx_msgs[i].data[3] << 16 | rx_msgs[i].data[4] << 24 | \
			(uint64_t)rx_msgs[i].data[5] << 32 |(uint64_t)rx_msgs[i].data[6] << 40 | \
			(uint64_t)rx_msgs[i].data[7] << 48 );

			switch (rx_msgs[i].id){
				/*knee*/
				case 0x141:					
					leg_data->q_knee[leg_no] = offset_knee[leg_no] + (pposition*60*PI)/(2 * 16383*180);
					break;
				/*hip*/
				case 0x142:
					leg_data->q_hip[leg_no] = offset_hip[leg_no] + (pposition*60*PI)/(2 * 16383*180);

					break;
				/*abad*/
				case 0x143:	
					leg_data->q_abad[leg_no] = (pposition*60*PI)/(2 * 16383*180);
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
void* write_task_func_imu(void * arg)
{
	int err, id= 0, flag = 0;
	struct imu_rt_task_args* args;
	args = (struct imu_rt_task_args*)arg;
	int isRunning_local=1;

	struct period_info pinfo;
	periodic_task_init(&pinfo,TASK_PERIOD_IMU);
	
	while(isRunning_local)
	{
		
		/*send reqs for imu data*/
		/*139 - Quaternions, 142 - Accel, 145 - Gyro*/
		err = pcanfd_send_msg(args->fd, &args->get_quat);
		/* if txq is full then exit the for loop*/
		if (err)
			rt_printf("[IMU-RT-TASK] : DANGER : IMU Send msg failed because of error no. %d \n", err);
		err = pcanfd_send_msg(args->fd, &args->get_gyro);
		/* if txq is full then exit the for loop*/
		if (err)
			rt_printf("[IMU-RT-TASK] : DANGER : IMU Send msg failed because of error no. %d \n", err);
		err = pcanfd_send_msg(args->fd, &args->get_accel);
		/* if txq is full then exit the for loop*/
		if (err)
			rt_printf("[IMU-RT-TASK] : DANGER : IMU Send msg failed because of error no. %d \n", err);
		/*read responses*/
		int msg_count = 0;
		while(1)
		{
			err = pcanfd_recv_msg(args->fd, &args->res_msgs_imu[msg_count]);
			if(err < 0) /* no msgs left to read*/
			{
				break;
			}
			else 
			{
			// rt_printf("[IMU-RT-TASK] : DANGER : Recv msg failed in IMU thread because of error no. %d\n", err);	
			 	#ifdef DEBUG
				// print_message_single(imu_port, &args->res_msgs_imu[msg_count]);	
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
					// args->data.quat[0] = (args->res_msgs_imu[i].data[3] << 24| args->res_msgs_imu[i].data[2] << 16| args->res_msgs_imu[i].data[1] << 8| args->res_msgs_imu[i].data[0]);
					// args->data.quat[1] = (args->res_msgs_imu[i].data[7] << 24| args->res_msgs_imu[i].data[6] << 16| args->res_msgs_imu[i].data[5] << 8| args->res_msgs_imu[i].data[4]);
				case 141:
					args->data.quat[2] = bytes_to_float(&args->res_msgs_imu[i].data[0]);
					args->data.quat[3] = bytes_to_float(&args->res_msgs_imu[i].data[4]);
					// args->data.quat[2] = (args->res_msgs_imu[i].data[3] << 24| args->res_msgs_imu[i].data[2] << 16| args->res_msgs_imu[i].data[1] << 8| args->res_msgs_imu[i].data[0]);
					// args->data.quat[3] = (args->res_msgs_imu[i].data[7] << 24| args->res_msgs_imu[i].data[6] << 16| args->res_msgs_imu[i].data[5] << 8| args->res_msgs_imu[i].data[4]);
				case 146:
					args->data.gyro[0] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[0]), 9);
					// args->data.gyro[0] = (&args->res_msgs_imu[i].data[i].data[1] << 8| &args->res_msgs_imu[i].data[i].data[0]);
					// args->data.gyro[0] = qToFloat(args->data.gyro[0], 9);
					args->data.gyro[1] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[2]), 9);
					// args->data.gyro[1] = (&args->res_msgs_imu[i].data[i].data[3] << 8| &args->res_msgs_imu[i].data[i].data[2]);
					// args->data.gyro[1] = qToFloat(args->data.gyro[1], 9 );
					args->data.gyro[2] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[4]), 9);
					// args->data.gyro[2] = (&args->res_msgs_imu[i].data[5] << 8| &args->res_msgs_imu[i].data[4]);
					// args->data.gyro[2] = qToFloat(args->data.gyro[2], 9);
				case 143:
					args->data.accel[0] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[0]), 8);
					// args->data.accel[0] = (&args->res_msgs_imu[i].data[1] << 8| &args->res_msgs_imu[i].data[0]);
					// args->data.accel[0] = qToFloat(args->data.accel[0], 8);
					args->data.accel[1] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[2]), 8);
					// args->data.accel[1] = (&args->res_msgs_imu[i].data[3] << 8| &args->res_msgs_imu[i].data[2]);
					// args->data.accel[1] = qToFloat(args->data.accel[1], 8);
					args->data.accel[1] = qToFloat(bytes_to_uint16(&args->res_msgs_imu[i].data[4]), 8);
					// args->data.accel[2] = (args->res_msgs_imu[i].data[5] << 8| args->res_msgs_imu[i].data[4]);
					// args->data.accel[2] = qToFloat(args->data.accel[2], 8);
			}


		}
		
		/* acquire mutex*/
		err = pthread_mutex_trylock(&mutex);
		if(err)
		{
			if(errno == EPERM)
				rt_printf("[IMU-RT-TASK] : Invalid context, failed to lock mutex\n");
			else if (errno == EINVAL)
				rt_printf("[IMU-RT-TASK] : Invalid mutex, failed to lock\n");
			else
				rt_printf("[IMU-RT-TASK] : Failed to acquire mutex\n");

		}
		else
		{
			flag = status;
			isRunning_local = isRunning;

			/*release mutex*/
			err = pthread_mutex_unlock(&mutex);
			if(err)
			{
				if(errno == EPERM)
					rt_printf("[IMU-RT-TASK] : Invalid context, failed to unlock mutex\n");
				else if (errno == EINVAL)
					rt_printf("[IMU-RT-TASK] : Invalid mutex, failed to unlock\n");
				else
					rt_printf("[IMU-RT-TASK] : Failed to release mutex\n");

			}
		}
		
		


		
		// if(err)
		// {
		// 	#ifdef DEBUG
		// 	rt_printf("[IMU-RT-TASK] : Failed to release mutex lock\n");
		// 	#endif 

		// }		
		if (flag)
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
					rt_printf("[IMU-RT-TASK] : Invalid fd for imu msg q\n");
				else if(errno == EMSGSIZE)
					rt_printf("[IMU-RT-TASK] : Invalid msg size for imu msg q\n");
			}
		  
		}

		else
		{
			// rt_printf("[IMU-RT-TASK] : IMU TASK : Waiting for higher controller\n");
		}

		wait_rest_of_period(&pinfo);	
		
	}
	rt_printf("[IMU-RT-TASK] : Exiting pthread\n");
}

void* write_task_func(void * arg)
{
	int err = 0,count = 0, flag=0;
	struct rt_task_args* args;
	args = (struct rt_task_args*)arg;
	int prio = 1;
	struct period_info pinfo;
	periodic_task_init(&pinfo,TASK_PERIOD_MOTOR);
	int isRunning_local=1;
	spi_command_t data;
	// clock_gettime(CLOCK_MONOTONIC, &start);

	/*enter infinite loop*/
	while(isRunning_local)
	{
		/* wait for task release*/
		// err = rt_task_wait_period(NULL);
		// err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &period, NULL);
		// if(err)
		// 	rt_printf("Sleed failed\n");
		// if(err == -EINTR)
		// 	rt_printf("RT task has been deleted already, errno =%d\n", err);		
		// else if(err == -ETIMEDOUT)
		// 	rt_printf("RT TASK (write_task_func) : timer overrun, increase period\n");
		// else if (err < 0)
		// 	rt_printf("DANGER :RT TASK MOTORS wait failed\n");

		/* read setpoints from mss q*/
		// err = mq_receive(args->msg_q_cmd, (char *) &data, sizeof(spi_command_t), &prio);
		err = mq_receive(args->msg_q_cmd, (char *) &args->setpoints, sizeof(spi_command_t), &prio);
		memset(&args->setpoints, 0 , sizeof(spi_command_t));
		if(err < 0)
		{
			if(errno == EAGAIN)
			{
				#ifdef DEBUG
				// rt_printf("[MOTOR-RT-TASK] :No msg available in cmd msg q, skipping\n");
				#endif 	
			}
			else if(errno == EBADF)
				rt_printf("[MOTOR-RT-TASK] :Invalid fd for cmd msg q\n");
			else if(errno == EMSGSIZE)
				rt_printf("[MOTOR-RT-TASK] :Invalid msg size for cmd msg q\n");
		}
		else if (err == sizeof(spi_command_t))
		{
			#ifdef DEBUG
			// rt_printf("[MOTOR-RT-TASK] : Read setpoints from higher controller\n");	
			#endif	
			// isRunning = 0;
			// return;
			// memcpy(&args->setpoints, &data,sizeof(spi_command_t) );

		}
		else
			rt_printf("[MOTOR-RT-TASK]: DANGER : Read incomplete setpoints from higher controller %d\n");

		// /*read status msg*/
		// /*lock mutex*/
		// err = rt_mutex_acquire(&mutex, TM_NONBLOCK);
		// if(err == -EWOULDBLOCK)
		// {
		// 	#ifdef DEBUG
		// 	rt_printf("[MOTOR-RT-TASK] :write task : unable to acquire mutex\n");
		// 	#endif 

		// }
		// else if(err < 0 && err != -EWOULDBLOCK)
		// {
		// 	rt_printf("[MOTOR-RT-TASK] :write task : invalid mutex descriptor   =%d\n, ", err);		
		// 	// isRunning = 0;
		// 	return;
		// }

		err = pthread_mutex_trylock(&mutex);
		if(err)
		{
			if(errno == EPERM)
				rt_printf("[MOTOR-RT-TASK] : Invalid context, failed to lock mutex\n");
			else if (errno == EINVAL)
				rt_printf("[MOTOR-RT-TASK] : Invalid mutex, failed to lock\n");
			else
				rt_printf("[MOTOR-RT-TASK] : Failed to acquire mutex\n");

		}
		else
		{
			// err = mq_receive(args->msg_q_status, (char *)(&flag), sizeof(int), &prio);
			err = mq_receive(args->msg_q_status, (char *)(&status), sizeof(int), &prio);
			if(err < 0)
			{
				if(errno == EAGAIN)
				{
					#ifdef DEBUG
					// rt_printf("[MOTOR-RT-TASK] :No msg available in status msg q, skipping\n");
					#endif 	
				}
				else if(errno == EBADF)
					rt_printf("[MOTOR-RT-TASK] :Invalid fd for status msg q\n");
				else if(errno == EMSGSIZE)
					rt_printf("[MOTOR-RT-TASK] :Invalid msg size for status msg q\n");
			}
			else if (err == sizeof(int))
			{
				#ifdef DEBUG
				// rt_printf("[MOTOR-RT-TASK] : Read status msg from higher controller\n");	
				#endif	
				// isRunning = 0;
				// return;
				// memcpy(&status, &flag, sizeof(int));


			}
			else
				rt_printf("[MOTOR-RT-TASK]: DANGER : Read incomplete status from higher controller %d\n");
			isRunning_local = isRunning;
			err = pthread_mutex_unlock(&mutex);
			if(err)
			{
				if(errno == EPERM)
					rt_printf("[MOTOR-RT-TASK] : Invalid context, failed to unlock mutex\n");
				else if (errno == EINVAL)
					rt_printf("[MOTOR-RT-TASK] : Invalid mutex, failed to unlock\n");
				else
					rt_printf("[MOTOR-RT-TASK] : Failed to release mutex\n");

			}


		}

		
		// /*unlock mutex*/
		// err = rt_mutex_release(&mutex);
		// if(err)
		// {
		// 	#ifdef DEBUG
		// 	rt_printf("[MOTOR-RT-TASK] :Failed to release mutex lock\n");
		// 	#endif 

		// }
		
			
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
				/* if txq is full then exit the for loop*/
				if (err)
				{
					rt_printf("[MOTOR-RT-TASK] :DANGER : Send msg failed because of error no. %d on port %d\n", err, i);
					// isRunning = 0;
					// return;		
	
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
				/* if txq is full then exit the for loop*/
				if (err)
				{
					rt_printf("[MOTOR-RT-TASK] : DANGER : Send msg failed because of errorr no. %d on port %d\n", err ,i);				
					// isRunning = 0;
					// return;		
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
					if(err < 0)
					{
						break;
					}
					else
					{
					 	#ifdef DEBUG
						// print_message_single(ports[i], &args->res_msgs[msg_count]);	
						#endif
						msg_count++;
					} 	

				}

				if(err != -EWOULDBLOCK)
				{
					rt_printf("[MOTOR-RT-TASK] :DANGER : Recv msg failed because of error no. %d\n", err);	
					// isRunning = 0;
					// return;
				}

			unpack_reply(args->res_msgs, msg_count, &args->leg_data, i);

		}
	
		/* do control computations*/
		control_comp(args);
		/* stuff data to can msg*/
		pack_torque_cmd(args);
		#ifdef DEBUG
		print_spi_data(&args->leg_data);
		// print_torq_data(&args->motor_torq_setpoints);
		#endif 
		/* publish on msg q before sending over CANBUS*/
		if (status)
		{
			err = mq_send(args->msg_q_data, (char *) (&args->leg_data), sizeof(spi_data_t), 1);
			if(err < 0)
			{
				if(errno == EAGAIN)
				{
					#ifdef DEBUG
					// rt_printf("[MOTOR-RT-TASK] :Data msg q full, skipping\n");
					#endif 	
				}
				else if(errno == EBADF)
					rt_printf("[MOTOR-RT-TASK] :Invalid fd for data msg q\n");
				else if(errno == EMSGSIZE)
					rt_printf("[MOTOR-RT-TASK] :Invalid msg size for data msg q\n");
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
			// rt_printf("[MOTOR-RT-TASK] :Motors task : Waiting for higher controller\n ");
		}
		
		
		/* send msgs*/
		#ifdef CAN_WRITE
		for (int i = 0; i < 1; ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				switch(j)
				{
					case 0:		/*knee*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.knee_setpoints[i]);
						if (err)
						{
							rt_printf("[MOTOR-RT-TASK] :DANGER: Send msg failed because of errno %d, \n", err);		
							// isRunning = 0;		
							// return;
						}
						break;
					case 1:		/*hip*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.hip_setpoints[i]);
						if (err)
						{
							rt_printf("[MOTOR-RT-TASK] :DANGER: Send msg failed because of err=%d, \n", err);		
							// isRunning = 0;		
							// return;
						}
						break;
					case 2:		/*abad*/
						err = pcanfd_send_msg(args->fd[i], &args->setpoint_msgs.abad_setpoints[i]);
						if (err)
						{
							rt_printf("[MOTOR-RT-TASK] :DANGER: Send msg failed because of err=%d, \n", err);	
							// isRunning = 0;
							// return;
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
	
	/*timeout for lcm select */
	int err;
	/* struct for passing */
	/* for isolating CPUs*/
	cpu_set_t cpu_motors, cpu_imu;
	CPU_ZERO(&cpu_motors);
	CPU_ZERO(&cpu_imu);
	CPU_SET(1,&cpu_motors);
	CPU_SET(2,&cpu_imu);
	/*pthread tasks*/
	pthread_t motor_thread, imu_thread;

	/* declare rt task arg structure*/
	// struct rt_task_args* args = malloc(sizeof(struct rt_task_args));
	// if(!args)
	// {
	// 	rt_printf("Joint PD controller (main): Malloc for args failed, exiting program\n");
	// 	return 0;
	// }
	struct rt_task_args args;
	/* set everything to 0*/
	memset(&args,0,sizeof(struct rt_task_args));

	// args->setpoints = (spi_command_t*)malloc(sizeof(spi_command_t));
	// if(!args->setpoints)
	// {
	// 	rt_printf("Joint PD controller (main):Malloc for args.setpoints failed, exiting program\n");
	// 	return 0;
	// }

	/* set initial setpoints */
	// memset(args.setpoints, 0 , sizeof(spi_command_t));
	// args.setpoints->q_des_hip[0] = -0.78;
	// args.setpoints->kp_hip[0] = 70;
	// args.setpoints->kd_hip[0] = 10;


	/* declare rt task arg structure for imu*/
	// struct imu_rt_task_args* args_imu = malloc(sizeof(struct imu_rt_task_args));
	// if(!args_imu)
	// {
	// 	rt_printf("Joint PD controller (main): Malloc for args failed, exiting program\n");
	// 	return 0;
	// }
	struct imu_rt_task_args args_imu;
	/* set everything to 0*/
	memset(&args_imu,0,sizeof(struct imu_rt_task_args));
	// struct pcanfd_msg_filter* msg_filter =  malloc(sizeof(struct pcanfd_msg_filter));
	// if(!msg_filter)
	// {
	// 	rt_printf("Joint PD controller (main):Malloc for msg_filter failed, exiting program\n");
	// 	return 0;
	// }
	struct pcanfd_msg_filter msg_filter;


	
	/* open can ports*/
	for (int i = 0; i < 4; ++i)
	{
		args.fd[i] = pcanfd_open(ports[i], OFD_BITRATE  | PCANFD_INIT_STD_MSG_ONLY | OFD_NONBLOCKING, 1000000);
		if (args.fd[i] < 0)
		{
			rt_printf("Joint PD controller (main):Pcanfd open failed with err %d on port no. %d\n",args.fd[i], i);
			return 0;
		}
	}

	/*open imu can port*/
	args_imu.fd = pcanfd_open(imu_port, OFD_BITRATE  | PCANFD_INIT_STD_MSG_ONLY | OFD_NONBLOCKING, 1000000);
	if (args_imu.fd < 0)
	{
		rt_printf("Joint PD controller (main):Pcanfd open failed with err %d on port no.\n",args_imu.fd);
		return 0;
	}

	/* set options to allow only STD-CAN msgs */
	uint32_t allowed_msg = PCANFD_ALLOWED_MSG_CAN;
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_set_option(args.fd[i], PCANFD_OPT_ALLOWED_MSGS, &allowed_msg, sizeof(allowed_msg));
		if (err) 
		{
			rt_printf("Joint PD Controller (main): Failed to set_option on fd %d with errno %d\n", args.fd[i], err);
			return 0;

		}
				
	}
	/*set filter on imu canbus*/
	err = pcanfd_set_option(args_imu.fd, PCANFD_OPT_ALLOWED_MSGS, &allowed_msg, sizeof(allowed_msg));
	if (err) 
	{
		rt_printf("Joint PD Controller (main): Failed to set_option on fd %d with errno %d\n", args_imu.fd, err);
		return 0;

	}

	/* add filters on CANID for safety, allow only CANIDS 141 - 143 */
	msg_filter.id_from = 0x141;
	msg_filter.id_to = 0x143;
	for (int i = 0; i < 4; ++i)
	{
		err = pcanfd_add_filter(args.fd[i], &msg_filter);
		if (err) 
		{
			rt_printf("Joint PD Controller (main): Failed to add_filter on fd %d with errno %d\n", args.fd[i], err);
			return 0;

		}
				
	}
	/* add filter for imu canbus*/
	msg_filter.id_from = 139;
	msg_filter.id_to = 151;
	err = pcanfd_add_filter(args_imu.fd, &msg_filter);
	if (err) 
	{
		rt_printf("Joint PD Controller (main): Failed to add_filter on fd %d with errno %d\n", args_imu.fd, err);
		return 0;

	
	
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
	

	// /*create real_time task*/
	// err = rt_task_create(&args.write_task, "write_task", 0, 99, 0);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to create rt task, code %d\n", err);
	// 	return err;
	// }
	// /* set affinity to isolate real-time task to only 1 cpus*/
	// err = rt_task_set_affinity(&args.write_task, &cpu_motors);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to create rt task, code %d\n", err);
	// 	return err;
	// }
	// /* set peridic task*/

	// err = rt_task_set_periodic(&args.write_task, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD_MOTOR));
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed set periodic task , code %d\n" ,err);
	// 	return err;
	// }

	// create real_time task for imu
	// err = rt_task_create(&args_imu.write_task, "write_task_imu", 0, 98, 0);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to create rt task, code %d\n", err);
	// 	return err;
	// }
	// /* set affinity to isolate real-time task to only 1 cpus*/
	// err = rt_task_set_affinity(&args_imu.write_task, &cpu_imu);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to create rt task, code %d\n", err);
	// 	return err;
	// }
	// /* set peridic task*/

	// err = rt_task_set_periodic(&args_imu.write_task, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD_IMU));
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed set periodic task, code %d\n" ,err);
	// 	return err;
	// }

	struct mq_attr data_q_attr;
	data_q_attr.mq_flags = 0;
	data_q_attr.mq_maxmsg = 100;
	data_q_attr.mq_msgsize = sizeof(spi_data_t);
	args.msg_q_data = mq_open(devname_data, O_WRONLY | O_NONBLOCK | O_CREAT, 0666, &data_q_attr);
  	if(args.msg_q_data < 0)
   		rt_printf("Failed to open data msg q\n");

    struct mq_attr msg_q_attr;
	msg_q_attr.mq_flags = 0;
	msg_q_attr.mq_maxmsg = 100;
	msg_q_attr.mq_msgsize = sizeof(spi_command_t);
	args.msg_q_cmd = mq_open(devname_cmd, O_RDONLY| O_NONBLOCK | O_CREAT, 0666, &msg_q_attr);
  	if(args.msg_q_cmd < 0)
   		rt_printf("Failed to open data msg q\n");

   	struct mq_attr imu_q_attr;
	imu_q_attr.mq_flags = 0;
	imu_q_attr.mq_maxmsg = 100;
	imu_q_attr.mq_msgsize = sizeof(struct imu_data);
	args_imu.msg_q_imu = mq_open(devname_imu, O_WRONLY | O_NONBLOCK | O_CREAT, 0666, &imu_q_attr);
  	if(args_imu.msg_q_imu < 0)
   		rt_printf("Failed to open data msg q\n");

   	struct mq_attr status_q_attr;
	status_q_attr.mq_flags = 0;
	status_q_attr.mq_maxmsg = 100;
	status_q_attr.mq_msgsize = sizeof(int);
	mqd_t msg_q_status;
	args.msg_q_status = mq_open(devname_status, O_RDONLY | O_NONBLOCK | O_CREAT, 0666, &status_q_attr);
  	if(args.msg_q_status < 0)
   		rt_printf("Failed to open data msg q\n");

   	/* flush the open q*/
   	uint prio;
   	spi_command_t data_temp;
   	int status_temp;
   	while(1)
   	{
   		err = mq_receive(args.msg_q_cmd, (char *) &data_temp, sizeof(spi_command_t), &prio);
   		if (err < 0)
   		{
   			break;
   		}
   		usleep(1000);

   	}
   	rt_printf("Flushed cmd msg q\n");
   	while(1)
   	{
   		err = mq_receive(args.msg_q_status, (char *)(&status_temp), sizeof(int), &prio);
   		if (err < 0)
   		{
   			break;
   		}
   		usleep(1000);

   	}
   	rt_printf("Flushed status msg q\n");


	// err = rt_queue_create(&args.msg_q_setpoints, devname_cmd, 100*sizeof(spi_command_t), Q_UNLIMITED, Q_FIFO);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to create msg spi_command msg q, code %d\n" ,err);
	// 	return err;
	// }
	
	// err = rt_queue_create(&args.msg_q_data, devname_data , 100*sizeof(spi_data_t), Q_UNLIMITED, Q_FIFO);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to create spi_data msg q, code %d\n" ,err);
	// 	return err;
	// }

	// err = rt_queue_create(&args_imu.msg_q_imu, devname_imu, 100*sizeof(struct imu_data), Q_UNLIMITED, Q_FIFO);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to create imu_data msg q, code %d\n" ,err);
	// 	return err;
	// }

	// err = rt_queue_create(&args.msg_q_status, devname_status, sizeof(int), Q_UNLIMITED, Q_FIFO);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to create status msg q, code %d\n" ,err);
	// 	return err;
	// }

	// args.data_msg_buf = rt_queue_alloc(&args.msg_q_data, sizeof(spi_data_t));
	// if(!args.data_msg_buf)
	// {
	// 	rt_printf("Failed to allocate buffer for data msg\n");
	// 	return 0;
	// }

	// args_imu.imu_data_msg_buf = rt_queue_alloc(&args_imu.msg_q_imu, sizeof(struct imu_data));
	// if(!args_imu.imu_data_msg_buf)
	// {
	// 	rt_printf("Failed to allocate buffer for imu msg\n");
	// 	return 0;
	// }
	
	// err = rt_task_start(&args.write_task, write_task_func, args);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to start rt task, code %d\n", err);
	// 	return err;
	// }

	// err = rt_task_start(&args_imu.write_task, write_task_func_imu, args_imu);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to start rt task, code %d\n", err);
	// 	return err;
	// }
	// /*create mutex lock*/
	// err = pthread_mutexattr_init(&attr);
	// // err = rt_mutex_create(&mutex, "mutex_lock");
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to set mutex lock attr %d\n", err);
	// 	return err;
	// }

	// err = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
	// // err = rt_mutex_create(&mutex, "mutex_lock");
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to set mutex lock type to non-block %d\n", err);
	// 	return err;
	// }

	err = pthread_mutex_init(&mutex, NULL);
	// err = rt_mutex_create(&mutex, "mutex_lock");
	if (err) {
		rt_printf("Joint PD Controller (main): Failed to init mutex lock %d\n", err);
		return err;
	}


	/* set sched policy*/
	pid_t id = gettid();
	struct sched_param params;
	params.sched_priority = 99;
	err = sched_setscheduler(id, SCHED_FIFO, &params);
	if (err)
    	rt_printf("[Init] sched_setscheduler failed.\n");


		/*for avoiding page faults*/	

	mlockall(MCL_CURRENT | MCL_FUTURE);
  
	// err = pthread_create(&motor_thread, NULL, write_task_func, &args);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to start motor pthread, code %d\n", err);
	// 	return err;
	// }

	err = pthread_create(&imu_thread, NULL, write_task_func_imu, &args_imu);
	if (err) {
		rt_printf("Joint PD Controller (main): Failed to start imu pthread, code %d\n", err);
		return err;
	}

	/*wait for threads to exit*/

	/*blocking call*/
	rt_printf("Press any key + ENTER to exit\n");
	getchar();
	/* exit the main loop*/
	err = pthread_mutex_lock(&mutex);
	if(err)
	{
		if(errno == EPERM)
			rt_printf("[MAIN-RT-TASK] : Invalid context, failed to lock mutex\n");
		else if (errno == EINVAL)
			rt_printf("[MAIN-RT-TASK] : Invalid mutex, failed to lock\n");
		else
			rt_printf("[MAIN-RT-TASK] : Failed to acquire mutex\n");

	}
	
	rt_printf("[MAIN-RT-TASK] : Acquired mutex\n");
	isRunning = 0;
	err = pthread_mutex_unlock(&mutex);
		if(err)
		{
			if(errno == EPERM)
				rt_printf("[IMU-RT-TASK] : Invalid context, failed to unlock mutex\n");
			else if (errno == EINVAL)
				rt_printf("[IMU-RT-TASK] : Invalid mutex, failed to unlock\n");
			else
				rt_printf("[IMU-RT-TASK] : Failed to release mutex\n");

		}
	pthread_join(imu_thread, NULL);
	pthread_join(motor_thread, NULL);

	// err = rt_task_delete(&args.write_task);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to delete rt task, code %d\n", err);		
	// 	return err;
	// }

	// err = rt_task_delete(&args_imu.write_task);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to delete imu rt task, code %d\n", err);		
	// 	return err;
	// }
	mq_close(args.msg_q_data);
	mq_close(args.msg_q_status);
	mq_close(args.msg_q_cmd);
	mq_close(args_imu.msg_q_imu);


	// err = rt_queue_delete(&args.msg_q_data);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to delete motor rt task msg_q_data, code %d\n", err);		
	// 	return err;
	// }

	// err = rt_queue_delete(&args_imu.msg_q_imu);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to delete rt task msg_q_imu, code %d\n", err);		
	// 	return err;
	// }
	// err = rt_queue_delete(&args.msg_q_setpoints);
	// if (err) {
	// 	rt_printf("Joint PD Controller (main): Failed to delete rt task msg_q_setpoints, code %d\n", err);		
	// 	return err;
	// }
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
						// isRunning = 0;		
						// return;
					}
					break;
				case 1:		/*hip*/
					err = pcanfd_send_msg(args.fd[i], &args.setpoint_msgs.hip_setpoints[i]);
					if (err)
					{
						rt_printf("DANGER: Send msg failed because of err=%d, \n", err);		
						// isRunning = 0;		
						// return;
					}
					break;
				case 2:		/*abad*/
					err = pcanfd_send_msg(args.fd[i], &args.setpoint_msgs.abad_setpoints[i]);
					if (err)
					{
						rt_printf("DANGER: Send msg failed because of err=%d, \n", err);	
						// isRunning = 0;
						// return;
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
			rt_printf("Joint PD Controller (main):Pcanfd close failed with err %d\n",err);
	}

	
	return 0;

}
