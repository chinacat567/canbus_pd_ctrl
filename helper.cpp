
#include"helper.hpp"
int helper(mqd_t msg_q_cmd)
{
	      /* send cmd data*/
	int err;
	spi_command_t cmd;
      err = mq_send(msg_q_cmd, (char*) &cmd, sizeof(spi_command_t), 1);
      if(err < 0)
      {   
        if (errno == EAGAIN)
        {
         #ifdef DEBUG  
           printf("Cmd msg q full, skiping send\n");
         #endif 
        }
       
        else if (errno == EBADF)
          printf("Failed to write to cmd msg q because of bad fd\n");
        else if (errno = EMSGSIZE)
          printf("Failed to write to cmd msg q because of ivalid msgsize  \n");

      }
}