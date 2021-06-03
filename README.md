# canbus_pd_ctrl
Embedded Controller for a 12 DoF quadruped robot. 

System details :
1. Advantech PCM 3365 PC104 with two PCAN-PC/104-Plus Quad CANBUS cards
2. OS - Xenomai 3.0 Cobalt co-kernel running alongside Linux on an Ubuntu 16.04 distribution
3. Actuators : Gyems RMD X8 Pro (CANBUS) (more about this on my github page here)
4. Bosch bno055 CANBUS IMU system developed by my labmate (credits Ashwin Narayan) 

In order to achieve deterministic real - time behavior, Xenomai 3.0 was used. Addtionally, PEAk systems provides RTDM drivers for the their CANBUS cards so that their hardware can be used with Xenomai. More on this here.

motors_can.c - main progam running the real - time PD control loop. It performs the following impedance control loop for each of the 12 motors at roughly 500 Hz
tau = Kp * (q - q_des) + Kd * (qd - qd_des) + tau_ff
The parameters Kp, Kd, q_des, qd_des, tau_ff are received from a different process (the high level controller) using Posix message queues.
During each thread cycle, the following operations take place :
1. A non-blocking read on the FD msg_q_cmd (posix q for receiving setpoints from the higher controller).
2. Write operation on the CANBUS for sending out data requests to all the 12 motors. The CANBUS protocol for the actuators is based on a client - server model and hence we need to request for fresh data at the start of each new loop.
3. Read the data received from the motors (non - blocking read, empty the receive q completely in each cycle)
4. Perform the control computations
5. Write the setpoints to all the 12 motors after applying safety limits (torque setpoints)
6. Publish the data over the FD msg_q_data (posix q for sending out the motor data to the higher controller)

I had also added a GPIO estop which would trigger the GPIO pins and trigger the relays in case the motors cross the safety limits. 

imu_can.c - perform a very similiar function but for the IMU