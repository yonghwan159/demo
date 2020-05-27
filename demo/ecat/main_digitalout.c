/*
 *  This file is owned by the Embedded Systems Laboratory of Seoul National University of Science and Technology
 *  to test EtherCAT master protocol using the IgH EtherCAT master userspace library.	
 *  
 *  
 *  IgH EtherCAT master library for Linux is found at the following URL: 
 *  <http://www.etherlab.org/en/ethercat>
 *
 *
 *
 *
 *  2015 Raimarius Delgado
*/
/****************************************************************************/
#include "main_digitalout.h"
/****************************************************************************/
/* Private Functions */
void XenoInit(void);
void XenoQuit(void);
void DoInput();
void SignalHandler(int signum);

/****************************************************************************/
void EcatCtrlTask(void *arg){
	 	
#ifdef MEASURE_TIMING
	RtmEcatPeriodEnd = rt_timer_read();
#endif
   
   while (1) {

	   rt_task_wait_period(NULL);
#ifdef MEASURE_TIMING
	   RtmEcatPeriodStart = rt_timer_read();
#endif
    	/* Receive Process Data */
	   EcatReceiveProcessDomain();

    	/* check process data state (optional) */
	   EcatStat = EcatStatusCheck(); 

	/*Do reading of the current process data from here
	 * before processing 
	 */ 

	
	   

	/* write application time to master */
   	   RtmEcatMasterAppTime = rt_timer_read();
   	   EcatWriteAppTimeToMaster((uint64_t)RtmEcatMasterAppTime);

	if(EcatStat.master_state == OP && EcatStat.slave_state == OP)
	{
		/* Input Command */
		DoInput(); 
		}



	/* send process data */
	   EcatSendProcessDomain();

#if 0 //  0 to omit : 1 for processing

	   if (!(iTaskTick % FREQ_PERSEC(ECATCTRL_TASK_PERIOD))){
		/*Do every 1 second */

		    rt_printf("Master State:%u Slave State:%u Slave Number:%u \n",
				    EcatStat.master_state, 
				    EcatStat.slave_state,
				    EcatStat.slave_number); 
		   
		   // rt_printf("Slave0:%s Slave1:%s Slave2:%s Slave3:%s \n",
				 //   lsmecaGetStatusStr(lsmecaGetStatusWordN(0)),
				 //   lsmecaGetStatusStr(lsmecaGetStatusWordN(1)),
				 //   lsmecaGetStatusStr(lsmecaGetStatusWordN(2)),
				 //   lsmecaGetStatusStr(lsmecaGetStatusWordN(3))); 		   
	   	// rt_printf("Slave0:0x%x Slave1:0x%x Slave2:0x%x Slave3:0x%x \n",
				 //   lsmecaGetStatusWordN(0),
				 //   lsmecaGetStatusWordN(1),
				 //   lsmecaGetStatusWordN(2),
				 //   lsmecaGetStatusWordN(3)); 		   
	   	// rt_printf("Slave0:%d Slave1:%d Slave2:%d Slave3:%d \n",
				 //   lsmecaGetActualVelocityN(0),
				 //   lsmecaGetActualVelocityN(1),
				 //   lsmecaGetActualVelocityN(2),
				 //   lsmecaGetActualVelocityN(3)); 		 

		// rt_printf("Slave0:%d Slave1:%d Slave2:%d Slave3:%d \n",
		// 		   lsmecaGetActualPosition(0),
		// 		   lsmecaGetActualPosition(1),
		// 		   lsmecaGetActualPosition(2),
		// 		   lsmecaGetActualPosition(3));   


	   }
#endif
		
#ifdef MEASURE_TIMING	    
	   RtmEcatExecTime = rt_timer_read();
   	   RtmEcatPeriodEnd = RtmEcatPeriodStart;
#endif
    }

}
/****************************************************************************/
void InptCtrlTask(void *arg){

	while (1) {
	
		rt_task_wait_period(NULL);
		
		InptChar = getche(); /* libs/embedded/embdCONIO.h */
   	}
}

/****************************************************************************/

int main(int argc, char **argv){
   	 
	int ret = 0;
	
	/* Interrupt Handler "ctrl+c"  */
	signal(SIGTERM, SignalHandler);
    signal(SIGINT, SignalHandler);	 

	/* EtherCAT Init */
   	if ((ret = EcatInit(ECATCTRL_TASK_PERIOD,0x09))!=0){
		fprintf(stderr, 
			"Failed to Initiate IgH EtherCAT Master!\n");
	}
	
	mlockall(MCL_CURRENT|MCL_FUTURE); //Lock Memory to Avoid Memory Swapping

	/* RT-task */

	XenoInit();
	
	while (1) {
		usleep(1);
		if (quitFlag) break;
	}

	XenoQuit();
	EcatQuit();
return ret;
}
/****************************************************************************/

/****************************************************************************/

void SignalHandler(int signum){
		quitFlag=1;
}

/****************************************************************************/

void XenoInit(void){

	rt_print_auto_init(1); //RTDK

	printf("Creating Xenomai Realtime Task(s)...");
	rt_task_create(&TskEcatCtrl,"EtherCAT Control", 0, ECATCTRL_TASK_PRIORITY,ECATCTRL_TASK_MODE);
	rt_task_create(&TskInptCtrl,"Input Control", 0, INPTCTRL_TASK_PRIORITY,ECATCTRL_TASK_MODE);
	printf("OK!\n");


	printf("Making Realtime Task(s) Periodic...");
	rt_task_set_periodic(&TskEcatCtrl, TM_NOW,rt_timer_ns2ticks(ECATCTRL_TASK_PERIOD));	
	rt_task_set_periodic(&TskInptCtrl, TM_NOW,rt_timer_ns2ticks(INPTCTRL_TASK_PERIOD));
	printf("OK!\n");

	printf("Starting Xenomai Realtime Task(s)...");
	rt_task_start(&TskEcatCtrl, &EcatCtrlTask, NULL);
	rt_task_start(&TskInptCtrl, &InptCtrlTask, NULL);
	printf("OK!\n");
	
}

/****************************************************************************/

void XenoQuit(void){
	
	rt_task_suspend(&TskInptCtrl);
	rt_task_suspend(&TskEcatCtrl);

	rt_task_delete(&TskInptCtrl);
	rt_task_delete(&TskEcatCtrl);
	printf("\033[%dm%s\033[0m\n",95,"\nXenomai Task(s) Deleted!");

}

/****************************************************************************/


void DoInput(){

	switch(InptChar)
	{
	case 'Q'	:
	case 'q'	:
		lsmecaSet(_ALLNODE,0x01);
		break;
	case 'w'	:
	case 'W'	:
		lsmecaSet(_ALLNODE,0x03);
		break;
	case 'e'	:
	case 'E'	:
		lsmecaSet(_ALLNODE,0x07);
		break;
	case 'R'	:
	case 'r'	:
		lsmecaSet(_ALLNODE,0x0f);
		break;
	default		:
		break;
	}
	InptChar = 0;
}
