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
#include "main.h"
/****************************************************************************/
/* Private Functions */
void XenoInit(void);
void XenoQuit(void);
void DoInput();
void SignalHandler(int signum);
/****************************************************************************/
float ref=104.72;
float err_p[4]={0.,}; 
float err_i[4] = {0.,};
float err_d[4] = {0.,}; 
float Ki=0.001;
float Kd=0;
float Kp=0.001;
float PID_velocity[4]={0.,};
float prev_err_p[4]={0.,};
int Velocity[4]={500,1000,1500,2000};
int led[16]={0x0000,0x0001,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007,0x0008,0x0009,0x000a,0x000b,0x000c,0x000d,0x000e,0x000f};
int curVel[4]={0.,};
int settime=6000;


unsigned short previousStatus[LSMECAPION_SLAVENUM] = {0,};
void PrintEval(int BufPrd[], int BufExe[], int BufJit[], int ArraySize);

/****************************************************************************/
void EcatCtrlTask(void *arg){
	int iTaskTick = 0;
	int runFlag = 0;
	int iSlaveCnt = 0;
	int count=0;	
	
#ifdef MEASURE_TIMING
	RtmEcatPeriodEnd = rt_timer_read();
#endif
   while (1) {
   		
	   wait_rt_period(&TskEcatCtrl);
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

	/* Input Command */
	   DoInput(); 

	if(EcatStat.master_state == OP && EcatStat.slave_state == OP)
	{
		timingFlag=1;
		// for (iSlaveCnt = 0; iSlaveCnt < LS_SLAVENUM; iSlaveCnt++)
		// {
		// 	if (lsmecaGetStatusWordN(iSlaveCnt) == previousStatus[iSlaveCnt])
		// 		continue;

		// 	if ((lsmecaGetStatusWordN(iSlaveCnt)) == LSMECA_SWITCH_ON_ENABLED)
		// 	{
		// 		lsmecaServoOn(iSlaveCnt);
		// 	}
		// 	else if ((lsmecaGetStatusWordN(iSlaveCnt)) == LSMECA_READY_TO_SWITCH_ON)
		// 	{
		// 		lsmecaReady(iSlaveCnt);
		// 	}

		// 	else if ((lsmecaGetStatusWordN(iSlaveCnt)) == LSMECA_OPERATION_ENABLED)
		// 	{
		// 		runFlag = 1;
		// 		timingFlag = 1;
		// 	}
		// 	else
		// 		lsmecaReady(iSlaveCnt);

		// 	previousStatus[iSlaveCnt] = lsmecaGetStatusWordN(iSlaveCnt);
		// }
	
		// if (runFlag)
		// {	
			/*synchronization*/
			if (count){
				count=0;
			}
			else {
				count=0xff;
			}
			lsmecaSet(0,count);
			lsmecaSet(1,count);	
			lsmecaSet1(0,count);

			// if (!(iTaskTick%(settime*SECNUM)))
			// {	
			// 	quitFlag = 1;
			// 	break;
			// }
			// for (iSlaveCnt = 0; iSlaveCnt < LS_SLAVENUM; iSlaveCnt++)
			// {
			// 	lsmecaSetVelocity(iSlaveCnt,100);
			// }
			/*count demo*/
			// if (!(iTaskTick % FREQ_PERSEC(ECATCTRL_TASK_PERIOD))){
				
			// 	lsmecaSet(0,led[count]);	
			// 	for (iSlaveCnt = 0; iSlaveCnt <LS_SLAVENUM; iSlaveCnt++)
			// 	{	
			// 		if((count>>iSlaveCnt)%2)
			// 			curVel[iSlaveCnt] = Velocity[iSlaveCnt];
			// 		else	
			// 			curVel[iSlaveCnt] = 0;
			// 	}
				
			// 	if(count>16)
			// 		count=0;

			// 	count++;
				
			// 	}
			// for (iSlaveCnt = 0; iSlaveCnt < LS_SLAVENUM; iSlaveCnt++)
			// {
				
			// 	if (iSlaveCnt != 1)
			// 	lsmecaSetVelocity(iSlaveCnt,curVel[iSlaveCnt]);
			// 	else
			// 	lsmecaSetVelocity(iSlaveCnt,0);
			// }
			
		// }
	// iTaskTick++;
	}
	/* write application time to master */
   	   RtmEcatMasterAppTime = rt_timer_read();
   	   EcatWriteAppTimeToMaster((uint64_t)RtmEcatMasterAppTime);

	/* send process data */
	   EcatSendProcessDomain();

#ifdef MEASURE_TIMING
		RtmEcatExecTime = rt_timer_read();
		EcatPeriod 	 = (RtmEcatPeriodStart - RtmEcatPeriodEnd); RtmEcatPeriodEnd = RtmEcatPeriodStart;
		EcatExecution = (RtmEcatExecTime - RtmEcatPeriodStart);
		EcatJitter	 = MathAbsValI(EcatPeriod - ECATCTRL_TASK_PERIOD);


		if(timingFlag == 1) {
			BufEcatPeriodTime[iBufEcatDataCnt] 	=   EcatPeriod;
			BufEcatExecTime[iBufEcatDataCnt]  	=   EcatExecution;
			BufEcatJitter[iBufEcatDataCnt]  	=   EcatJitter;

			if (BUF_SIZE == iBufEcatDataCnt)
				quitFlag=1;

			++iBufEcatDataCnt;
		}
#endif

	   

    }
    delete_rt_task();
}

/****************************************************************************/
void InptCtrlTask(void *arg){

	while (1) {
	
		wait_rt_period(&TskInptCtrl);
		
		InptChar = getche(); /* libs/embedded/embdCONIO.h */
   	}
   	delete_rt_task();
}

/****************************************************************************/

int main(int argc, char **argv){
   	 
	int ret = 0;
	

	/* Interrupt Handler "ctrl+c"  */
	signal(SIGTERM, SignalHandler);
    signal(SIGINT, SignalHandler);	 

	/* EtherCAT Init */
   	if ((ret = EcatInit(ECATCTRL_TASK_PERIOD,LSMECA_CYCLIC_VELOCITY))!=0){
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


	printf("%d %d\n",iBufEcatDataCnt, BUF_SIZE);
	PrintEval(BufEcatPeriodTime,BufEcatExecTime,BufEcatJitter,iBufEcatDataCnt);
	// XenoQuit();
	// EcatQuit();
return ret;
}
/****************************************************************************/

void DoInput(){

	switch(InptChar)
	{
	case 'Q'	:
	case 'q'	:
		lsmecaShutDown(_ALLNODE);
		quitFlag = 1;
		break;
	case 'd'	:
	case 'D'	:
		lsmecaSetVelocity(_ALLNODE,10000);
		break;
	case 'e'	:
	case 'E'	:
		lsmecaSetVelocity(_ALLNODE,-300);
		break;
	case 's'	:
	case 'S'	:
		lsmecaSetAngularVelocity(_ALLNODE,3.4*10);
		break;
	case 'r'	:
	case 'R'	:
		lsmecaReady(_ALLNODE);
		break;
	case 'O'	:
	case 'o'	:
		lsmecaServoOn(_ALLNODE);
		break;
	case 'x'	:
	case 'X'	:
		lsmecaSetVelocity(_ALLNODE,0);
		lsmecaServoOff(_ALLNODE);
		break;
	default		:
		break;
	}
	InptChar = 0;
}

/****************************************************************************/

void SignalHandler(int signum){
		quitFlag=1;
}

/****************************************************************************/

void XenoInit(void){

	// rt_print_auto_init(1); //RTDK

	printf("Creating  Realtime Task(s)...");
	create_rt_task(&TskEcatCtrl,"EtherCAT Control", ECATCTRL_TASK_PRIORITY);
	create_rt_task(&TskInptCtrl,"Input Control", INPTCTRL_TASK_PRIORITY);
	printf("OK!\n");


	printf("Making Realtime Task(s) Periodic...");
	set_rt_task_period(&TskEcatCtrl,ECATCTRL_TASK_PERIOD);
	set_rt_task_period(&TskInptCtrl,INPTCTRL_TASK_PERIOD);
	printf("OK!\n");

	printf("Starting Xenomai Realtime Task(s)...");
	start_rt_task(1,&TskEcatCtrl,&EcatCtrlTask);
	start_rt_task(1,&TskInptCtrl,&InptCtrlTask);
	printf("OK!\n");
	
}

/****************************************************************************/

void XenoQuit(void){
	
	// rt_task_suspend(&TskInptCtrl);
	// rt_task_suspend(&TskEcatCtrl);

	// rt_task_delete(&TskInptCtrl);
	// rt_task_delete(&TskEcatCtrl);
	printf("\033[%dm%s\033[0m\n",95,"\nXenomai Task(s) Deleted!");

}

/****************************************************************************/


void PrintEval(int BufPrd[], int BufExe[], int BufJit[], int ArraySize){

/* MATH_STATS: Simple Statistical Analysis (libs/embedded/embdMATH.h)
 * 	float ave;
 * 	float max;
 * 	float min;
 * 	float std; */
	MATH_STATS EcatPeriodStat, EcatExecStat, EcatJitterStat;

	EcatPeriodStat = GetStatistics(BufPrd, ArraySize,SCALE_1M);  
	printf("\n[Period] Max: %.6f Min: %.6f Ave: %.6f St. D: %.6f\n", EcatPeriodStat.max,
			EcatPeriodStat.min,
			EcatPeriodStat.ave,
			EcatPeriodStat.std);
	EcatExecStat = GetStatistics(BufExe, ArraySize,SCALE_1K);  
	printf("[Exec]	 Max: %.3f Min: %.3f Ave: %.3f St. D: %.3f\n", EcatExecStat.max,
			EcatExecStat.min,
			EcatExecStat.ave,
			EcatExecStat.std);
	EcatJitterStat = GetStatistics(BufJit, ArraySize,SCALE_1K);  
	printf("[Jitter] Max: %.3f Min: %.3f Ave: %.3f St. D: %.3f\n", EcatJitterStat.max,
			EcatJitterStat.min,
			EcatJitterStat.ave,
			EcatJitterStat.std);
}

/****************************************************************************/