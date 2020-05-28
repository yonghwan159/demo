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
#ifndef EMBD_ECATM_H
#define EMBD_ECATM_H
/*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
/****************************************************************************/
#include <ecrt.h>
/****************************************************************************/
#include "ptlLSMECA.h"
/************************ss****************************************************/

/*EtherCAT State Machine*/
#define INIT			(0x01)
#define PREOP			(0x02)
#define SAFEOP			(0x04)
#define	OP			(0x08)

/* Slaves  */
#define ALIAS_POSITION(x) 	0,x
#define _ALLNODE 		(-5)
#define LSMECAPION_SLAVENUM 	6
#define LS_SLAVENUM 	4

/*Covertion Macros */
#define RADPS2RPS(x)		(x*(0.1591549430919))
#define RPS2RADPS(x)		(x/(0.1591549430919))
#define RPM2RPS(x)		(x/60)
#define RPS2RPM(x)		(x*60)

/****************************************************************************/
typedef struct EcatStatus {
	
	unsigned int master_state;
	unsigned int slave_state;
	unsigned int slave_number;
} ECAT_STAT;

/*****************************************************************************/
/* Public Functions */

/* EtherCAT*/
void EcatWriteAppTimeToMaster(uint64_t masterAppTime);
int  EcatInit(int cycleTime, int operationMode);
void EcatQuit(void);
void EcatReceiveProcessDomain(void);
void EcatSendProcessDomain(void);
ECAT_STAT EcatStatusCheck(void);

/* LS Mecapion Servo Control*/
void lsmecaReady(int iNode);
void lsmecaServoOn(int iNode);
void lsmecaServoOff(int iNode);
void lsmecaShutDown(int iNode);
void lsmecaSetVelocity(int iNode, int RevPerMinute);
void lsmecaSetAngularVelocity(int iNode, float AngularVelocity);
unsigned short lsmecaGetStatusWordN(int iNode);
int lsmecaGetActualVelocityN(int iNode);
float lsmecaGetActualAngularVelocityN(int iNode);
int lsmecaGetActualPosition(int iNode);
void lsmecaSet(int iNode, int lcd);
#endif//EMBD_ECATM_H
