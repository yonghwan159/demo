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
 *  2016 Raimarius Delgado
*/
/****************************************************************************/
#ifndef LSMECAPION_ESI_H
#define LSMECAPION_ESI_H

#include "embdECATM_digitalout.h"

/****************************************************************************/

#define LS_SERVO 0x00000002, 0x07e83052 // Vendor ID , Product Code
#define Beckhoff_EK1100 0x00000002, 0x044c2c52

#define LS_SERVO_REGS(x,y) \
	{ALIAS_POSITION(x), LS_SERVO, 0x7000, 0x01 , &lsmecaCtrlWordOff[y]} 
	
/****************************************************************************/

unsigned int lsmecaCtrlWordOff[LSMECAPION_SLAVENUM]	=	{0,};

/*****************************************************************************/
ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x7000, 0x01, 1}, /* Output */
    {0x7010, 0x01, 1}, /* Output */
    {0x7020, 0x01, 1}, /* Output */
    {0x7030, 0x01, 1}, /* Output */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 1, slave_1_pdo_entries + 0}, /* Channel 1 */
    {0x1601, 1, slave_1_pdo_entries + 1}, /* Channel 2 */
    {0x1602, 1, slave_1_pdo_entries + 2}, /* Channel 3 */
    {0x1603, 1, slave_1_pdo_entries + 3}, /* Channel 4 */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 4, slave_1_pdos + 0, EC_WD_ENABLE},
    {0xff}
};



#endif //LSMECAPION_ESI_H

