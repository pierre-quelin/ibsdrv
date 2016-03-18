/**
 * Library for Linux Driver ibs
 * (Phoenix Contact GmbH & Co. PCI Interbus controller boards IBS PCI SC/I-T)
 *
 * Copyright(c) 2010 Pierre Quelin <pierre.quelin.1972@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU Lesser General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU Lesser General Public License is included in this distribution
 * in the file called "COPYING".
 */


#ifndef IBSLIB_H_
#define IBSLIB_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef void* IBSMASTER;

/* Linux et Windows uniquement en première approche */
#if defined(__linux__)

#include <sys/types.h> /* __off_t */
#include <unistd.h> /* ssize_t */

#elif defined(__MINGW32__)

#undef __off_t
typedef int __off_t;
#include <unistd.h> /* ssize_t */

#else

#undef __off_t
typedef int __off_t;
#undef ssize_t
typedef int ssize_t;

#endif


/* System device access */
IBSMASTER ibsMasterOpen( const char* dev );
int ibsMasterClose( IBSMASTER master );

int ibsMasterReset( IBSMASTER master );

/* Specific register access */
unsigned short ibsSysFailGet( IBSMASTER master );
unsigned short ibsStatusGet( IBSMASTER master );
unsigned short ibsParamGet( IBSMASTER master );

/* Diagnostic Status Register Flags */
enum IBS_DIAG_FLAG
{
   IBS_DIAG_STATUS_USER       = 0x0001,
   IBS_DIAG_STATUS_PF         = 0x0002,
   IBS_DIAG_STATUS_BUS        = 0x0004,
   IBS_DIAG_STATUS_CTRL       = 0x0001,
   IBS_DIAG_STATUS_DETECT     = 0x0010,
   IBS_DIAG_STATUS_RUN        = 0x0020,
   IBS_DIAG_STATUS_ACTIVE     = 0x0040,
   IBS_DIAG_STATUS_READY      = 0x0080,
   IBS_DIAG_STATUS_BSA        = 0x0100,
   IBS_DIAG_STATUS_BASP       = 0x0200,
   IBS_DIAG_STATUS_SYSFAIL    = 0x0200,
   IBS_DIAG_STATUS_RESULT     = 0x0400,
   IBS_DIAG_STATUS_SYRESULT   = 0x0800,
   IBS_DIAG_STATUS_DCRESULT   = 0x1000,
   IBS_DIAG_STATUS_WARNING    = 0x2000,
   IBS_DIAG_STATUS_QUALITY    = 0x4000,
   IBS_DIAG_STATUS_SDSI       = 0x8000,
};

/* Interbus data access */
ssize_t ibsDataWrite( IBSMASTER master, const char* from, size_t count, __off_t offset );
ssize_t ibsDataRead( IBSMASTER master, char* to, size_t count, __off_t offset );

/* Interbus cycle control */
int ibsCyclePrepare( IBSMASTER master );

/* Controller messaging */
typedef struct
{
   unsigned short id;
   unsigned short nb;
   unsigned short body[254]; /* 512/sizeof(unsigned short) - 2 */
} IBSMSG;

enum IBS_SERVICES
{
   /* Service flags */
   IBS_SF_REQUEST                                  = 0x0000,
   IBS_SF_INDICATION                               = 0x4000,
   IBS_SF_CONFIRMATION                             = 0x8000,
   IBS_SF_RESPONSE                                 = 0xC000,
   IBS_SF_UNCONFIRMED                              = 0x0800,

   /* Services for Parameterizing the Controller Board */
   IBS_Control_Parameterization                    = 0x030E,
   IBS_Set_Indication                              = 0x0152,
   IBS_Set_Value                                   = 0x0750,
   IBS_Read_Value                                  = 0x0351,
   IBS_Initiate_Load_Configuration                 = 0x0306,
   IBS_Load_Configuration                          = 0x0307,
   IBS_Terminate_Load_Configuration                = 0x0308,
   IBS_Activate_Configuration                      = 0x0711,
   IBS_Deactivate_Configuration                    = 0x0712,
   IBS_Confirm_Diagnostics                         = 0x0760,

   /* Services for Defining Process Data Description */
   IBS_Initiate_Put_Process_Data_Description_List  = 0x0320,
   IBS_Put_Process_Data_Description_List           = 0x0321,
   IBS_Terminate_Put_Process_Data_Description_List = 0x0322,

   /* Services for assigning Process Data */
   IBS_Initiate_Load_Process_Data_Reference_List   = 0x0324,
   IBS_Load_Process_Data_Reference_List            = 0x0325,
   IBS_Terminate_Load_Process_Data_Reference_List  = 0x0326,

   /* Services for Direct INTERBUS Access */
   IBS_Start_Data_Transfer                         = 0x0701,
   IBS_Alarm_Stop                                  = 0x1303,
   IBS_Stop_Data_Transfer                          = 0x0702,

   /* Diagnostic Services */
   IBS_Get_Error_Info                              = 0x0316,
   IBS_Get_Version_Info                            = 0x032A,
   IBS_Get_Diag_Info                               = 0x032B,
   IBS_Control_Statistics                          = 0x030F,

   /* Automatic Indications of the Controller Board */
   IBS_Fault                                       = 0x0341,
   IBS_Device_Fail                                 = 0x1340,
   IBS_Bus_Error                                   = 0x2342,

   /* PCP */
   IBS_Load_CRL_Attribute_Loc                      = 0x0264,
   IBS_PCP_Read                                    = 0x0081,
   IBS_PCP_Write                                   = 0x0082,
   IBS_PCP_Status                                  = 0x0086,
   IBS_PCP_Initiate                                = 0x008B,
   IBS_PCP_Abort                                   = 0x088D,
};
int ibsSendMsg( IBSMASTER master, const IBSMSG* from, int timeout );
int ibsReceiveMsg( IBSMASTER master, IBSMSG* to );

/* Interrupt handler */
enum IBS_ISR_FLAG
{
   IBS_ISR_SYSFAIL_FLAG       = 0x01,
   IBS_ISR_MSG_SND_FLAG       = 0x02,
   IBS_ISR_MSG_RCV_FLAG       = 0x04,
   IBS_ISR_CYCLE_START_FLAG   = 0x08,
   IBS_ISR_CYCLE_END_FLAG     = 0x10,
   IBS_ISR_NODE_SIGNAL_FLAG   = 0x20,
};
int ibsIsrHandler( IBSMASTER master, int flag, int timeout );
int ibsIsrAck( IBSMASTER master, int flag );

#ifdef __cplusplus
}
#endif


#endif /* IBSLIB_H_ */
