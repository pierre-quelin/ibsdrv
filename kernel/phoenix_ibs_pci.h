/**
 * Driver for Phoenix Contact GmbH & Co. PCI Interbus controller boards
 * IBS PCI SC/I-T
 *
 * Copyright(c) 2009 Pierre Quelin <pierre.quelin.1972@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#ifndef PHOENIX_IBS_PCI_H_
#define PHOENIX_IBS_PCI_H_

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */

#define DEVICE_NAME "ibs"

#define INTERBUS_MAJOR 0   /* dynamic major by default */
#define INTERBUS_DEVS 4

/* Use 'i' as magic number */
#define INTERBUS_IOC_MAGIC  'i' // TODO - To be define

/* struct used by INTERBUS_IOCTQ_IO_READ/INTERBUS_IOCTQ_IO_WRITE */
typedef struct
{
   unsigned int addr;
   unsigned char value;
} INTERBUS_DEVIO_REG;

/* enum used by INTERBUS_IOCS_ISR_CLEAR_FLAG */
enum INTERBUS_ISR_FLAG
{
   INTERBUS_ISR_NO_FLAG          = 0x00,
   INTERBUS_ISR_SYSFAIL_FLAG     = 0x01,
   INTERBUS_ISR_MSG_SND_FLAG     = 0x02,
   INTERBUS_ISR_MSG_RCV_FLAG     = 0x04,
   INTERBUS_ISR_CYCLE_START_FLAG = 0x08,
   INTERBUS_ISR_CYCLE_END_FLAG   = 0x10,
   INTERBUS_ISR_NODE_SIGNAL_FLAG = 0x20,
};

/* struct used by INTERBUS_IOCQ_ISR_HND */
typedef struct
{
   int flag; /* flag to wait */
   int timeout; /* timeout in ms */
} INTERBUS_ISR_HND;

/*
 * S means "Set" through a pointer
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": switch G and S atomically
 * H means "sHift": switch T and Q atomically
 */
#define INTERBUS_IOCT_M_RESET             _IO(INTERBUS_IOC_MAGIC,   1)
#define INTERBUS_IOCX_IO_READ             _IOWR(INTERBUS_IOC_MAGIC, 2, INTERBUS_DEVIO_REG)
#define INTERBUS_IOCS_IO_WRITE            _IOW(INTERBUS_IOC_MAGIC,  3, INTERBUS_DEVIO_REG)
#define INTERBUS_IOCQ_ISR_HND             _IOW(INTERBUS_IOC_MAGIC,  4, INTERBUS_ISR_HND)
#define INTERBUS_IOCS_ISR_CLEAR_FLAG      _IOW(INTERBUS_IOC_MAGIC,  5, int)
#define INTERBUS_IOC_MAXNR 5

/**
 * I/O Register Definitions
 */
#define IO_BOARD_NUMBER                   0x00
#define IO_IRQ_CONTROL_HOST               0x02
#define IO_WDT_CONTROL_HOST               0x03
#define IO_RESET_CONTROL_HOST             0x04
#define IO_STATUS                         0x07

/**
 * MPM - Soft Registers
 */
#define MPM_REG_NODE_0                    0x3440
#define MPM_REG_NODE_1                    0x378a
/*
#define MPM_REG_NODE_2                    0x3aaa
#define MPM_REG_NODE_3                    0x3dca
*/

/**
 * I/O Register Masks / Constants
 */
/* IO IRQ CONTROL HOST MASK */
#define BB_CTL_W_IRQ_DISABLE              0x00
#define BB_CTL_W_IRQ_ENABLE               0x08
#define BB_CTL_R_IRQ_MASTER1              0x40
/*
#define BB_CTL_R_IRQ_MASTER2              0x20
*/

/**
 * I/O Register Masks / Constants
 */
/* IO_IRQ_CONTROL_HOST */
#define IO_RESET_MASTER1                  0xCA
/*
#define IO_RESET_MASTER2                  0xC5
*/
#define IO_SELF_TEST_MASTER1              0x01
/*
#define IO_SELF_TEST_MASTER2              0x02
*/


/**
 * MPM CONFIGURATION MASK
 */
#define MPM_NODE_PAR_READY_0              0x0001
#define MPM_NODE_PAR_READY_1              0x0002
/*
#define MPM_NODE_PAR_READY_2              0x0004
#define MPM_NODE_PAR_READY_3              0x0008
*/
#define GENERATE_HOST_BUS_TIMEOUT         0x0040
#define FUNCTION_DECODER_PRESENT          0x1000
#define MPM_SIZE                          0xC000
#define MPM_SIZE_16KB                     0x0000
#define MPM_SIZE_64KB                     0x4000
#define MPM_SIZE_256KB                    0x8000
#define MPM_SIZE_512KB                    0xC000
#define RESERVED                          0x2fb0
/**
 * STATUS REGISTER 1 MASK
 */
#define MPM_NODE_READY_0                  0x8000
#define HS_INT_0                          0x2000
#define SYS_FAIL_0                        0x1000
#define MPM_NODE_READY_1                  0x0800
#define HS_INT_1                          0x0200
#define SYS_FAIL_1                        0x0100
/*
#define MPM_NODE_READY_2                  0x0080
#define HS_INT_2                          0x0020
#define SYS_FAIL_2                        0x0010
#define MPM_NODE_READY_3                  0x0008
#define HS_INT_3                          0x0002
#define SYS_FAIL_3                        0x0001
*/
/**
 * STATUS REGISTER 2 MASK
 */
#define SYNC_REQ_0                        0x1000
#define TIMEOUT_ACCESSOR_0                0xC000
#define SYNC_REQ_1                        0x0100
#define TIMEOUT_ACCESSOR_1                0x0C00
/*
#define SYNC_REQ_2                        0x0010
#define TIMEOUT_ACCESSOR_2                0x00C0
#define SYNC_REQ_3                        0x0001
#define TIMEOUT_ACCESSOR_3                0x000C
*/
/**
 * STATUS SYSFAIL REG MASK
 */
#define STATUS_SYSFAIL_0                  0x1000
#define STATUS_SYSFAIL_1                  0x0100
/*
#define STATUS_SYSFAIL_2                  0x0010
#define STATUS_SYSFAIL_3                  0x0001
*/

/**
 * MPM Hardware Registers
 */
/**
 * Readable Registers
 */
#define MPM_R_CONFIGURATION               0x3f90
#define MPM_R_READ_MEMORY_PAGE            0x3f98
#define MPM_R_RDY_BITS                    0x3fa2
#define MPM_RW_SERIAL_DATA                0x3fa4
#define MPM_R_STATUS_REG_1                0x3fb0
#define MPM_R_STATUS_SYSFAIL_REG          0x3fb2
#define MPM_R_STATUS_NODE_SG_INT_REG      0x3fb4 /* Status node signal interface register */
#define MPM_R_STATUS_REG_2                0x3fb6
#define MPM_R_HANDSHAKE_REG_A             0x3fc0
#define MPM_R_HANDSHAKE_REG_B             0x3fc2

// TODO
#define MPM_RW_DIAG_REG                   0x3520
#define MPM_RW_DIAG_PARAM_REG             0x3522
#define MPM_RW_CTRL_REG                   0x3524
#define MPM_RW_STATE_REG                  0x3526
/*
#define MPM_R_DIAG_EX_REG                 0x37E6
*/
/**
 * CTRL REG MASK
 */
#define MB_CTRL_REG_DATA_CYCLE_ACTIVATE   0x4000
#define MB_STATE_REG_DATA_CYCLE           0x4000

#define MB_CLEAR_STATUS_SYSFAIL           0x8000
#define MB_CLEAR_STATUS_SYNC_REQ          0x4000
#define MB_CLEAR_STATUS_MPM_TO            0x1000

/**
 * Writable Registers
 */
#define MPM_W_SET_MPM_NODE_PAR_READY_0    0x3f90
#define MPM_W_SET_MPM_NODE_PAR_READY_1    0x3f92
/*
#define MPM_W_SET_MPM_NODE_PAR_READY_2    0x3f94
#define MPM_W_SET_MPM_NODE_PAR_READY_3    0x3f96
*/
#define MPM_W_SWITCH_MEMORY               0x3f98
#define MPM_W_SET_GHBT                    0x3f9a
#define MPM_W_SET_SYNC_REQ                0x3f9c
#define MPM_W_SET_FUNC_DECODER_ACTIVE_BIT 0x3f9e
#define MPM_W_SET_SYSFAIL_REQ             0x3fa0
#define MPM_W_PROGRAM_BITS                0x3fa2
/* MPM_RW_SERIAL_DATA */
#define MPM_W_SERIAL_ADDRESS              0x3fa6
#define MPM_W_CLEAR_STATUS_BIT_0          0x3fa8
#define MPM_W_CLEAR_STATUS_BIT_1          0x3faa
/*
#define MPM_W_CLEAR_STATUS_BIT_2          0x3fac
#define MPM_W_CLEAR_STATUS_BIT_3          0x3fae
*/
#define MPM_W_SET_MPM_NODE_SG_INT_0       0x3fb0
#define MPM_W_SET_MPM_NODE_SG_INT_1       0x3fb2
#define MPM_W_SET_MPM_NODE_SG_INT_2       0x3fb4
#define MPM_W_SET_MPM_NODE_SG_INT_3       0x3fb6
#define MPM_W_SET_MPM_NODE_READY_0        0x3fb8
#define MPM_W_SET_MPM_NODE_READY_1        0x3fba
/*
#define MPM_W_SET_MPM_NODE_READY_2        0x3fbc
#define MPM_W_SET_MPM_NODE_READY_3        0x3fbe
*/
#define MPM_W_SET_HS_A_3                  0x3fc0
#define MPM_W_SET_HS_A_11                 0x3fc2
#define MPM_W_SET_HS_B_3                  0x3fc4
#define MPM_W_SET_HS_B_11                 0x3fc6
#define MPM_W_SET_HS_A_7                  0x3fc8
#define MPM_W_SET_HS_A_16                 0x3fca /* A_15 ? */
#define MPM_W_SET_HS_B_7                  0x3fcc
#define MPM_W_SET_HS_B_16                 0x3fce /* B_15 ? */
#define MPM_W_SET_HS_A_2                  0x3fd0
#define MPM_W_SET_HS_A_10                 0x3fd2
#define MPM_W_SET_HS_B_2                  0x3fd4
#define MPM_W_SET_HS_B_10                 0x3fd6
#define MPM_W_SET_HS_A_6                  0x3fd8
#define MPM_W_SET_HS_A_14                 0x3fda
#define MPM_W_SET_HS_B_6                  0x3fdc
#define MPM_W_SET_HS_B_14                 0x3fde
#define MPM_W_SET_HS_A_1                  0x3fe0
#define MPM_W_SET_HS_A_9                  0x3fe2
#define MPM_W_SET_HS_B_1                  0x3fe4
#define MPM_W_SET_HS_B_9                  0x3fe6
#define MPM_W_SET_HS_A_5                  0x3fe8
#define MPM_W_SET_HS_A_13                 0x3fea
#define MPM_W_SET_HS_B_5                  0x3fec
#define MPM_W_SET_HS_B_13                 0x3fee
#define MPM_W_SET_HS_A_0                  0x3ff0
#define MPM_W_SET_HS_A_8                  0x3ff2
#define MPM_W_SET_HS_B_0                  0x3ff4
#define MPM_W_SET_HS_B_8                  0x3ff6
#define MPM_W_SET_HS_A_4                  0x3ff8
#define MPM_W_SET_HS_A_12                 0x3ffa
#define MPM_W_SET_HS_B_4                  0x3ffc
#define MPM_W_SET_HS_B_12                 0x3ffe

#endif /* PHOENIX_IBS_PCI_H_ */
