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

#if defined(__linux__)

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <asm/byteorder.h>
#include <sys/ioctl.h>
#include <time.h>
#include "phoenix_ibs_pci.h"
#include <stdlib.h>

#else

/* Windows en premiere approche */
#include <windows.h>

#endif

#include "ibs.h"


/**
 * struct interbus_mpm_area
 */
struct interbus_mpm_area
{
   unsigned short start; /**< Start address */
   unsigned short len; /**< Length */
};

/**
 * struct interbus_node_descriptor
 */
struct interbus_node_descriptor
{
   struct interbus_mpm_area data; /**< Data area */
   struct interbus_mpm_area extData; /**< Extended data area */
   struct interbus_mpm_area mailbox; /**< Mailbox area */
   struct interbus_mpm_area dataSegs[2][4]; /**< Division of the data area */
   unsigned short svr[2][4]; /**< Send vector register addresses */
   unsigned short avr[2][4]; /**< Acknowledge vector register addresses */
   unsigned short snr[2][4]; /**< Sub node register addresses */
};

/**
 * Interbus master handle
 */
typedef struct
{
   int    fd; /**< The interbus master file descriptor */
   struct interbus_node_descriptor node[4]; /**< The nodes configuration */
} IBSMHND;


unsigned short ibsRead16 (IBSMHND* mstr, int addr)
{
  unsigned short data;

  lseek(mstr->fd, addr, SEEK_SET);
  read(mstr->fd, &data, sizeof(data));
  return __be16_to_cpu(data);
}

void ibsWrite16 (IBSMHND* mstr, int addr, unsigned short data)
{
  data = __be16_to_cpu(data);
  lseek(mstr->fd, addr, SEEK_SET);
  write(mstr->fd, &data, sizeof(data));
}


/**
 * Open an interbus master device
 *
 * @param dev The device. (ex. "/dev/ibs0")
 * @return An interbus master handle. A value of -1 is returned in case of
 * failure.
 */
IBSMASTER ibsMasterOpen( const char* dev )
{
   IBSMHND* mstr = NULL;

#if defined(__linux__)
   unsigned int node;
   unsigned int addr;
   unsigned short* desc;
   unsigned int offset;
   INTERBUS_DEVIO_REG reg;

   mstr = malloc(sizeof(IBSMHND));

   mstr->fd = open(dev, O_RDWR);
   if( mstr->fd == -1 )
   {
      free(mstr);
      return (IBSMHND*)-1;
   }

   /* Disable the device interrupt */
   reg.addr = IO_IRQ_CONTROL_HOST;
   reg.value = BB_CTL_W_IRQ_DISABLE;
   (void)ioctl(mstr->fd, INTERBUS_IOCS_IO_WRITE, &reg);

   /* Get software vector register addresses */
   /* Just for the 2 first nodes */
   for( node = 0 ; node <= 1 ; node++)
   {
      switch(node)
      {
         case 0:
            addr = MPM_REG_NODE_0;
            break;
         case 1:
            addr = MPM_REG_NODE_1;
            break;
         default:
            break;
      }

      desc = (unsigned short*)&(mstr->node[node]);
      for(offset=0 ; offset < sizeof(struct interbus_node_descriptor)/sizeof(unsigned short) ; offset++)
      {
         desc[offset] = ibsRead16 (mstr, addr + sizeof(unsigned short)*offset);
      }
   }

   (void)ibsMasterReset(mstr);
#else
   (void)mstr;
#endif

   return (IBSMASTER)mstr;
}

/**
 * Close an interbus master device.
 *
 * @param master An interbus master handle.
 * @return The normal return value is 0. A value of -1 is returned in case of
 * failure.
 */
int ibsMasterClose( IBSMASTER master )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)

   close(mstr->fd);

   free(mstr);
#else
   (void)mstr;
#endif

   return 0;
}

/**
 * Reset an interbus master device.
 *
 * @param master An interbus master handle.
 * @return The normal return value is 0. A value of -1 is returned in case of
 * failure.
 */
int ibsMasterReset( IBSMASTER master )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)
   INTERBUS_DEVIO_REG reg;
   int count;

   /* Disable the device interrupt */
   reg.addr = IO_IRQ_CONTROL_HOST;
   reg.value = BB_CTL_W_IRQ_DISABLE;
   (void)ioctl(mstr->fd, INTERBUS_IOCS_IO_WRITE, &reg);

   /* reset the board (max 10s) */
   reg.addr = IO_RESET_CONTROL_HOST;
   reg.value = IO_RESET_MASTER1;
   ioctl(mstr->fd, INTERBUS_IOCS_IO_WRITE, &reg);

   count = 0;
   reg.addr = IO_STATUS;
   reg.value = 0;
   (void)ioctl(mstr->fd, INTERBUS_IOCX_IO_READ, &reg);
   while( (reg.value & 0x04) == 0x04 ) /* TODO - #define */
   {
      sleep(1);

      (void)ioctl(mstr->fd, INTERBUS_IOCX_IO_READ, &reg);

      count++;
      if( count > 10 )
         break; // TODO
   }

   /* Wait until the second node (68xxx) become ready */
   while( (ibsRead16 (mstr,MPM_R_STATUS_REG_1) & MPM_NODE_READY_1) != MPM_NODE_READY_1 )
   {
      sleep(1);
      count++;
      if( count > 10 )
         break; // TODO
   }

   /* Initialise the given card */
   reg.addr = IO_STATUS;
   reg.value = 0;
   (void)ioctl(mstr->fd, INTERBUS_IOCS_IO_WRITE, &reg);

   /* Erase sysfail bit */
   ibsWrite16 (mstr,  MPM_W_CLEAR_STATUS_BIT_0, MB_CLEAR_STATUS_SYSFAIL);

   /* Mark the host MPM nodes as ready (to inform the 68xxx)*/
   ibsWrite16 (mstr,  MPM_W_SET_MPM_NODE_READY_0, MPM_NODE_READY_0);
   ibsWrite16 (mstr, MPM_W_SET_MPM_NODE_PAR_READY_0, MPM_NODE_READY_0);

   /* Wait until the two nodes become ready */
   while( (ibsRead16 (mstr,  MPM_R_STATUS_REG_1) & (MPM_NODE_READY_0 | MPM_NODE_READY_1)) != (MPM_NODE_READY_0 | MPM_NODE_READY_1) )
   {
      sleep(1);
      count++;
      if( count > 10 )
         break; // TODO
   }

   /**
    * Set the Data Cycle Activate bit so the handshake doesn't start
    * immediately after the START DATA TRANSFER command
    */
   ibsWrite16 (mstr,  MPM_RW_CTRL_REG, ibsRead16 (mstr,  MPM_RW_CTRL_REG) | MB_CTRL_REG_DATA_CYCLE_ACTIVATE);

   /* Enable the device interrupt */
   reg.addr = IO_IRQ_CONTROL_HOST;
   reg.value = BB_CTL_W_IRQ_ENABLE;
   (void)ioctl(mstr->fd, INTERBUS_IOCS_IO_WRITE, &reg);
#else
   (void)mstr;
#endif

   return 0;
}

/**
 * Read the SysFail master device register.
 *
 * @param master An interbus master handle.
 * @return The SysFail master device register.
 */
unsigned short ibsSysFailGet( IBSMASTER master )
{
   IBSMHND* mstr = (IBSMHND*)master;

   unsigned short data = 0;

#if defined(__linux__)
   data = ibsRead16 (mstr,  MPM_R_STATUS_SYSFAIL_REG);
#else
   (void)mstr;
#endif

   return data;
}

/**
 * Read the Status master device register.
 *
 * @param master An interbus master handle.
 * @return The Status master device register.
 */
unsigned short ibsStatusGet( IBSMASTER master )
{
   IBSMHND* mstr = (IBSMHND*)master;

   unsigned short data = 0;

#if defined(__linux__)
   data = ibsRead16 (mstr, MPM_RW_DIAG_REG);
#else
   (void)mstr;
#endif

   return data;
}

/**
 * Read the Param master device register.
 *
 * @param master An interbus master handle.
 * @return The Param master device register.
 */
unsigned short ibsParamGet( IBSMASTER master )
{
   IBSMHND* mstr = (IBSMHND*)master;

   unsigned short data = 0;

#if defined(__linux__)
   data = ibsRead16 (mstr, MPM_RW_DIAG_PARAM_REG);
#else
   (void)mstr;
#endif

   return data;
}

/**
 * Prepare the interbus cycle.
 *
 * @param master An interbus master handle.
 * @return The normal return value is 0. A value of -1 is returned in case of
 * failure.
 */
int ibsCyclePrepare( IBSMASTER master )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)

   /* Check master status */
   if( ( ibsRead16 (mstr, MPM_RW_DIAG_REG) & 0xE0) != 0xE0 ) /* TODO - #define */
   {
      return -1;
   }

   /* Start cycle */
   ibsWrite16 (mstr,  MPM_RW_CTRL_REG, ibsRead16 (mstr, MPM_RW_CTRL_REG) & ~MB_STATE_REG_DATA_CYCLE);
   ibsWrite16 (mstr, MPM_W_SET_HS_A_13, 0x8000);/* TODO - #define */
#else
   (void)mstr;
#endif

   return 0;
}

/**
 * Send an interbus message.
 *
 * @param master An interbus master handle.
 * @param timeout ms
 * @return The normal return value is 0. A value of -1 is returned in case of
 * failure.
 */
int ibsSendMsg( IBSMASTER master, const IBSMSG* from, int timeout )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)
   unsigned int offset, count;
   unsigned short* msg;
   //INTERBUS_DEVIO_REG reg;
   int result;

   count = 2 + from->nb; /* size in u16 (id + nb + body[nb]) */
   if ( count > (sizeof(IBSMSG)/2) )
   {
      return -1;
   }

   /* Write the message into the MPM */
   msg = (unsigned short*)from;
   for(offset=0 ; offset < count ; offset++)
   {
      ibsWrite16 (mstr,  mstr->node[0].mailbox.start + offset *sizeof (unsigned short), msg[offset]);
   }

   /* Tell the controller about the new message and its position */
   ibsWrite16 (mstr, mstr->node[0].svr[0][1], mstr->node[0].mailbox.start);

   /* Send "Message present" from MPM accessor 0 to MPM accessor 1 (68xxx) */
   ibsWrite16 (mstr, MPM_W_SET_HS_A_8, 0x8000);/* TODO - #define */

   /* Wait the ISR acknowledge */
   result = ibsIsrHandler(mstr, IBS_ISR_MSG_SND_FLAG, timeout );
   if ( result & IBS_ISR_MSG_SND_FLAG )
   {
      ibsIsrAck(mstr, IBS_ISR_MSG_SND_FLAG);
      return 0;
   }
   else
   {
      return -1;
   }
#else
   (void)mstr;
#endif

   return 0;
}

/**
 * Receive an interbus message.
 *
 * @param master An interbus master handle.
 * @return The normal return value is 0. A value of -1 is returned in case of
 * failure.
 */
int ibsReceiveMsg( IBSMASTER master, IBSMSG* to )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)
   unsigned int offset;
   unsigned short addr;
   unsigned short id, nb;
   unsigned short* msg;

//   /* Message reception */
//   lseek(mstr->fd, MPM_R_HANDSHAKE_REG_A, SEEK_SET);
//   read(mstr->fd, &data, sizeof(addr));
//   if ( (data & 0x0002) != 0x0002 )
//   {
//      return -1;
//   }

   /* get the send-vector register addresse */
   addr = ibsRead16 (mstr,  mstr->node[0].svr[1][1]);

   /* read the msg id */
   id = ibsRead16 (mstr, addr);

   /* read the msg size (nb x unsigned short) */
   nb = ibsRead16 (mstr, addr+2);

   if ( 2 + nb > (sizeof(IBSMSG)/2) )
   {
      return -1;
   }

   msg = (unsigned short*)to;
   msg[0] = id;
   msg[1] = nb;
   for( offset = 2 ; offset < (2 + nb); offset++)
   {
      msg[offset] = ibsRead16 (mstr, addr+offset *sizeof (unsigned short));
   }

   /* set the acknowledge vector register */
   ibsWrite16 (mstr,  mstr->node[0].avr[0][1], addr);
   ibsWrite16 (mstr, MPM_W_SET_HS_A_12, 0x8000); /* TODO - #define */
#else
   (void)mstr;
#endif

   return 0;
}

/**
 * Write data at the specified master offset memory.
 *
 * @param master An interbus master handle.
 * @param from The data address.
 * @param count The data size in bytes.
 * @param offset The offset in master memory.
 * @return The return value is the number of bytes actually written. This must
 * be size. In the case of an error, returns -1.
 */
ssize_t ibsDataWrite( IBSMASTER master, const char* from, size_t count, __off_t offset )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)
   lseek(mstr->fd, offset, SEEK_SET);
   return write(mstr->fd, from, count);
#else
   (void)mstr;

   return 0;
#endif
}

/**
 * Read data at the specified master offset memory.
 *
 * @param master An interbus master handle.
 * @param to The data address.
 * @param count The data size in bytes.
 * @param offset The offset in master memory.
 * @return The return value is the number of bytes actually read. This must
 * be size. In the case of an error, returns -1.
 */
ssize_t ibsDataRead( IBSMASTER master, char* to, size_t count, __off_t offset )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)
   lseek(mstr->fd, offset, SEEK_SET);
   return read(mstr->fd, to, count);
#else
   (void)mstr;

   return 0;
#endif
}

/**
 * Wait one or more interrupt event.
 *
 * @param master An interbus master handle.
 * @param flag The events to wait.
 * @param timeout The maximum delay to wait (ms). A value of -1 indicate
 * forever.
 * @return The flags set by the interrupt routine. A value of 0 is returned in
 * case of failure or timeout.
 */
int ibsIsrHandler( IBSMASTER master, int flag, int timeout )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)
   int flag_set = 0;
   INTERBUS_ISR_HND isr_hnd;
   isr_hnd.flag = flag;
   isr_hnd.timeout = timeout;

   flag_set = ioctl(mstr->fd, INTERBUS_IOCQ_ISR_HND, &isr_hnd);
   if ( 0 > flag_set )
   {
      return 0;
   }

   return flag_set;
#else
   (void)mstr;

   Sleep(timeout);

   return 0;
#endif
}

/**
 * TODO
 */
int ibsIsrAck( IBSMASTER master, int flag )
{
   IBSMHND* mstr = (IBSMHND*)master;

#if defined(__linux__)
   int flag_ack = flag;
   ioctl(mstr->fd, INTERBUS_IOCS_ISR_CLEAR_FLAG, &flag_ack);
#else
   (void)mstr;
#endif

   return 0;
}
