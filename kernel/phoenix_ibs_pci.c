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
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>   /* copy_*_user */


#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>

#include "phoenix_ibs_pci.h"

static int interbus_major = INTERBUS_MAJOR;
static int interbus_devs  = INTERBUS_DEVS;
static int interbus_debug = 0;
static int interbus_fake  = 0;

module_param(interbus_major, int, 0);
module_param(interbus_devs, int, 0);

module_param(interbus_debug, int, 0644);
module_param(interbus_fake, int, 0644);

#define dprintk(args...) \
   do { \
      if (interbus_debug) printk(KERN_DEBUG args); \
   } while (0)


/**
 * Interbus device capabilities
 */
struct interbus_dev
{
   unsigned long iobase; /**< I/O port base address of the device */
   unsigned long iosize; /**< I/O port size of the device */
   unsigned long membase; /**< I/O memory base address of the device */
   unsigned long memsize; /**< I/O memory size of the device */
   void __iomem* membase_mapped; /**< The device mapped I/O memory address */

   int irq; /**< The IRQ number */
   irqreturn_t (*isr)(int irq, void* dev); /**< The interrupt service routine */

   wait_queue_head_t wait_q_isr;  /**< Wait queue interrupt */
   int isr_flag; /**< The isr flag register */

   int minor; /**< Device minor number */
   struct cdev cdev;

   /* TODO if read and write simultaneously ? */
   void* buffer; /**< Used as temporary buffer to optimized read/write operation in io memory */
};

/**
 * Interbus driver capabilities
 */
struct interbus_drv
{
   char* name; /**< Not used */
   char* version; /**< Not used */

   struct semaphore devCountMutex; /**< To protect devCount */
   int devCount; /**< Number of interbus board. Used for the minor number */
};

/**
 * TODO
 */
static struct interbus_drv* ibs_drv;
static struct class* ibs_class;


/**
 * Read an IO port register.
 *
 * @param dev The device
 * @param reg The IO port address register
 * @return The IO port value
 **/
static unsigned char interbus_read_io(struct interbus_dev* dev, u32 reg)
{
  return inb(dev->iobase + reg);
}

/**
 * Write an IO port register.
 *
 * @param dev The device
 * @param reg The IO port address register
 * @param value The IO port value
 **/
static void interbus_write_io(struct interbus_dev* dev, u32 reg, unsigned char value)
{
  outb(value, dev->iobase + reg);
}

/**
 * Read an IO memory.
 *
 * @param dev The device
 * @param offset The IO memory relative address
 * @return The IO memory value
 **/
static u16 interbus_read_mem_16(struct interbus_dev* dev, u32 offset)
{
   return be16_to_cpu(ioread16(dev->membase_mapped + offset));
}

/**
 * Write an IO memory.
 *
 * @param dev The device
 * @param offset The IO memory relative address
 * @param value The IO memory value
 **/
static void interbus_write_mem_16(struct interbus_dev* dev, u32 offset, u16 value)
{
   iowrite16(cpu_to_be16(value), dev->membase_mapped + offset);
}

/**
 * Enable the device interrupt.
 *
 * @param dev The device
 * @return The normal return value is 0; a value of -1 is returned in case of
 * failure.
 **/
static int interbus_enable_interrupt(struct interbus_dev* dev)
{
   if (0 != request_irq(dev->irq, dev->isr, IRQF_SHARED, DEVICE_NAME, dev))
   {
      return -1;
   }
   interbus_write_io(dev, IO_IRQ_CONTROL_HOST, BB_CTL_W_IRQ_ENABLE);

   return 0;
}

/**
 * Disable the device interrupt.
 *
 * @param dev The device
 * @return The normal return value is 0; a value of -1 is returned in case of
 * failure.
 **/
static int interbus_disable_interrupt(struct interbus_dev* dev)
{
   interbus_write_io(dev, IO_IRQ_CONTROL_HOST, BB_CTL_W_IRQ_DISABLE);
   free_irq(dev->irq, dev);

   return 0;
}

/**
 * Copy data from the I/O memory to user space.
 *
 * The interbus_read() function reads up to @count bytes from the
 * I/O mapped memory at offset @ppos into the user space address starting
 * at @to.
 *
 * @param file The file descriptor
 * @param to The user space buffer to read to
 * @param count The maximum number of bytes to read
 * @param ppos The current position in the I/O memory
 * @return On success, the number of bytes read is returned and the offset
 * @ppos is advanced by this number, or negative value is returned on error.
 **/
static ssize_t interbus_read(struct file* file,
                             char __user* to,
                             size_t count,
                             loff_t* ppos)
{
   struct interbus_dev* dev = file->private_data;

   loff_t pos = *ppos;
   u16* buffer = (u16*)(dev->buffer + pos); /* temporary buffer */
   /*const */u16* from = (u16*)(dev->membase_mapped + pos); /* the mpm to read from */
   size_t available = dev->memsize; /* the size of the mpm */
   int offset;

   if (pos < 0)
      return -EINVAL;
   if (pos >= available || !count)
      return 0;
   if (count > available - pos)
      count = available - pos;
   if (count%sizeof(u16) != 0)
      dprintk("BRD %d interbus_read odd bytes\n", dev->minor);

   for(offset=0 ; offset < count/sizeof(u16) ; offset++)
   {
      buffer[offset] = ioread16(&from[offset]);
   }
   if (copy_to_user(to, dev->buffer + pos, count))
      return -EFAULT;

   if ( 2 == count ) /* DEBUG */
   {
      dprintk("BRD %d interbus_read pos:0x%04X value:0x%04X\n", dev->minor, (unsigned int)pos, (unsigned short)be16_to_cpu(*buffer) );
   }
   else
   {
      dprintk("BRD %d interbus_read pos:0x%04X count:%zu\n", dev->minor, (unsigned int)pos, count);
   }

   *ppos = pos + count;

   return count;
}

/**
 * Copy data from user space to the I/O memory.
 *
 * The interbus_write function reads up to @count bytes from the user
 * space address starting at @from into the I/O mapped memory at offset @ppos.
 *
 * @param file The file descriptor
 * @param ppos The current position in the I/O memory
 * @param from The user space buffer to read from
 * @param count The maximum number of bytes to read
 *
 * @return On success, the number of bytes written is returned and the offset
 * @ppos is advanced by this number, or negative value is returned on error.
 **/
static ssize_t interbus_write(struct file* file,
                              const char __user* from,
                              size_t count,
                              loff_t* ppos)
{
   struct interbus_dev* dev = file->private_data;


   loff_t pos = *ppos;
   u16* buffer = (u16*)(dev->buffer + pos); /* temporary buffer */
   u16* to = (u16*)(dev->membase_mapped + pos); /* the mpm to write to */
   size_t available = dev->memsize; /* the size of the mpm */
   int offset;

   if (pos < 0)
      return -EINVAL;
   if (pos >= available || !count)
      return 0;
   if (count > available - pos)
      count = available - pos;
   if (count%sizeof(u16) != 0)
      dprintk("BRD %d interbus_write odd bytes\n", dev->minor);

   if (copy_from_user(dev->buffer+pos, from, count))
      return -EFAULT;

   for(offset=0 ; offset < count/sizeof(u16) ; offset++)
   {
      iowrite16(buffer[offset], &to[offset]);
   }

   if ( 2 == count ) /* DEBUG */
   {
      dprintk("BRD %d interbus_write pos:0x%04X value:0x%04X\n", dev->minor, (unsigned int)pos, (unsigned short)be16_to_cpu(*buffer) );
   }
   else
   {
      dprintk("BRD %d interbus_write pos:0x%04X count:%zu\n", dev->minor, (unsigned int)pos, (unsigned int)count);
   }

   *ppos = pos + count;

   return count;

}

/**
 * Reposition interbus_read/interbus_write file offset.
 *
 * Repositions the file position pointer associated with the file descriptor
 * @file as follows:
 * If @orig is SEEK_SET, the offset is set to @offset bytes.
 * If @orig is SEEK_CUR, the offset is set to its current location plus @offset
 * bytes.
 * If @orig is SEEK_END, the offset is set to the size of the file plus @offset
 * bytes.
 *
 * @param file The file descriptor
 * @param offset The offset in bytes
 * @param orig SEEK_SET, SEEK_CUR or SEEK_END
 * @return The resulting offset location as measured in bytes from the
 * beginning of the file. Otherwise, a value of -1 is returned
 **/
static loff_t interbus_llseek (struct file* file,
                              loff_t offset,
                              int orig)
{
   loff_t rval;
   struct interbus_dev* dev = file->private_data;

//   dprintk("BRD %d interbus_llseek offset:0x%04X\n", dev->minor, (unsigned int)offset);

   switch (orig)
   {
      case SEEK_SET:
         break;
      case SEEK_CUR:
         offset += file->f_pos;
         break;
      case SEEK_END:
         offset += dev->memsize;
         break;
      default:
         return -EINVAL;
   }

   if ( (offset < 0) ||
        (offset > dev->memsize) )
   {
      return -EINVAL;
   }

   rval = file->f_pos = offset;

   return rval;
}

/**
 * interbus_ioctl - control device.
 *
 * function manipulates the underlying device parameters of special files.
 *
 * @param inode Pointer to the inode
 * @param file The file descriptor
 * @param cmd The request code
 * @param arg Untyped pointer to parameters
 * @return On success zero is returned.
 **/
   static long interbus_ioctl(struct file* file,
                              unsigned int cmd,
                              unsigned long arg)
{
   struct interbus_dev* dev = file->private_data;
   int err = 0;
   int retval = 0;
   int isr_flag = 0;
   INTERBUS_ISR_HND isr_hnd;
   INTERBUS_DEVIO_REG reg;

//   mutex_lock(&file->f_dentry->d_inode->i_mutex); // TODO

   /*
    * extract the type and number bitfields, and don't decode
    * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
    */
   if (_IOC_TYPE(cmd) != INTERBUS_IOC_MAGIC)
   {
//      mutex_unlock(&file->f_dentry->d_inode->i_mutex);
      return -ENOTTY;
   }
   if (_IOC_NR(cmd) > INTERBUS_IOC_MAXNR)
   {
//      mutex_unlock(&file->f_dentry->d_inode->i_mutex);
      return -ENOTTY;
   }

   /*
    * the direction is a bitmask, and VERIFY_WRITE catches R/W
    * transfers. `Type' is user-oriented, while
    * access_ok is kernel-oriented, so the concept of "read" and
    * "write" is reversed
    */
   if (_IOC_DIR(cmd) & _IOC_READ)
      err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
   else if (_IOC_DIR(cmd) & _IOC_WRITE)
      err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
   if (err)
   {
//      mutex_unlock(&file->f_dentry->d_inode->i_mutex);
      return -EFAULT;
   }

   switch (cmd)
   {
      case INTERBUS_IOCX_IO_READ:
         if (copy_from_user(&reg, (void __user *)arg, sizeof(reg)))
            return -EFAULT;
         if ( reg.addr >= dev->iosize )
         {
            printk(KERN_ERR "BRD %d interbus_ioctl INTERBUS_IOCX_IO_READ 0x%04X EINVAL !\n", dev->minor, reg.addr);
//            mutex_unlock(&file->f_dentry->d_inode->i_mutex);
            return -EINVAL;
         }
         reg.value = interbus_read_io(dev, reg.addr);
         dprintk("BRD %d interbus_ioctl INTERBUS_IOCX_IO_READ reg:0x%04X val:0x%02X\n", dev->minor, reg.addr, reg.value);
         if (copy_to_user((void __user *)arg, &reg, sizeof(reg)))
            return -EFAULT;
         break;

      case INTERBUS_IOCS_IO_WRITE:
         if(copy_from_user(&reg, (void __user *)arg, sizeof(reg)))
            return -EFAULT;
         if ( reg.addr >= dev->iosize )
         {
            printk(KERN_ERR "BRD %d interbus_ioctl INTERBUS_IOCS_IO_WRITE 0x%04X EINVAL !\n", dev->minor, reg.addr);
//            mutex_unlock(&file->f_dentry->d_inode->i_mutex);
            return -EINVAL;
         }
         dprintk("BRD %d interbus_ioctl INTERBUS_IOCS_IO_WRITE reg:0x%04X val:0x%02X\n", dev->minor, reg.addr, reg.value);
         interbus_write_io(dev, reg.addr, reg.value);
         break;

      case INTERBUS_IOCQ_ISR_HND:
         if (copy_from_user(&isr_hnd, (void __user *)arg, sizeof(isr_hnd)))
            return -EFAULT;
//         dprintk("BRD %d interbus_ioctl INTERBUS_IOCQ_ISR_HND reg:0x%02X flag:0x%02X timeout:%dms\n", dev->isr_flag, dev->minor, isr_hnd.flag, isr_hnd.timeout );
         if (isr_hnd.timeout != -1)
         {
            if ( 0 >= wait_event_interruptible_timeout(dev->wait_q_isr,
                                                      (dev->isr_flag & isr_hnd.flag) != 0,
                                                      msecs_to_jiffies(isr_hnd.timeout)) )
            {
               retval = -ERESTARTSYS;
               break;
            }
         }
         else
         {
            if ( 0 != wait_event_interruptible(dev->wait_q_isr,
                                              (dev->isr_flag & isr_hnd.flag) != 0) )
            {
               retval = -ERESTARTSYS;
               break;
            }
         }
         retval = dev->isr_flag & isr_hnd.flag;
         break;

      case INTERBUS_IOCS_ISR_CLEAR_FLAG:
         if (copy_from_user(&isr_flag, (void __user *)arg, sizeof(isr_flag)))
            return -EFAULT;
         dprintk("BRD %d interbus_ioctl INTERBUS_IOCS_ISR_CLEAR_FLAG flag:0x%X to clear:0x%X\n", dev->minor, dev->isr_flag, isr_flag);
         dev->isr_flag &= ~isr_flag;
         break;

      default: /* redundant, as cmd was checked against */
         printk(KERN_ERR "BRD %d interbus_ioctl INTERBUS_IO_... ENOTTY !\n", dev->minor);
         retval = -ENOTTY;
         break;
   }

//   mutex_unlock(&file->f_dentry->d_inode->i_mutex);
   return retval;
}

/**
 * Open an interbus device
 *
 * @param inode Pointer to the inode
 * @param file The file descriptor
 * @return On success zero is returned.
 **/
static int interbus_open(struct inode *inode,
                         struct file *file)
{
   struct interbus_dev *dev;

   /*  Obtain the device */
   dev = container_of(inode->i_cdev, struct interbus_dev, cdev);
   if( !dev )
   {
      return -ENXIO;
   }

   /* Increment the use count. */
   if( !try_module_get(THIS_MODULE) )
   {
      return -ENODEV;
   }

   file->private_data = dev;

   printk(KERN_INFO "BRD %d interbus_open\n", dev->minor);

   return 0;
}

/**
 * Release an interbus device
 *
 * @param inode Pointer to the inode
 * @param file The file descriptor
 * @return On success zero is returned.
 **/
static int interbus_release(struct inode *inode,
                            struct file *file)
{
   struct interbus_dev *dev = file->private_data;

   printk(KERN_INFO "BRD %d interbus_release\n", dev->minor);

   /*file->private_data = NULL;*/

   /* Decrement the use count. */
   module_put(THIS_MODULE);

   return 0;
}


/*
 * Define which file operations are supported
 */
struct file_operations interbus_fops = {
   .owner            = THIS_MODULE,
   .read             = interbus_read,
   .write            = interbus_write,
   .llseek           = interbus_llseek,
   .unlocked_ioctl   = interbus_ioctl,
   .open             = interbus_open,
   .release          = interbus_release,
};

/**
 * Interbus interrupt service routine
 *
 * @param irq TODO
 * @param handle TODO
 * @return IRQ_HANDLED if treated or IRQ_NONE if none
 **/
static irqreturn_t interbus_isr(int irq,
                                void* handle)
{
   irqreturn_t ret = IRQ_NONE;
   u16 status16;
   u16 handshake16;
   u16 data16;
   unsigned char data8;
   struct interbus_dev* dev = handle;

   dprintk( "BRD %d ----------------------- INTERRUPT ---------------------- Flag:0x%X\n", dev->minor, dev->isr_flag);

   /**
    * TODO - How to identify our interrupts
    * D7:Master1, D6:Master2
    * The board has initated the interrupt
    */
   data8 = interbus_read_io(dev, IO_IRQ_CONTROL_HOST);
   dprintk("BRD %d INTERRUPT IO_IRQ_CONTROL_HOST(0x02):0x%02X", dev->minor, data8);
   if ( (data8 & 0xF0 /*(BB_CTL_R_IRQ_MASTER1 | BB_CTL_R_IRQ_MASTER2) don't work !!! */) == 0 )
   {
      // The interrupt might be shared and thus we might see interrupts
      // from other devices. When that happens, remove this statement.
      // For now it helps checking the hardware. 
      dprintk("BRD %d INTERRUPT NOT OUR INTERRUPT !!!", dev->minor);

      return IRQ_NONE;
   }

   /* 1 - Interrupt SysFail */
   status16 = interbus_read_mem_16(dev, MPM_R_STATUS_REG_1);
   dprintk( "BRD %d INTERRUPT MPM_R_STATUS_REG_1(0x3FB0):0x%04X", dev->minor, status16);
   if( status16 & STATUS_SYSFAIL_0 )
   {
      interbus_write_mem_16(dev, MPM_W_CLEAR_STATUS_BIT_0, MB_CLEAR_STATUS_SYSFAIL);
      dprintk("BRD %d INTERRUPT SYSFAIL", dev->minor);

      dev->isr_flag |= INTERBUS_ISR_SYSFAIL_FLAG;

      ret = IRQ_HANDLED;
   }

   /* 2 - Interrupt Cycle */
   handshake16 = interbus_read_mem_16( dev, MPM_R_HANDSHAKE_REG_A );
   dprintk("BRD %d INTERRUPT MPM_R_HANDSHAKE_REG_A(0x3FC0):0x%04X", dev->minor, handshake16);
   /* synchronisation from controller */
   if( handshake16 & 0x0001 )
   {
      /* Acknowledgment of Handshake Bit A0 (writing of 0 to register) */
      interbus_write_mem_16(dev, MPM_W_SET_HS_A_0, 0);

      /* Data Cycle State high : Start Ready */
      if( interbus_read_mem_16( dev, MPM_RW_STATE_REG ) & MB_STATE_REG_DATA_CYCLE)
      {
         dprintk("BRD %d INTERRUPT INTERRUPT CYCLE START READY", dev->minor);

         dev->isr_flag |= INTERBUS_ISR_CYCLE_START_FLAG;

         /* Check master status */
         data16 = interbus_read_mem_16( dev, MPM_RW_DIAG_REG );
         if( (data16 & 0x00E0) == 0x00E0)
         {
            dprintk("BRD %d INTERRUPT START CYCLE", dev->minor);

            /* Master status ready : Start Cycle */
            data16 = interbus_read_mem_16( dev, MPM_RW_CTRL_REG );
            data16 |= MB_CTRL_REG_DATA_CYCLE_ACTIVATE;
            interbus_write_mem_16(dev, MPM_RW_CTRL_REG, data16);

            interbus_write_mem_16(dev, MPM_W_SET_SYNC_REQ, 0x2800);
         }
      }
      else
      /* Data Cycle State low : Cycle End */
      {
         dprintk("BRD %d INTERRUPT CYCLE END", dev->minor);

         dev->isr_flag |= INTERBUS_ISR_CYCLE_END_FLAG;
      }

      ret = IRQ_HANDLED;
   }

   /* 3 - Interrupt_Mailbox */
   /* Send message confirmation */
   if( handshake16 & 0x0020 )
   {
      /* Node 1 received our mail */
      /* set "Mailbox free detected" from MPM accessor 0 to MPM accessor 1 */
      interbus_write_mem_16(dev, MPM_W_SET_HS_A_5, 0);
      dprintk("BRD %d INTERRUPT SEND MSG ACK", dev->minor);

      dev->isr_flag |= INTERBUS_ISR_MSG_SND_FLAG;
      ret = IRQ_HANDLED;
   }
   /* Message reception */
   if ( handshake16 & 0x0002 )
   {
      interbus_write_mem_16(dev, MPM_W_SET_HS_A_1, 0);
      dprintk("BRD %d INTERRUPT RCV MSG", dev->minor);

      dev->isr_flag |= INTERBUS_ISR_MSG_RCV_FLAG;
      ret = IRQ_HANDLED;
   }

   /* Mean that a node has received a signal - not used */
   data16 = interbus_read_mem_16( dev, MPM_R_STATUS_NODE_SG_INT_REG );
   dprintk("BRD %d INTERRUPT MPM_R_STATUS_NODE_SG_INT_REG(0x3FB4):0x%04X", dev->minor, data16);
   /* Node-SG-Int-x-y */
   if( data16 != 0 )
   {
       dprintk("BRD %d INTERRUPT Node-SG-Int-X", dev->minor);
      /* erase all interrupt signals */
      interbus_write_mem_16(dev, MPM_W_SET_MPM_NODE_SG_INT_0, 0);
      interbus_write_mem_16(dev, MPM_W_SET_MPM_NODE_SG_INT_1, 0);
      interbus_write_mem_16(dev, MPM_W_SET_MPM_NODE_SG_INT_2, 0);
      interbus_write_mem_16(dev, MPM_W_SET_MPM_NODE_SG_INT_3, 0);

      dev->isr_flag |= INTERBUS_ISR_NODE_SIGNAL_FLAG;
      ret = IRQ_HANDLED;
   }

   dprintk( "BRD %d --------------------------------------------------------\n", dev->minor );

   if ( ret == IRQ_HANDLED )
   {
      wake_up_interruptible(&dev->wait_q_isr);
   }

   return ret;
}

/**
 * Probe an interbus PCI device.
 *
 * @param pci_dev The PCI device
 * @param id ID the kernel will use to associate devices to this driver.
 * @return On success zero is returned.
 **/
static int interbus_pci_probe(struct pci_dev* pci_dev,
                                        const struct pci_device_id* id)
{
   int err;
   struct interbus_dev *dev;

   printk(KERN_INFO "Phoenix PCI Interbus controller board \"IBS PCI SC/I-T\" found\n");

   dev = kzalloc(sizeof(struct interbus_dev), GFP_KERNEL);
   if (!dev) {
      printk(KERN_ERR "interbus_pci_probe - kzalloc failed !\n");
      return -ENOMEM;
   }

   if (pci_enable_device(pci_dev)) {
      printk(KERN_ERR "interbus_pci_probe - pci_enable_device failed !\n");
      goto out_free;
   }

   if (pci_request_regions(pci_dev, DEVICE_NAME)) {
      printk(KERN_ERR "interbus_pci_probe - pci_request_regions failed !\n");
      goto out_disable;
   }

   printk(KERN_INFO "BAR0 flag:0x%08X\n", (unsigned int)pci_resource_flags(pci_dev, 0) );
   printk(KERN_INFO "BAR1 flag:0x%08X\n", (unsigned int)pci_resource_flags(pci_dev, 1) );

   /* PCI BAR 0 (I/O port) */
   if (!(pci_resource_flags(pci_dev, 0) & IORESOURCE_IO)) {
      printk(KERN_ERR "interbus_pci_probe - pci_resource_flags PCI BAR 0 (I/O port) failed !\n");
      goto out_regions;
   }
   dev->iobase = pci_resource_start(pci_dev, 0);
   dev->iosize = pci_resource_len(pci_dev, 0);

   /* PCI BAR 1 (I/O mem)*/
   if (!(pci_resource_flags(pci_dev, 1) & IORESOURCE_MEM)) {
      printk(KERN_ERR "interbus_pci_probe - pci_resource_flags PCI BAR 1 (I/O mem) failed !\n");
      goto out_regions;
   }
   dev->membase = pci_resource_start(pci_dev, 1);
   dev->memsize = pci_resource_len(pci_dev, 1);
   dev->membase_mapped = ioremap(dev->membase, dev->memsize);
   if (!dev->membase_mapped) {
      printk(KERN_ERR "interbus_pci_probe - ioremap failed !\n");
      goto out_regions;
   }

   /* pci_write_config_word */
   pci_set_master(pci_dev);
   pci_write_config_word(pci_dev, PCI_COMMAND,
                                  PCI_COMMAND_IO | PCI_COMMAND_MEMORY);

   dev->buffer = kzalloc(dev->memsize, GFP_KERNEL);
   if (!dev->buffer) {
      printk(KERN_ERR "interbus_pci_probe - kzalloc failed !\n");
      goto out_unmap;
   }

   /* obtain minor number */
   down(&ibs_drv->devCountMutex);
   if (ibs_drv->devCount >= interbus_devs) {
      up(&ibs_drv->devCountMutex);
      printk(KERN_ERR "interbus_pci_probe - too many board !\n");
      goto out_unmap;
   }
   dev->minor = ibs_drv->devCount;
   ibs_drv->devCount++;
   up(&ibs_drv->devCountMutex);

   /* Store the IRQ */
   dev->irq = pci_dev->irq;
   dev->isr = interbus_isr;

   /* Char device registration */
   cdev_init(&dev->cdev, &interbus_fops);
   dev->cdev.owner = THIS_MODULE;
   dev->cdev.ops = &interbus_fops;
   err = cdev_add (&dev->cdev, MKDEV(interbus_major, dev->minor), 1);
   if (err) {
      printk(KERN_ERR "interbus_pci_probe - cdev_add failed !\n");
      goto out_unmap;
   }

   dev->isr_flag = INTERBUS_ISR_NO_FLAG;
   init_waitqueue_head(&dev->wait_q_isr);

   /* Enable interrupt */
   if( -1 == interbus_enable_interrupt(dev) )
   {
      printk(KERN_ERR "interbus_pci_probe - can't enable interrupt\n");
      goto out_cdev;
   }

   /* Create device in /dev */
   if (IS_ERR(device_create(ibs_class, &pci_dev->dev,
             MKDEV(interbus_major, dev->minor), NULL,
             "ibs%u", dev->minor))) {
      printk(KERN_ERR "interbus_pci_probe - can't create device\n");
      goto out_interrupt;
   }

   /* Info */
   dprintk(" Card number: %d\n",
         dev->minor );
   dprintk(" I/O ports at: 0x%lx[size=0x%lx]\n",
         dev->iobase,
         dev->iosize );
   dprintk(" Memory at: 0x%lx[size=0x%lx] mapped at: 0x%px\n",
         dev->membase,
         dev->memsize,
         dev->membase_mapped );
   dprintk(" IRQ: 0x%x\n",
         dev->irq );

   pci_set_drvdata(pci_dev, dev);

   return 0; /* succeed */

out_interrupt:
   interbus_disable_interrupt(dev);
out_cdev:
   cdev_del(&dev->cdev);
out_unmap:
   iounmap(dev->membase_mapped);
out_regions:
   pci_release_regions(pci_dev);
out_disable:
   pci_disable_device(pci_dev);
out_free:
   kfree(dev);
   return -ENODEV;
}

/**
 * Remove an interbus PCI device.
 *
 * @param pci_dev The PCI device
 **/
static void interbus_pci_remove (struct pci_dev* pci_dev)
{
   struct interbus_dev* dev = pci_get_drvdata(pci_dev);

   interbus_disable_interrupt(dev);

   device_destroy(ibs_class, MKDEV(interbus_major, dev->minor));

   cdev_del(&dev->cdev);

   down(&ibs_drv->devCountMutex);
   ibs_drv->devCount--;
   up(&ibs_drv->devCountMutex);


   kfree(dev->buffer);
   pci_clear_master(pci_dev);
   pci_release_regions(pci_dev);
   pci_disable_device(pci_dev);
   pci_set_drvdata(pci_dev, NULL);
   iounmap(dev->membase_mapped);
   kfree(dev);
}


#ifdef CONFIG_PM
/**
 * Suspend an interbus PCI device.
 *
 * @param pci_dev The PCI device
 * @param state Power management state
 * @return On success zero is returned.
 **/
static int interbus_pci_suspend(struct pci_dev* pci_dev,
                                pm_message_t state)
{
   pci_save_state(pci_dev);
   pci_disable_device(pci_dev);
   pci_set_power_state(pci_dev, PCI_D3hot);

   return 0;
}

/**
 * Resume an interbus PCI device.
 *
 * @param pci_dev The PCI device
 * @return On success zero is returned.
 **/
static int interbus_pci_resume(struct pci_dev* pci_dev)
{
   int err;

   pci_restore_state(pci_dev);
   err = pci_enable_device(pci_dev);
   if (err)
      return err;

   return 0;
}
#endif



#define PCI_VENDOR_ID_PHOENIX  0x1442
#define PCI_DEVICE_ID_INTERBUS 0x0002

static struct pci_device_id interbus_pci_ids[ ] = {
   { PCI_DEVICE(PCI_VENDOR_ID_PHOENIX, PCI_DEVICE_ID_INTERBUS) },
   { 0, },
   };
MODULE_DEVICE_TABLE(pci, interbus_pci_ids);


static struct pci_driver interbus_pci_driver = {
   .name       = DEVICE_NAME,
   .id_table   = interbus_pci_ids,
   .probe      = interbus_pci_probe,
   .remove     = interbus_pci_remove,
#ifdef CONFIG_PM
   .resume     = interbus_pci_resume,
   .suspend    = interbus_pci_suspend,
#endif
   };

/**
 * This function is called when the interbus module is loaded
 *
 * @return On success zero is returned.
 **/
static int __init interbus_init(void)
{
   int retval;
   dev_t dev = MKDEV(interbus_major, 0);

   /* Register device in sysfs */
   ibs_class = class_create(THIS_MODULE, "ibs");
   if (IS_ERR(ibs_class)) {
      retval = PTR_ERR(ibs_class);
      printk(KERN_ERR "interbus_init - can't register ibs class\n");
      goto err;
   }

   /*
    * Register major, and accept a dynamic number.
    */
   if (interbus_major) {
      retval = register_chrdev_region(dev, interbus_devs, DEVICE_NAME);
   }
   else {
      retval = alloc_chrdev_region(&dev, 0, interbus_devs, DEVICE_NAME);
      interbus_major = MAJOR(dev);
   }
   if (retval < 0) {
      printk(KERN_ERR "interbus_init - can't register chrdev region\n");
      goto err_class;
   }

   ibs_drv = kzalloc(sizeof(struct interbus_drv), GFP_KERNEL);
   if (!ibs_drv) {
      printk(KERN_ERR "interbus_init - can't allocate memory\n");
      retval = -ENOMEM;
      goto err_unchr;
   }

   sema_init(&ibs_drv->devCountMutex, 1);
   ibs_drv->devCount = 0;

	return pci_register_driver(&interbus_pci_driver);

err_unchr:
   unregister_chrdev_region(dev, interbus_devs);
err_class:
   class_destroy(ibs_class);
err:
   return retval;
}

/**
 * This function is called when the interbus module is unloaded
 **/
static void __exit interbus_exit(void)
{
   pci_unregister_driver(&interbus_pci_driver);

   kfree(ibs_drv);

   unregister_chrdev_region(MKDEV (interbus_major, 0), interbus_devs);

   class_destroy(ibs_class);
}

module_init(interbus_init);
module_exit(interbus_exit);

MODULE_DESCRIPTION("Driver for Phoenix Contact GmbH & Co. PCI Interbus controller boards");
MODULE_AUTHOR("Pierre Quelin <pierre.quelin.1972@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_PARM_DESC(interbus_major, "Dynamic major number");
MODULE_PARM_DESC(interbus_debug, "Turn on/off debugging traces (default:off)");
MODULE_PARM_DESC(interbus_fake,  "Turn on/off the fake pci implementation (to test on Virtual Machine) (default:off)");
