/*
################################################################################
#
# r8168 is the Linux device driver released for Realtek Gigabit Ethernet
# controllers with PCI-Express interface.
#
# Copyright(c) 2014-2018, Realtek Semiconductor Corp. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>.
#
# Author:
# Realtek NIC software team <nicfae@realtek.com>
# No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
#
################################################################################
*/

/************************************************************************************
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 ***********************************************************************************/

#ifndef _PG_DRV_H
#define _PG_DRV_H

#include <linux/cdev.h>

#define BYTE	__u8
#define WORD	__u16
#define DWORD	__u32
#define DWORD64	__u64
#define BOOLEAN	int
typedef void *	PVOID;
typedef BYTE *	PBYTE;
typedef WORD *	PWORD;
typedef DWORD *	PDWORD;

#ifndef TRUE
	#define TRUE	1
	#define FALSE	0
#endif

#define PRINT_LEVEL					KERN_NOTICE

#ifdef DEBUG
#define DebugPrint(fmt,args...)				printk(PRINT_LEVEL "[RTNICPG]" fmt "\n",## args)
#define DbgFunPrint(fmt,args...)			printk(PRINT_LEVEL "[RTNICPG]" "%s %i: " fmt "\n",__FUNCTION__,__LINE__,## args)
#else
#define DebugPrint(fmt,args...)				//printk(PRINT_LEVEL fmt "\n",## args)
#define DbgFunPrint(fmt,args...)			//printk(PRINT_LEVEL "%s %i: " fmt "\n",__FUNCTION__,__LINE__,## args)
#endif

/*******************************************************************************
*******************************************************************************/
#define Writel(Address,Data)			writel(Data,(void *)(Address))
#define Readl(Address)				readl((void *)(Address))
#define Writew(Address,Data)			writew(Data,(void *)(Address))
#define Readw(Address)				readw((void *)(Address))
#define Writeb(Address,Data)			writeb(Data,(void *)(Address))
#define Readb(Address)				readb((void *)(Address))

#define PGNAME					"pgtool"
#define MAX_DEV_NUM				10
#define MAX_IO_SIZE				0x100

typedef struct _DEV_INFO_
{
	dev_t					devno;
	bool					bUsed;
}DEV_INFO,*PDEV_INFO;

typedef struct _PG_DEV_
{
	struct cdev				cdev;
	struct pci_dev				*pdev;
	const struct pci_device_id		*id;
	unsigned long				base_phyaddr;
	unsigned int				offset;
	unsigned int				deviceID;
	atomic_t				count;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	struct semaphore			dev_sem;
#else
	struct mutex				dev_mutex;
#endif
	unsigned int				index;
}PGDEV,*PPGDEV;

typedef struct _PCI_CONFIG_RW_
{
	union
	{
		unsigned char	byte;
		unsigned short	word;
		unsigned int	dword;
	};
	unsigned int		bRead:1;
	unsigned int		size:7;
	unsigned int		addr:8;
	unsigned int		reserve:16;
}PCI_CONFIG_RW,*PPCI_CONFIG_RW;

#define RTL_IOC_MAGIC					0x95

#define IOC_PCI_CONFIG					_IOWR(RTL_IOC_MAGIC, 0, PCI_CONFIG_RW)
#define IOC_IOMEM_OFFSET				_IOR(RTL_IOC_MAGIC, 1, unsigned int)
#define IOC_DEV_FUN					_IOR(RTL_IOC_MAGIC, 2, unsigned int)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
#define __devinit
#define __devexit
#define __devexit_p(func)   func
#endif

int __init pgdrv_init(void);
void __exit pgdrv_exit(void);
int __devinit pgdrv_prob(struct pci_dev *pdev, const struct pci_device_id *id);
void __devexit pgdrv_remove(struct pci_dev *pdev);

#endif // end of #ifndef _PG_DRV_H
