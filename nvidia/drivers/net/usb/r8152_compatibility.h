#ifndef LINUX_COMPATIBILITY_H
#define LINUX_COMPATIBILITY_H

/*
 * Definition and macro
 */

#include <linux/init.h>
#include <linux/version.h>
#include <linux/in.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30)
#include <linux/mdio.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
	#define ether_addr_copy(dst, src)		memcpy(dst, src, ETH_ALEN)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	#define NETIF_F_HW_VLAN_CTAG_RX			NETIF_F_HW_VLAN_RX
	#define NETIF_F_HW_VLAN_CTAG_TX			NETIF_F_HW_VLAN_TX
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	static inline __sum16 tcp_v6_check(int len,
					   const struct in6_addr *saddr,
					   const struct in6_addr *daddr,
					   __wsum base)
	{
		return csum_ipv6_magic(saddr, daddr, len, IPPROTO_TCP, base);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
	#define eth_random_addr(addr)			random_ether_addr(addr)
	#define MDIO_EEE_100TX				0x0002	/* 100TX EEE cap */
	#define MDIO_EEE_1000T				0x0004	/* 1000T EEE cap */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
	#define ETH_MDIO_SUPPORTS_C22			MDIO_SUPPORTS_C22

	static inline void eth_hw_addr_random(struct net_device *dev)
	{
		random_ether_addr(dev->dev_addr);
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
	#define module_usb_driver(__driver) \
	static int __init __driver##_init(void) \
	{ \
		return usb_register(&(__driver)); \
	} \
	module_init(__driver##_init); \
	static void __exit __driver##_exit(void) \
	{ \
		usb_deregister(&(__driver)); \
	} \
	module_exit(__driver##_exit);

	#define netdev_features_t			u32
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
	#define PMSG_IS_AUTO(msg)	(((msg).event & PM_EVENT_AUTO) != 0)

	static inline struct page *skb_frag_page(const skb_frag_t *frag)
	{
		return frag->page;
	}

	static inline void *skb_frag_address(const skb_frag_t *frag)
	{
		return page_address(skb_frag_page(frag)) + frag->page_offset;
	}

	static inline unsigned int skb_frag_size(const skb_frag_t *frag)
	{
		return frag->size;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
	#define ndo_set_rx_mode				ndo_set_multicast_list
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
	#define NETIF_F_RXCSUM				(1 << 29) /* Receive checksumming offload */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
	#define skb_checksum_none_assert(skb_ptr)	(skb_ptr)->ip_summed = CHECKSUM_NONE
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	static inline void usleep_range(unsigned long min, unsigned long max)
	{
		unsigned long ms = min / 1000;

		if (ms)
			mdelay(ms);

		udelay(min % 1000);
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	static inline bool pci_dev_run_wake(struct pci_dev *dev)
	{
		return 1;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
	#define netdev_mc_count(netdev)			((netdev)->mc_count)
	#define netdev_mc_empty(netdev)			(netdev_mc_count(netdev) == 0)

	#define netif_printk(priv, type, level, netdev, fmt, args...)	\
	do {								\
		if (netif_msg_##type(priv))				\
			printk(level "%s: " fmt,(netdev)->name , ##args); \
	} while (0)

	#define netif_emerg(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_EMERG, netdev, fmt, ##args)
	#define netif_alert(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_ALERT, netdev, fmt, ##args)
	#define netif_crit(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_CRIT, netdev, fmt, ##args)
	#define netif_err(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_ERR, netdev, fmt, ##args)
	#define netif_warn(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_WARNING, netdev, fmt, ##args)
	#define netif_notice(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_NOTICE, netdev, fmt, ##args)
	#define netif_info(priv, type, netdev, fmt, args...)		\
		netif_printk(priv, type, KERN_INFO, (netdev), fmt, ##args)

	static inline int usb_enable_autosuspend(struct usb_device *udev)
	{ return 0; }
	static inline int usb_disable_autosuspend(struct usb_device *udev)
	{ return 0; }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	#define get_sset_count				get_stats_count
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	#define pm_request_resume(para)
	#define pm_runtime_set_suspended(para)
	#define pm_schedule_suspend(para1, para2)
	#define pm_runtime_get_sync(para)
	#define pm_runtime_put_sync(para)
	#define pm_runtime_put_noidle(para)
	#define pm_runtime_idle(para)
	#define pm_runtime_set_active(para)
	#define pm_runtime_enable(para)
	#define pm_runtime_disable(para)
	typedef int netdev_tx_t;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
	#define USB_SPEED_SUPER		(USB_SPEED_VARIABLE + 1)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
	static inline void usb_autopm_put_interface_async(struct usb_interface *intf)
	{
		struct usb_device *udev = interface_to_usbdev(intf);
		int status = 0;

		if (intf->condition == USB_INTERFACE_UNBOUND) {
			status = -ENODEV;
		} else {
			udev->last_busy = jiffies;
			--intf->pm_usage_cnt;
			if (udev->autosuspend_disabled || udev->autosuspend_delay < 0)
				status = -EPERM;
		}
	}

	static inline int usb_autopm_get_interface_async(struct usb_interface *intf)
	{
		struct usb_device *udev = interface_to_usbdev(intf);
		int status = 0;

		if (intf->condition == USB_INTERFACE_UNBOUND)
			status = -ENODEV;
		else if (udev->autoresume_disabled)
			status = -EPERM;
		else
			++intf->pm_usage_cnt;
		return status;
	}

	static inline int eth_change_mtu(struct net_device *dev, int new_mtu)
	{
		if (new_mtu < 68 || new_mtu > ETH_DATA_LEN)
			return -EINVAL;
		dev->mtu = new_mtu;
		return 0;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
	static inline void __skb_queue_splice(const struct sk_buff_head *list,
					      struct sk_buff *prev,
					      struct sk_buff *next)
	{
		struct sk_buff *first = list->next;
		struct sk_buff *last = list->prev;

		first->prev = prev;
		prev->next = first;

		last->next = next;
		next->prev = last;
	}

	static inline void skb_queue_splice(const struct sk_buff_head *list,
					    struct sk_buff_head *head)
	{
		if (!skb_queue_empty(list)) {
			__skb_queue_splice(list, (struct sk_buff *) head, head->next);
			head->qlen += list->qlen;
		}
	}

	static inline void __skb_queue_head_init(struct sk_buff_head *list)
	{
		list->prev = list->next = (struct sk_buff *)list;
		list->qlen = 0;
	}

	static inline void skb_queue_splice_init(struct sk_buff_head *list,
						 struct sk_buff_head *head)
	{
		if (!skb_queue_empty(list)) {
			__skb_queue_splice(list, (struct sk_buff *) head, head->next);
			head->qlen += list->qlen;
			__skb_queue_head_init(list);
		}
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
	#define PM_EVENT_AUTO		0x0400
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	#define napi_enable(napi_ptr)			netif_poll_enable(tp->netdev)
	#define napi_disable(napi_ptr)			netif_poll_disable(tp->netdev)
	#define napi_schedule(napi_ptr)			netif_rx_schedule(tp->netdev)
	#define napi_complete(napi_ptr)			netif_rx_complete(tp->netdev)
	#define netif_napi_del(napi_ptr)
	#define netif_napi_add(ndev, napi_ptr, function, weight_t) \
		ndev->poll = function; \
		ndev->weight = weight_t;
	typedef unsigned long				uintptr_t;
	#define DMA_BIT_MASK(value) \
		(value < 64 ? ((1ULL << value) - 1) : 0xFFFFFFFFFFFFFFFFULL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
	#define NETIF_F_IPV6_CSUM			16
	#define cancel_delayed_work_sync		cancel_delayed_work

	static inline int skb_cow_head(struct sk_buff *skb, unsigned int headroom)
	{
		int delta = 0;

		if (headroom > skb_headroom(skb))
			delta = headroom - skb_headroom(skb);

		if (delta || skb_header_cloned(skb))
			return pskb_expand_head(skb, ALIGN(delta, NET_SKB_PAD), 0,
						GFP_ATOMIC);
		return 0;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
	#define ip_hdr(skb_ptr)				(skb_ptr)->nh.iph
	#define ipv6hdr(skb_ptr)			(skb_ptr)->nh.ipv6h
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
	#define vlan_group_set_device(vlgrp, vid, value) \
		if (vlgrp) \
			(vlgrp)->vlan_devices[vid] = value;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	#define delayed_work				work_struct
	#define INIT_DELAYED_WORK(a,b)			INIT_WORK(a,b,tp)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	#define CHECKSUM_PARTIAL			CHECKSUM_HW

	static inline void *kmemdup(const void *src, size_t len, gfp_t gfp)
	{
		void *p;

		p = kmalloc_track_caller(len, gfp);
		if (p)
			memcpy(p, src, len);
		return p;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
	#define skb_is_gso(skb_ptr)			skb_shinfo(skb_ptr)->tso_size
	#define netdev_alloc_skb(dev, len)		dev_alloc_skb(len)
	#define IRQF_SHARED				SA_SHIRQ

	static inline struct sk_buff *skb_gso_segment(struct sk_buff *skb, int features)
	{
		return NULL;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#ifndef __LINUX_MUTEX_H
	#define mutex					semaphore
	#define mutex_lock				down
	#define mutex_unlock				up
	#define mutex_trylock				down_trylock
	#define mutex_lock_interruptible		down_interruptible
	#define mutex_init				init_MUTEX
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
	#define ADVERTISED_Pause			(1 << 13)
	#define ADVERTISED_Asym_Pause			(1 << 14)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
	#define skb_header_cloned(skb)			skb_cloned(skb)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0) */

#ifndef FALSE
	#define TRUE	1
	#define FALSE	0
#endif

/*
 * inline function
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
static inline struct sk_buff *netdev_alloc_skb_ip_align(struct net_device *dev,
							unsigned int length)
{
	struct sk_buff *skb = netdev_alloc_skb(dev, length + NET_IP_ALIGN);

	if (NET_IP_ALIGN && skb)
		skb_reserve(skb, NET_IP_ALIGN);
	return skb;
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
static inline void skb_copy_from_linear_data(const struct sk_buff *skb,
					     void *to,
					     const unsigned int len)
{
	memcpy(to, skb->data, len);
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23)
/**
 *  netif_napi_del - remove a napi context
 *  @napi: napi context
 *
 *  netif_napi_del() removes a napi context from the network device napi list
 */
static inline void netif_napi_del(struct napi_struct *napi)
{
#ifdef CONFIG_NETPOLL
        list_del(&napi->dev_list);
#endif
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
	#define skb_tx_timestamp(skb)	(tp->netdev->trans_start = jiffies)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	#define skb_tx_timestamp(skb)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31) */

enum rtl_cmd {
	RTLTOOL_PLA_OCP_READ_DWORD = 0,
	RTLTOOL_PLA_OCP_WRITE_DWORD,
	RTLTOOL_USB_OCP_READ_DWORD,
	RTLTOOL_USB_OCP_WRITE_DWORD,
	RTLTOOL_PLA_OCP_READ,
	RTLTOOL_PLA_OCP_WRITE,
	RTLTOOL_USB_OCP_READ,
	RTLTOOL_USB_OCP_WRITE,
	RTLTOOL_USB_INFO,
	RTL_ENABLE_USB_DIAG,
	RTL_DISABLE_USB_DIAG,

	RTLTOOL_INVALID
};

struct usb_device_info {
	__u16		idVendor;
	__u16		idProduct;
	__u16		bcdDevice;
	__u8		dev_addr[8];
	char		devpath[16];
};

struct rtltool_cmd {
	__u32	cmd;
	__u32	offset;
	__u32	byteen;
	__u32	data;
	void	*buf;
	struct usb_device_info nic_info;
	struct sockaddr ifru_addr;
	struct sockaddr ifru_netmask;
	struct sockaddr ifru_hwaddr;
};

#endif /* LINUX_COMPATIBILITY_H */
