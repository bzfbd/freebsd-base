/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2019 Mellanox Technologies, Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */
#ifndef	_LINUX_NETDEVICE_H_
#define	_LINUX_NETDEVICE_H_

#include <linux/types.h>

#include <sys/socket.h>

#include <net/if_types.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_dl.h>

#include <linux/list.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/net.h>
#include <linux/notifier.h>

#ifdef VIMAGE
#define	init_net *vnet0
#else
#define	init_net *((struct vnet *)0)
#endif

#define	MAX_ADDR_LEN		20

/* XXX FIXME, this does not work. */
#define	net_device	ifnet

struct netdev_hw_addr_list {
	struct list_head	addr_list;
	int			count;
};

struct netdev_hw_addr {
	struct list_head	addr_list;
	uint8_t			addr[6];	/* XXX FIXME */
};

struct napi_struct {
		/* XXX TODO */
	int			poll, rx_count;
	struct list_head	rx_list;
};

typedef	uint32_t	netdev_features_t;	/* XXX TODO */
#define	NETIF_F_CSUM_MASK	__LINE__ /* XXX TODO */
#define	NETIF_F_HIGHDMA		__LINE__ /* XXX TODO */
#define	NETIF_F_SG		__LINE__ /* XXX TODO */
#define	NETIF_F_IPV6_CSUM	__LINE__ /* XXX TODO */
#define	NETIF_F_IP_CSUM		__LINE__ /* XXX TODO */
#define	NETIF_F_TSO		__LINE__ /* XXX TODO */
#define	NETIF_F_TSO6		__LINE__ /* XXX TODO */
#define	NETIF_F_RXCSUM		__LINE__ /* XXX TODO */

static inline struct ifnet *
dev_get_by_index(struct vnet *vnet, int if_index)
{
	struct epoch_tracker et;
	struct ifnet *retval;

	NET_EPOCH_ENTER(et);
	CURVNET_SET(vnet);
	retval = ifnet_byindex_ref(if_index);
	CURVNET_RESTORE();
	NET_EPOCH_EXIT(et);

	return (retval);
}

#define	dev_hold(d)	if_ref(d)
#define	dev_put(d)	if_rele(d)
#define	dev_net(d)	((d)->if_vnet)

#define	net_eq(a,b)	((a) == (b))

#define	netif_running(dev)	!!((dev)->if_drv_flags & IFF_DRV_RUNNING)
#define	netif_oper_up(dev)	!!((dev)->if_flags & IFF_UP)
#define	netif_carrier_ok(dev)	((dev)->if_link_state == LINK_STATE_UP)

static inline void *
netdev_priv(const struct net_device *dev)
{
	return (dev->if_softc);
}

static inline struct net_device *
netdev_notifier_info_to_dev(void *ifp)
{
	return (ifp);
}

static inline void
netdev_rss_key_fill(uint32_t *buf, size_t len)
{
	/* XXX TODO */
	return;
}

int	register_netdevice_notifier(struct notifier_block *);
int	register_inetaddr_notifier(struct notifier_block *);
int	unregister_netdevice_notifier(struct notifier_block *);
int	unregister_inetaddr_notifier(struct notifier_block *);

#define	rtnl_lock()
#define	rtnl_unlock()

static inline int
dev_mc_delete(struct net_device *dev, void *addr, int alen, int all)
{
	struct sockaddr_dl sdl;

	if (alen > sizeof(sdl.sdl_data))
		return (-EINVAL);
	memset(&sdl, 0, sizeof(sdl));
	sdl.sdl_len = sizeof(sdl);
	sdl.sdl_family = AF_LINK;
	sdl.sdl_alen = alen;
	memcpy(&sdl.sdl_data, addr, alen);

	return -if_delmulti(dev, (struct sockaddr *)&sdl);
}

static inline int
dev_mc_del(struct net_device *dev, void *addr)
{
	return (dev_mc_delete(dev, addr, 6, 0));
}

static inline int
dev_mc_add(struct net_device *dev, void *addr, int alen, int newonly)
{
	struct sockaddr_dl sdl;

	if (alen > sizeof(sdl.sdl_data))
		return (-EINVAL);
	memset(&sdl, 0, sizeof(sdl));
	sdl.sdl_len = sizeof(sdl);
	sdl.sdl_family = AF_LINK;
	sdl.sdl_alen = alen;
	memcpy(&sdl.sdl_data, addr, alen);

	return -if_addmulti(dev, (struct sockaddr *)&sdl, NULL);
}

static __inline void
napi_gro_flush(struct napi_struct *napi, bool t)
{
	/* XXX TODO */
	return;
}

static __inline void
netif_napi_add(struct net_device *ndev, struct napi_struct *napi,
    int(*napi_poll)(struct napi_struct *, int), int x)
{
	/* XXX TODO */
	return;
}

static __inline void
netif_napi_del(struct napi_struct *napi)
{
	/* XXX TODO */
	return;
}

static __inline void
netif_receive_skb_list(struct list_head *head)
{
	/* XXX TODO */
	return;
}

static __inline void
init_dummy_netdev(struct net_device *napi_dev)
{
	/* XXX TODO */
	return;
}

static __inline int
netdev_hw_addr_list_count(struct netdev_hw_addr_list *list)
{

	return (list->count);
}

#define	netdev_hw_addr_list_for_each(_addr, _list)			\
	list_for_each_entry((_addr), &(_list)->addr_list, addr_list)


static __inline void
synchronize_net(void)
{
	/* XXX TODO */
	return;
}

/* XXX probably does not belong here. */
static __inline void
addrconf_addr_solict_mult(struct in6_addr *ia6, struct in6_addr *solia6)
{
	/* XXX TODO */
	return;
}

static __inline void
get_random_mask_addr(uint8_t *sa, const uint8_t *mac, const uint8_t *mask)
{
	/* XXX TODO */
	return;
}


#endif	/* _LINUX_NETDEVICE_H_ */
