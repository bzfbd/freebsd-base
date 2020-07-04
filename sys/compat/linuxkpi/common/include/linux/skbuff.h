/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 The FreeBSD Foundation
 *
 * This software was developed by Björn Zeeb under sponsorship from
 * the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	_LINUX_SKBUFF_H
#define	_LINUX_SKBUFF_H

#include <linux/page.h>
#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <linux/list.h>

struct skb_frag {
		/* XXX TODO */
};
typedef	struct skb_frag	skb_frag_t;

struct sk_buff_head {
		/* XXX TODO */
	struct sk_buff		*next;
	struct sk_buff		*prev;
};

struct sk_buff {
		/* XXX TODO */
	/* struct sk_buff_head */
	struct sk_buff		*next;
	struct sk_buff		*prev;
	int		csum_offset, csum_start, data_len, ip_summed, protocol;
	int		priority;
	uint8_t		cb[64];	/* ??? I see sizeof() operations so probably an array. */
	uint32_t	len;
	uint8_t		*head;
	uint8_t		*data;
	uint8_t		*tail;
	uint8_t		*end;
};

struct skb_shinfo {
	uint16_t		gso_size;
	uint16_t		nr_frags;
	skb_frag_t		frags[64];		/* XXX TODO */
};

static __inline struct sk_buff *
alloc_skb(size_t size, gfp_t gfp)
{
	struct sk_buff *skb;

	skb = kmalloc(sizeof(*skb) + size, gfp);
	/* XXX TODO */
	memset(skb, '\0', sizeof(*skb));	/* %^#%^ */
	skb->len = size;
	skb->head = (uint8_t *)(skb+1);
	skb->data = (uint8_t *)(skb+1);
	skb->tail = (uint8_t *)(skb+1);
	skb->end = skb->head + skb->len;
	return (skb);
}

static __inline struct sk_buff *
dev_alloc_skb(size_t len)
{
	struct sk_buff *skb;

	skb = alloc_skb(len, GFP_KERNEL);
	/* XXX TODO */
	return (skb);
}

static __inline void
kfree_skb(struct sk_buff *skb)
{
	/* XXX TODO */
	kfree(skb);
	return;
}

static __inline void
dev_kfree_skb(struct sk_buff *skb)
{
	/* XXX TODO */
	kfree_skb(skb);
	return;
}

static __inline void
dev_kfree_skb_any(struct sk_buff *skb)
{
	/* XXX TODO */
	dev_kfree_skb(skb);
	return;
}

static __inline void
dev_kfree_skb_irq(struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

/* XXX BZ review these for terminal condition as Linux "queues" are special. */
#define	skb_list_walk_safe(_q, skb, tmp)				\
	for ((skb) = (_q)->next; (skb) != NULL && ((tmp) = (skb)->next); (skb) = (tmp))

#define	skb_queue_walk(_q, skb)						\
	for ((skb) = (_q)->next; (skb) != NULL; (skb) = (skb)->next)

#define	skb_queue_walk_safe(_q, skb, tmp)				\
	for ((skb) = (_q)->next; (skb) != NULL && ((tmp) = (skb)->next); (skb) = (tmp))

static __inline void
skb_reserve(struct sk_buff *skb, size_t len)
{

	skb->data += len;
	skb->tail += len;
	return;
}

static __inline uint8_t *
skb_push(struct sk_buff *skb, size_t len)
{

	KASSERT(((skb->data - len) >= skb->head), ("%s: skb %p (data %p - "
	    "len %zu) < head %p\n", __func__, skb, skb->data, len, skb->data));
	skb->len  += len;
	skb->data -= len;
	return (skb->data);
}

static __inline uint8_t *
skb_tail_pointer(struct sk_buff *skb)
{

	return (skb->tail);
}

static __inline void *
skb_put(struct sk_buff *skb, size_t len)
{
	void *s;

	KASSERT(((skb->tail + len) <= skb->end), ("%s: skb %p (tail %p + "
	    "len %zu) > end %p\n", __func__, skb, skb->tail, len, skb->end));

	s = skb_tail_pointer(skb);
	skb->tail += len;
	skb->len += len;
	return (s);
}

static __inline void *
skb_put_data(struct sk_buff *skb, const void *buf, size_t len)
{
	void *s;

	s = skb_put(skb, len);
	memcpy(s, buf, len);
	return (s);
}

static __inline bool
skb_queue_empty(struct sk_buff_head *q)
{
	bool empty;

	/* If we point back to ourselves, there's no skb on the list. */
	empty = (q->next == (struct sk_buff *)q);
	return (empty);
}


static __inline struct sk_buff *
skb_copy(struct sk_buff *skb, gfp_t gfp)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
consume_skb(struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline unsigned int
skb_tailroom(struct sk_buff *skb)
{
	/* XXX TODO */
	return (-1);
}

static __inline void
skb_add_rx_frag(struct sk_buff *skb, int x, struct page *page, off_t offset,
    size_t size, unsigned int truesize)
{
	/* XXX TODO */
	return;
}

static __inline uint16_t
skb_checksum(struct sk_buff *skb, int offs, size_t len, int x)
{
	/* XXX TODO */
	return (0xffff);
}

static __inline int
skb_checksum_start_offset(struct sk_buff *skb)
{
	/* XXX TODO */
	return (-1);
}

static __inline dma_addr_t
skb_frag_dma_map(struct device *dev, const skb_frag_t *frag, int x,
    size_t fragsz, enum dma_data_direction dir)
{
	/* XXX TODO */
	return (-1);
}

static __inline size_t
skb_frag_size(const skb_frag_t *frag)
{
	/* XXX TODO */
	return (-1);
}

static __inline size_t
skb_headlen(struct sk_buff *skb)
{
	/* XXX TODO */
	return (-1);
}

static __inline bool
skb_is_nonlinear(struct sk_buff *skb)
{
	/* XXX TODO */
	return (true);
}

static __inline void
skb_pull(struct sk_buff *skb, size_t len)
{
	/* XXX TODO */
	return;
}

#define	skb_walk_frags(_skb, _frag)					\
	for ((_frag) = (_skb); false; (_frag)++)

static __inline void
skb_checksum_help(struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline struct skb_shinfo *
skb_shinfo(struct sk_buff *skb)
{
	/* XXX TODO */
	return (NULL);
}

static __inline bool
skb_ensure_writable(struct sk_buff *skb, size_t off)
{
	/* XXX TODO */
	return (false);
}

static __inline void *
skb_frag_address(const skb_frag_t *frag)
{
	/* XXX TODO */
	return (NULL);
}

static __inline struct sk_buff *
skb_gso_segment(struct sk_buff *skb, netdev_features_t netdev_flags)
{
	/* XXX TODO */
	return (NULL);
}

static __inline bool
skb_is_gso(struct sk_buff *skb)
{
	/* XXX TODO */
	return (false);
}

static __inline void
skb_mark_not_on_list(struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline struct sk_buff *
skb_peek_tail(struct sk_buff_head *frames)
{
	/* XXX TODO */
	return (NULL);
}

static __inline void
skb_queue_splice_init(struct sk_buff_head *qa, struct sk_buff_head *qb)
{
	/* XXX TODO */
	return;
}

static __inline void
skb_reset_transport_header(struct sk_buff *skb)
{
	/* XXX TODO */
	return;
}

static __inline uint8_t *
skb_network_header(struct sk_buff *skb)
{
	/* XXX TODO */
	return (NULL);
}

static __inline uint8_t *
skb_transport_header(struct sk_buff *skb)
{
	/* XXX TODO */
	return (NULL);
}

static __inline int
__skb_linearize(struct sk_buff *skb)
{
	/* XXX TODO */
	return (ENXIO);
}

static __inline void
__skb_queue_purge(struct sk_buff_head *q)
{
	/* XXX TODO */
	return;
}

static __inline void
skb_queue_purge(struct sk_buff_head *q)
{

	return (__skb_queue_purge(q));
}

static __inline void
__skb_queue_head_init(struct sk_buff_head *q)
{

	q->prev = q->next = (struct sk_buff *)q;
	return;
}

static __inline void
skb_queue_head_init(struct sk_buff_head *q)
{

	return (__skb_queue_head_init(q));
}


static __inline void
__skb_queue_tail(struct sk_buff_head *q, struct sk_buff *skb)
{
	struct sk_buff *s;

	s = (struct sk_buff *)q;
	s->prev->next = skb;
	skb->prev = s->prev;
	skb->next = s;
	s->prev = skb;
	return;
}

static __inline void
skb_queue_tail(struct sk_buff_head *q, struct sk_buff *skb)
{

	return (__skb_queue_tail(q, skb));
}

static __inline void
__skb_unlink(struct sk_buff *skb, struct sk_buff_head *head)
{
	struct sk_buff *p, *n;;

	p = skb->prev;
	n = skb->next;
	p->next = n;
	n->prev = p;
	skb->prev = skb->next = NULL;
	return;
}

static __inline void
skb_unlink(struct sk_buff *skb, struct sk_buff_head *head)
{
	return (__skb_unlink(skb, head));
}

static __inline struct sk_buff *
__skb_dequeue(struct sk_buff_head *q)
{
	struct sk_buff *skb;

	skb = q->next;
	if (skb == (struct sk_buff *)q)
		return (NULL);
	if (skb != NULL)
		__skb_unlink(skb, q);
	return (skb);
}

static __inline struct sk_buff *
skb_dequeue(struct sk_buff_head *q)
{

	return (__skb_dequeue(q));
}

static __inline bool
pskb_expand_head(struct sk_buff *skb, int x, int len, gfp_t gfp)
{
	/* XXX TODO */
	return (false);
}

static __inline uint16_t
skb_get_queue_mapping(struct sk_buff *skb)
{
	/* XXX TODO */
	return (-1);
}

static __inline void *
skb_put_zero(struct sk_buff *skb, size_t len)
{
	/* XXX TODO */
	return (NULL);
}

static __inline size_t
skb_queue_len(struct sk_buff_head *head)
{
	/* XXX TODO */
	return (-1);
}

#endif	/* _LINUX_SKBUFF_H */

/* end */
