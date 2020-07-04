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

#ifndef	_LINUX_CPU_H
#define	_LINUX_CPU_H

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/cpuset.h>
#include <linux/compiler.h>
#include <linux/slab.h>

typedef	cpuset_t	cpumask_t;	/* XXX TODO */

extern cpumask_t cpu_online_mask;

static __inline int
cpumask_next(int n, cpumask_t mask)
{
	/* XXX TODO */
	return (-1);
}

static __inline void
cpumask_set_cpu(int cpu, cpumask_t *mask)
{
	/* XXX TODO */
	return;
}

/* XXX-BZ DPCPU? uma_zcreate(,, UMA_ZONE_PCPU)? */
#define	alloc_percpu(_type)						\
	__percpu kmalloc(sizeof(_type) * mp_ncpus, GFP_KERNEL|__GFP_ZERO)	/* XXX FIXME */

static __inline void
free_percpu(__percpu void *p)
{
	/* XXX FIXME */
	kfree(p);
	return;
}

#define	for_each_possible_cpu(i)					\
	for (i = 0; i < mp_ncpus; i++)		/* XXX TODO */

#define	per_cpu_ptr(_p, _cpu)						\
	((__typeof(*(_p)) *)((uintptr_t)(_p) + (_cpu) * sizeof(*(_p))))
	/* XXX KASSERT(_cpu < mp_ncpus) ? */

#define	this_cpu_ptr(_p)						\
	((__typeof(*(_p)) *)((uintptr_t)(_p) + curcpu * sizeof(*(_p))))

#endif	/* _LINUX_CPU_H */

/* end */
