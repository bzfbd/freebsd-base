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

#ifndef _LINUX_AVERAGE_H
#define _LINUX_AVERAGE_H

/* Exponentially Weighted Moving Average (EWMA) */
/*
 * Seems Linux is generating accessor functions with the "name" included
 * so need to make this macros and glue things together.
 */
/* XXX TODO */
#define	DECLARE_EWMA(_name, _n, _l)						\
	struct ewma_ ## _name {							\
	};									\
										\
	static inline void							\
	ewma_ ## _name ## _init(struct ewma_ ## _name *ewma)			\
	{									\
	}									\
										\
	static inline void							\
	ewma_ ## _name ## _add(struct ewma_ ## _name *ewma, uint32_t v)		\
	{									\
	}									\
										\
	static inline unsigned long						\
	ewma_ ## _name ## _read(struct ewma_ ## _name *ewma)			\
	{									\
		return (0);							\
	}									\

#endif /* _LINUX_AVERAGE_H */

/* end */
