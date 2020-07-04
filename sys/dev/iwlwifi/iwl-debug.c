/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2011 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2005 - 2011 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#if defined(CONFIG_IWLWIFI_DEBUG)
#include <linux/net.h>
#endif
#include "iwl-drv.h"
#include "iwl-debug.h"
#include "iwl-modparams.h"
#if defined(__linux__)
#include "iwl-devtrace.h"

#elif defined(__FreeBSD__)
#if defined(CONFIG_IWLWIFI_DEBUG)
#include <sys/systm.h>		/* hexdump(9) */
#include <linux/preempt.h>
#endif
#if !defined(CONFIG_IWLWIFI_DEVICE_TRACING)
static void trace_iwlwifi_warn(struct va_format *vaf __unused) { };
static void trace_iwlwifi_info(struct va_format *vaf __unused) { };
static void trace_iwlwifi_crit(struct va_format *vaf __unused) { };
#endif
#endif

#if defined(__linux__)
#define __iwl_fn(fn)						\
void __iwl_ ##fn(struct device *dev, const char *fmt, ...)	\
{								\
	struct va_format vaf = {				\
		.fmt = fmt,					\
	};							\
	va_list args;						\
								\
	va_start(args, fmt);					\
	vaf.va = &args;						\
	dev_ ##fn(dev, "%pV", &vaf);				\
	trace_iwlwifi_ ##fn(&vaf);				\
	va_end(args);						\
}
#elif defined(__FreeBSD__)
#define __iwl_fn(fn)						\
void __iwl_ ##fn(struct device *dev, const char *fmt, ...)	\
{								\
	struct va_format vaf = {				\
		.fmt = fmt,					\
	};							\
	va_list args;						\
	char *str;						\
								\
	va_start(args, fmt);					\
	vaf.va = &args;						\
	vasprintf(&str, M_KMALLOC, fmt, args);			\
	dev_ ##fn(dev, "%s", str);				\
	trace_iwlwifi_ ##fn(&vaf);				\
	free(str, M_KMALLOC);					\
	va_end(args);						\
}
#endif

__iwl_fn(warn)
IWL_EXPORT_SYMBOL(__iwl_warn);
__iwl_fn(info)
IWL_EXPORT_SYMBOL(__iwl_info);
__iwl_fn(crit)
IWL_EXPORT_SYMBOL(__iwl_crit);

void __iwl_err(struct device *dev, bool rfkill_prefix, bool trace_only,
		const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
	if (!trace_only) {
#if defined(__linux_)
		dev_err(dev, "%s%pV", (rfkill_prefix) ? "(RFKILL)" : "", &vaf);
#elif defined(__FreeBSD__)
		char *str;
		vasprintf(&str, M_KMALLOC, fmt, args);
		dev_err(dev, "%s%s", (rfkill_prefix) ? "(RFKILL)" : "", str);
		free(str, M_KMALLOC);
#endif
	}
#if defined(CONFIG_IWLWIFI_DEVICE_TRACING)
	trace_iwlwifi_err(&vaf);
#endif
	va_end(args);
}
IWL_EXPORT_SYMBOL(__iwl_err);

#if defined(CONFIG_IWLWIFI_DEBUG) || defined(CONFIG_IWLWIFI_DEVICE_TRACING)

#ifdef CONFIG_IWLWIFI_DEBUG
bool
iwl_have_debug_level(enum iwl_dl level)
{

	return (iwlwifi_mod_params.debug_level & level || level == IWL_DL_ANY);
}

/* Passing the iwl_drv * in seems pointless. */
void
iwl_print_hex_dump(void *drv __unused, enum iwl_dl level,
    const char *prefix, uint8_t *data, size_t len)
{

	/* Given we have a level, check for it. */
	if (!iwl_have_debug_level(level))
		return;

#if defined(__linux_)
	/* XXX I am cluseless in my editor. pcie/trans.c to the rescue. */
	print_hex_dump(KERN_ERR, prefix, DUMP_PREFIX_OFFSET,
	    32, 4, data, len, 0);
#elif defined(__FreeBSD__)
	hexdump(data, len, prefix, 0);
#endif
}
#endif

void __iwl_dbg(struct device *dev,
	       u32 level, bool limit, const char *function,
	       const char *fmt, ...)
{
	struct va_format vaf = {
		.fmt = fmt,
	};
	va_list args;

	va_start(args, fmt);
	vaf.va = &args;
#if defined(CONFIG_IWLWIFI_DEBUG)
	if (iwl_have_debug_level(level) &&
	    (!limit || net_ratelimit())) {
#if defined(__linux_)
		dev_printk(KERN_DEBUG, dev, "%c %s %pV",
			   in_interrupt() ? 'I' : 'U', function, &vaf);
#elif defined(__FreeBSD__)
		char *str;
		vasprintf(&str, M_KMALLOC, fmt, args);
		dev_printk(KERN_DEBUG, dev, "%c %s %s",
			   in_interrupt() ? 'I' : 'U', function, str);
		free(str, M_KMALLOC);
#endif
	}

#endif
#if defined(CONFIG_IWLWIFI_DEVICE_TRACING)
	trace_iwlwifi_dbg(level, in_interrupt(), function, &vaf);
#endif
	va_end(args);
}
IWL_EXPORT_SYMBOL(__iwl_dbg);
#endif
