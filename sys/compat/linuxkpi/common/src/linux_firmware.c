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

#include <sys/types.h>
#include <sys/firmware.h>

#include <linux/types.h>
#include <linux/device.h>

/* Keep in sync with linux/firmware.h header file. */
struct linux_firmware {
	size_t			size;
	const uint8_t		*data;
	/* XXX Does Linux expose anything else? */

	const struct firmware	*fbdfw;
};

int request_firmware_nowait(struct module *, bool, const char *,
    struct device *, gfp_t, void *,
    void(*cont)(const struct linux_firmware *, void *));
int request_firmware(const struct linux_firmware **, const char *, struct device *);
void release_firmware(const struct linux_firmware *);


static int
_linux_request_firmware(const char *fw_name, const struct linux_firmware **fw,
    gfp_t gfp, bool enoentok)
{
	const struct firmware *fbdfw;
	struct linux_firmware *lfw;
	const char *fwimg;

	if (fw == NULL)
		return (-EINVAL);

	lfw = kzalloc(sizeof(*lfw), gfp);
	if (lfw == NULL)
		return (-ENOMEM);

	/*
	 * Linux can have a path in the firmware which is hard to replicate
	 * for auto-firmware-module-loading.
	 */
	fwimg = strrchr(fw_name, '/');
	if (fwimg != NULL)
		fwimg++;
	if (fwimg == NULL || *fwimg == '\0')
		fwimg = fw_name;
	fbdfw = firmware_get(fwimg);
	if (fbdfw == NULL) {
		if (enoentok)
			*fw = lfw;
		else
			kfree(lfw);
		return (-ENOENT);
	}

	lfw->fbdfw = fbdfw;
	lfw->data = (const uint8_t *)fbdfw->data;
	lfw->size = fbdfw->datasize;
	*fw = lfw;
	return (0);
}

int
request_firmware_nowait(struct module *mod __unused, bool _t __unused,
    const char *fw_name, struct device *dev __unused, gfp_t gfp, void *drv,
    void(*cont)(const struct linux_firmware *, void *))
{
	const struct linux_firmware *lfw;
	int error;

	/* Linux seems to run the callback if it cannot find the fimrware. */
	error = _linux_request_firmware(fw_name, &lfw, gfp, true);
	if (error == -ENOENT)
		error = 0;
	/* XXX-BZ we do not run this deferred. It's a compat layer. */
	if (error == 0)
		cont(lfw, drv);

	return (error);
}

int
request_firmware(const struct linux_firmware **fw, const char *fw_name,
    struct device *dev __unused)
{

	return (_linux_request_firmware(fw_name, fw, GFP_KERNEL, false));
}

void
release_firmware(const struct linux_firmware *fw)
{

	if (fw == NULL)
		return;

	if (fw->fbdfw)
		firmware_put(fw->fbdfw, FIRMWARE_UNLOAD);
	kfree(fw);
	return;
}

/* end */
