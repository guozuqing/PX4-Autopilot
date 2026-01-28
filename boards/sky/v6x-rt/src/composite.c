/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file composite.c
 *
 * Board-specific USB Composite Device support (CDC/ACM + Mass Storage).
 */

#include <nuttx/config.h>

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

#include <sys/types.h>
#include <stdint.h>
#include <syslog.h>
#include <assert.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>

#ifdef CONFIG_USBMSC_COMPOSITE
static void *g_mschandle;
#endif

#ifdef CONFIG_USBMSC_COMPOSITE
static int board_mscclassobject(int minor,
				struct usbdev_devinfo_s *devinfo,
				struct usbdevclass_driver_s **classdev)
{
	int ret;

	DEBUGASSERT(g_mschandle == NULL);

	syslog(LOG_INFO, "board_mscclassobject: Configuring NLUNS=1\n");
	ret = usbmsc_configure(1, &g_mschandle);

	if (ret < 0) {
		syslog(LOG_ERR, "usbmsc_configure failed: %d\n", -ret);
		return ret;
	}

	syslog(LOG_INFO, "board_mscclassobject: Bind LUN=0 to /dev/mmcsd0\n");
	ret = usbmsc_bindlun(g_mschandle, "/dev/mmcsd0", 0, 0, 0, false);

	if (ret < 0) {
		syslog(LOG_ERR, "usbmsc_bindlun failed: %d\n", -ret);
		usbmsc_uninitialize(g_mschandle);
		g_mschandle = NULL;
		return ret;
	}

	ret = usbmsc_classobject(g_mschandle, devinfo, classdev);

	if (ret < 0) {
		syslog(LOG_ERR, "usbmsc_classobject failed: %d\n", -ret);
		usbmsc_uninitialize(g_mschandle);
		g_mschandle = NULL;
	}

	return ret;
}

static void board_mscuninitialize(struct usbdevclass_driver_s *classdev)
{
	DEBUGASSERT(g_mschandle != NULL);
	usbmsc_uninitialize(g_mschandle);
	g_mschandle = NULL;
}
#endif

int board_composite_initialize(int port)
{
	return OK;
}

void *board_composite_connect(int port, int configid)
{
	if (configid == 0) {
#ifdef CONFIG_USBMSC_COMPOSITE
		struct composite_devdesc_s dev[2];
		int ifnobase = 0;
		int strbase  = COMPOSITE_NSTRIDS;

		cdcacm_get_composite_devdesc(&dev[0]);

		dev[0].classobject  = cdcacm_classobject;
		dev[0].uninitialize = cdcacm_uninitialize;
		dev[0].devinfo.ifnobase = ifnobase;
		dev[0].minor = 0;
		dev[0].devinfo.strbase = strbase;
		dev[0].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 1;
		dev[0].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 2;
		dev[0].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 3;

		ifnobase += dev[0].devinfo.ninterfaces;
		strbase  += dev[0].devinfo.nstrings;

		usbmsc_get_composite_devdesc(&dev[1]);

		dev[1].classobject  = board_mscclassobject;
		dev[1].uninitialize = board_mscuninitialize;
		dev[1].devinfo.ifnobase = ifnobase;
		dev[1].minor = 0;
		dev[1].devinfo.strbase = strbase;
		dev[1].devinfo.epno[USBMSC_EP_BULKOUT_IDX] = 4;
		dev[1].devinfo.epno[USBMSC_EP_BULKIN_IDX]  = 5;

		return composite_initialize(2, dev);
#endif
	}

	return NULL;
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
