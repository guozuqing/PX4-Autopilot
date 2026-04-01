/**
 * @file composite.c
 *
 * Board-specific USB Composite Device support (CDC/ACM + Mass Storage).
 */

#include <nuttx/config.h>

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <syslog.h>
#include <assert.h>
#include <unistd.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <errno.h>

#define MMCSD_DEVPATH   "/dev/mmcsd0"
#define MMCSD_MOUNTPT   "/fs/microsd"
#define MMCSD_WAIT_MS   200
#define MMCSD_RETRIES   15       /* 15 x 200ms = 3s max wait */

#ifdef CONFIG_USBMSC_COMPOSITE
static void *g_mschandle;
#endif

#ifdef CONFIG_USBMSC_COMPOSITE

/**
 * @brief Wait for SD card block device to appear
 */
static bool wait_for_mmcsd(void)
{
    struct stat st;

    for (int i = 0; i < MMCSD_RETRIES; i++) {
        if (stat(MMCSD_DEVPATH, &st) == 0) {
            syslog(LOG_INFO, "composite: %s ready (attempt %d)\n",
                   MMCSD_DEVPATH, i + 1);
            return true;
        }

        usleep(MMCSD_WAIT_MS * 1000);
    }

    syslog(LOG_ERR, "composite: %s not found after %d ms\n",
           MMCSD_DEVPATH, MMCSD_WAIT_MS * MMCSD_RETRIES);
    return false;
}

/**
 * @brief Create MSC class object
 *
 * Called during composite device initialization to configure MSC and bind SD card
 */
static int board_mscclassobject(int minor,
                                struct usbdev_devinfo_s *devinfo,
                                struct usbdevclass_driver_s **classdev)
{
    int ret;

    DEBUGASSERT(g_mschandle == NULL);

    /* Wait for SD card block device */
    if (!wait_for_mmcsd()) {
        return -ENODEV;
    }

    /* Unmount SD card before binding MSC to avoid filesystem conflicts */
    ret = umount(MMCSD_MOUNTPT);
    if (ret < 0) {
        syslog(LOG_WARNING, "composite: umount %s: %d (ok if not mounted)\n",
               MMCSD_MOUNTPT, errno);
    }

    /* Configure MSC with 1 LUN */
    ret = usbmsc_configure(1, &g_mschandle);
    if (ret < 0) {
        syslog(LOG_ERR, "composite: usbmsc_configure failed: %d\n", -ret);
        return ret;
    }

    /* Bind SD card block device to LUN 0 */
    ret = usbmsc_bindlun(g_mschandle, MMCSD_DEVPATH, 0, 0, 0, false);
    if (ret < 0) {
        syslog(LOG_ERR, "composite: usbmsc_bindlun failed: %d\n", -ret);
        usbmsc_uninitialize(g_mschandle);
        g_mschandle = NULL;
        return ret;
    }

    /* Get MSC class object for composite framework */
    ret = usbmsc_classobject(g_mschandle, devinfo, classdev);
    if (ret < 0) {
        syslog(LOG_ERR, "composite: usbmsc_classobject failed: %d\n", -ret);
        usbmsc_uninitialize(g_mschandle);
        g_mschandle = NULL;
    }

    syslog(LOG_INFO, "composite: MSC class object created OK\n");
    return ret;
}

/**
 * @brief Uninitialize MSC class object
 */
static void board_mscuninitialize(struct usbdevclass_driver_s *classdev)
{
    if (g_mschandle != NULL) {
        usbmsc_uninitialize(g_mschandle);
        g_mschandle = NULL;
    }
}
#endif

/**
 * @brief Board-level composite device initialization
 *
 * Called by boardctl(BOARDIOC_USBDEV_CONTROL)
 */
int board_composite_initialize(int port)
{
    return OK;
}

/**
 * @brief Connect composite device
 *
 * Creates and returns composite device handle containing CDC ACM and MSC
 *
 * @param port USB port number
 * @param configid Configuration ID (0 = CDC+MSC)
 * @return Composite device handle, NULL on failure
 */
void *board_composite_connect(int port, int configid)
{
    syslog(LOG_INFO, "board_composite_connect: port=%d configid=%d\n",
           port, configid);

    if (configid == 0) {
#ifdef CONFIG_USBMSC_COMPOSITE
        struct composite_devdesc_s dev[2];
        int ifnobase = 0;
        int strbase  = COMPOSITE_NSTRIDS;

        /* Device 0: CDC ACM */
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

        /* Device 1: MSC */
        usbmsc_get_composite_devdesc(&dev[1]);
        dev[1].classobject  = board_mscclassobject;
        dev[1].uninitialize = board_mscuninitialize;
        dev[1].devinfo.ifnobase = ifnobase;
        dev[1].minor = 0;
        dev[1].devinfo.strbase = strbase;
        dev[1].devinfo.epno[USBMSC_EP_BULKOUT_IDX] = 4;
        dev[1].devinfo.epno[USBMSC_EP_BULKIN_IDX]  = 5;

        void *handle = composite_initialize(2, dev);

        if (handle == NULL) {
            syslog(LOG_ERR, "board_composite_connect: composite_initialize FAILED\n");
        } else {
            syslog(LOG_INFO, "board_composite_connect: composite device ready\n");
        }

        return handle;
#endif
    }

    return NULL;
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
