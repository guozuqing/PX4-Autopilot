/**
 * @file usbmsc.c
 *
 * Board-specific USB Mass Storage Class initialization.
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>

/**
 * @brief Board-level USB MSC initialization
 *
 * Perform architecture specific initialization of the USB MSC device.
 * SD slot is already initialized in board_app_initialize(), so nothing
 * further is needed here.
 */

int board_usbmsc_initialize(int port)
{
    return OK;
}
