/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "SdUmsSwitch.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <errno.h>

#if defined(__PX4_NUTTX)
#include <nuttx/config.h>
#include <sys/boardctl.h>
#endif

using namespace time_literals;

namespace sd_ums_switch
{

extern "C" int board_read_VBUS_state(void);

#if defined(CONFIG_SYSTEM_USBMSC)
extern "C" int msconn_main(int argc, char *argv[]);
extern "C" int msdis_main(int argc, char *argv[]);
#endif

#if defined(CONFIG_SYSTEM_CDCACM)
extern "C" int sercon_main(int argc, char *argv[]);
extern "C" int serdis_main(int argc, char *argv[]);
#endif

SdUmsSwitch::SdUmsSwitch() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

SdUmsSwitch::~SdUmsSwitch()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int SdUmsSwitch::task_spawn(int argc, char *argv[])
{
	SdUmsSwitch *obj = new SdUmsSwitch();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	obj->start();

	return 0;
}

void SdUmsSwitch::start()
{
	// Initialize VBUS state
	_vbus_prev = read_vbus_state();
	_vbus_stable_time = hrt_absolute_time();

	PX4_INFO("sd_ums_switch started, VBUS=%s", _vbus_prev ? "connected" : "disconnected");

	ScheduleOnInterval(100_ms); // 10 Hz polling
}

void SdUmsSwitch::Run()
{
	perf_begin(_cycle_perf);

	bool vbus_now = read_vbus_state();
	hrt_abstime now = hrt_absolute_time();

	// First run: if USB already connected at startup, trigger switch immediately
	if (_first_run) {
		_first_run = false;
		PX4_INFO("First run: state=%s, VBUS=%s",
			_state == State::PX4_SD ? "PX4_SD" : "USB_MSC",
			vbus_now ? "connected" : "disconnected");

		if (vbus_now && _state == State::PX4_SD) {
			PX4_INFO("USB already connected at startup, switching to MSC...");
			if (switch_to_msc()) {
				_state = State::USB_MSC;
				PX4_INFO("Now in USB MSC mode");
			} else {
				PX4_ERR("Failed to switch to MSC mode");
			}
		}
		_vbus_prev = vbus_now;
		perf_end(_cycle_perf);
		return;
	}

	// Debounce logic for subsequent runs
	if (vbus_now != _vbus_prev) {
		_vbus_stable_time = now;
		_vbus_prev = vbus_now;
	}

	bool vbus_stable = (now - _vbus_stable_time) >= DEBOUNCE_TIME_US;

	if (vbus_stable) {
		if (_state == State::PX4_SD && vbus_now) {
			// A -> B: USB plugged in, switch to MSC mode
			PX4_INFO("USB connected, switching to MSC mode...");

			if (switch_to_msc()) {
				_state = State::USB_MSC;
				PX4_INFO("Now in USB MSC mode");
			} else {
				PX4_ERR("Failed to switch to MSC mode");
			}

		} else if (_state == State::USB_MSC && !vbus_now) {
			// B -> A: USB unplugged, switch back to PX4 mode
			PX4_INFO("USB disconnected, switching to PX4 mode...");

			if (switch_to_px4()) {
				_state = State::PX4_SD;
				PX4_INFO("Now in PX4 SD mode");
			} else {
				PX4_ERR("Failed to switch to PX4 mode");
			}
		}
	}

	perf_end(_cycle_perf);
}

bool SdUmsSwitch::read_vbus_state()
{
	// board_read_VBUS_state returns 0 if connected, 1 if not connected
	return board_read_VBUS_state() == 0;
}

bool SdUmsSwitch::switch_to_msc()
{
	// Step 1: Stop logger and other SD writers
	if (!stop_logger()) {
		PX4_WARN("Failed to stop logger");
		// Continue anyway
	}

	// Step 2: Sync filesystem
	sync();
	px4_usleep(100000); // 100ms delay

	// Step 3: Unmount SD card
	if (!unmount_sd()) {
		PX4_ERR("Failed to unmount SD card");
		start_logger(); // Try to recover
		return false;
	}

	// Step 4: Disconnect CDC ACM (if enabled)
#if defined(CONFIG_SYSTEM_CDCACM)
	serdis_main(0, nullptr);
	px4_usleep(50000); // 50ms delay
#endif

	// Step 5: Start USB MSC
	if (!start_usb_msc()) {
		PX4_ERR("Failed to start USB MSC");
		mount_sd(); // Try to recover
		start_logger();
		return false;
	}

	return true;
}

bool SdUmsSwitch::switch_to_px4()
{
	// Step 1: Stop USB MSC
	if (!stop_usb_msc()) {
		PX4_WARN("Failed to stop USB MSC");
		// Continue anyway
	}

	px4_usleep(100000); // 100ms delay

	// Step 2: Re-mount SD card
	if (!mount_sd()) {
		PX4_ERR("Failed to mount SD card");
		return false;
	}

	// Step 3: Reconnect CDC ACM (if enabled)
#if defined(CONFIG_SYSTEM_CDCACM)
	px4_usleep(50000); // 50ms delay
	sercon_main(0, nullptr);
#endif

	// Step 4: Restart logger (optional - user can do manually)
	// if (!start_logger()) {
	//     PX4_WARN("Failed to start logger");
	// }

	return true;
}

bool SdUmsSwitch::stop_logger()
{
	// Use system() to stop logger
	int ret = system("logger stop");
	px4_usleep(200000); // 200ms to allow logger to stop
	return ret == 0;
}

bool SdUmsSwitch::start_logger()
{
	// Use system() to start logger with default parameters
	int ret = system("logger start -e -t");
	return ret == 0;
}

bool SdUmsSwitch::unmount_sd()
{
	int ret = umount("/fs/microsd");

	if (ret != 0) {
		PX4_ERR("umount failed: %d (%s)", errno, strerror(errno));
		return false;
	}

	PX4_INFO("SD card unmounted");
	return true;
}

bool SdUmsSwitch::mount_sd()
{
	int ret = mount("/dev/mmcsd0", "/fs/microsd", "vfat", 0, nullptr);

	if (ret != 0) {
		PX4_ERR("mount failed: %d (%s)", errno, strerror(errno));
		return false;
	}

	PX4_INFO("SD card mounted");
	return true;
}

bool SdUmsSwitch::start_usb_msc()
{
#if defined(CONFIG_SYSTEM_USBMSC)
	int ret = msconn_main(0, nullptr);

	if (ret != 0) {
		PX4_ERR("msconn failed: %d", ret);
		return false;
	}

	PX4_INFO("USB MSC started");
	return true;
#else
	PX4_ERR("USB MSC not enabled in config");
	return false;
#endif
}

bool SdUmsSwitch::stop_usb_msc()
{
#if defined(CONFIG_SYSTEM_USBMSC)
	int ret = msdis_main(0, nullptr);

	if (ret != 0) {
		PX4_ERR("msdis failed: %d", ret);
		return false;
	}

	PX4_INFO("USB MSC stopped");
	return true;
#else
	return true;
#endif
}

int SdUmsSwitch::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "status")) {
		if (_object.load()) {
			SdUmsSwitch *obj = static_cast<SdUmsSwitch *>(_object.load());
			PX4_INFO("State: %s", obj->_state == State::PX4_SD ? "PX4_SD" : "USB_MSC");
			PX4_INFO("VBUS: %s", obj->read_vbus_state() ? "connected" : "disconnected");
			return 0;
		}

		return -1;
	}

	return print_usage("unknown command");
}

int SdUmsSwitch::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SD card and USB Mass Storage switch module.

This module monitors USB VBUS state and automatically switches between:
- State A (PX4_SD): PX4 uses SD card for logging
- State B (USB_MSC): PC uses SD card via USB Mass Storage

When USB is connected, it stops the logger, unmounts the SD card, and starts USB MSC.
When USB is disconnected, it stops USB MSC, remounts the SD card.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sd_ums_switch", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace sd_ums_switch

extern "C" __EXPORT int sd_ums_switch_main(int argc, char *argv[])
{
	return sd_ums_switch::SdUmsSwitch::main(argc, argv);
}
