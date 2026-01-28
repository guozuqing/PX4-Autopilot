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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

namespace sd_ums_switch
{

class SdUmsSwitch : public ModuleBase<SdUmsSwitch>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SdUmsSwitch();
	~SdUmsSwitch() override;

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	void start();

private:
	enum class State {
		PX4_SD,      // State A: PX4 uses SD (normal flight/logging)
		USB_MSC      // State B: PC uses SD (USB mass storage mode)
	};

	void Run() override;

	bool read_vbus_state();
	bool switch_to_msc();
	bool switch_to_px4();

	bool stop_logger();
	bool start_logger();
	bool unmount_sd();
	bool mount_sd();
	bool start_usb_msc();
	bool stop_usb_msc();

	State _state{State::PX4_SD};

	bool _first_run{true};
	bool _vbus_prev{false};
	hrt_abstime _vbus_stable_time{0};
	static constexpr hrt_abstime DEBOUNCE_TIME_US = 300000; // 300ms debounce

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};

} // namespace sd_ums_switch
