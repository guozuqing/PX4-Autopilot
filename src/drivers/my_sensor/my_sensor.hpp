/****************************************************************************
 * Simple MySensor Driver - Header File
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_simple.h>
#include <SerialImpl.hpp>
#include <cstring>

class MySensor : public ModuleBase<MySensor>
{
public:
	MySensor();
	~MySensor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MySensor *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void run();

private:
	static int run_trampoline(int argc, char *argv[]);

	bool init();
	void parseAndPublishData();

	device::SerialImpl _uart;
	uORB::Publication<sensor_simple_s> _sensor_pub{ORB_ID(sensor_simple)};

	// 参考代码中的数据解析相关变量
	char _data_buffer[10];
	bool _waiting_for_data{false};
	int _data_index{0};

	// 用于存储解析后的传感器数据
	struct {
		char data_str[5];
		char data1_str[5];
		float data;
		float data1;
	} _sensor_data;

	static constexpr const char *module_name = "my_sensor";
};
