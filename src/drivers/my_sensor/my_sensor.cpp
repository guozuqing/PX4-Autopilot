/****************************************************************************
 * Simple MySensor Driver - 按照标准PX4任务模式实现
 ****************************************************************************/

#include "my_sensor.hpp"
#include <px4_platform_common/getopt.h>
#include <errno.h>

using device::SerialConfig::ByteSize;
using device::SerialConfig::Parity;
using device::SerialConfig::StopBits;
using device::SerialConfig::FlowControl;

MySensor::MySensor() :
	ModuleBase(),
	_uart("/dev/ttyS1", 115200, ByteSize::EightBits,
	      Parity::None, StopBits::One,
	      FlowControl::Disabled)
{
	_waiting_for_data = false;
	_data_index = 0;
	memset(&_sensor_data, 0, sizeof(_sensor_data));
}

MySensor::~MySensor()
{
}

MySensor *MySensor::instantiate(int argc, char *argv[])
{
	MySensor *instance = new MySensor();
	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}
	return instance;
}

bool MySensor::init()
{
	// 打开串口
	if (!_uart.open()) {
		PX4_ERR("Failed to open %s", _uart.getPort());
		return false;
	}

	// 验证串口确实已打开
	if (!_uart.isOpen()) {
		PX4_ERR("UART not open after successful open() call");
		return false;
	}

	// 刷新串口缓冲区
	_uart.flush();

	PX4_INFO("MySensor initialized on %s at %d baud",
	         _uart.getPort(), (int)_uart.getBaudrate());

	return true;
}

void MySensor::run()
{
	int consecutive_errors = 0;
	const int max_consecutive_errors = 10;

	while (!should_exit()) {
		// 检查串口是否仍然打开
		if (!_uart.isOpen()) {
			PX4_WARN("UART closed, attempting to reopen...");
			if (!_uart.open()) {
				px4_usleep(1000000); // 1秒
				continue;
			}
			consecutive_errors = 0;
		}

		// 读取数据
		uint8_t read_buf[32];
		ssize_t n = _uart.read(read_buf, sizeof(read_buf));

		if (n > 0) {
			// 成功读取数据，重置错误计数器
			consecutive_errors = 0;

			// 处理读取到的每个字节
			for (int i = 0; i < n; i++) {
				uint8_t c = read_buf[i];

				// 实现参考代码中的数据解析逻辑
				if (!_waiting_for_data && c == 'r') {
					// 找到数据包开始标识符 'r'
					_waiting_for_data = true;
					_data_index = 0;
					memset(_data_buffer, 0, sizeof(_data_buffer));
				} else if (_waiting_for_data && _data_index < 10) {
					// 收集10个字符的数据
					_data_buffer[_data_index] = c;
					_data_index++;

					if (_data_index == 10) {
						// 收集完10个字符，开始解析
						parseAndPublishData();
						_waiting_for_data = false;
						_data_index = 0;
					}
				}
			}
		} else if (n < 0) {
			// 检查错误类型
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				// 非阻塞I/O中没有数据可读，这是正常的
				consecutive_errors = 0;
				px4_usleep(10000); // 10ms延时
			} else {
				// 真正的错误
				consecutive_errors++;

				// 如果连续错误太多，尝试重新打开串口
				if (consecutive_errors >= max_consecutive_errors) {
					PX4_WARN("Too many UART errors, reopening port...");
					_uart.close();
					px4_usleep(100000); // 等待100ms

					if (_uart.open()) {
						consecutive_errors = 0;
					} else {
						px4_usleep(1000000); // 等待1秒后重试
					}
					continue;
				}

				px4_usleep(10000); // 10ms延时
			}
		} else {
			// n == 0，没有数据，这是正常的
			consecutive_errors = 0;
			px4_usleep(10000); // 10ms延时
		}
	}
}

void MySensor::parseAndPublishData()
{
	// 实现参考代码中的逗号分隔解析逻辑
	const char delimiter[2] = ",";
	char *result = nullptr;
	int index = 0;

	// 复制数据缓冲区用于strtok处理（strtok会修改原字符串）
	char buffer_copy[11];
	memcpy(buffer_copy, _data_buffer, 10);
	buffer_copy[10] = '\0';

	// 清零传感器数据结构
	memset(&_sensor_data, 0, sizeof(_sensor_data));

	// 使用逗号分隔字符串
	result = strtok(buffer_copy, delimiter);
	while (result != nullptr && index < 2) {
		index++;

		switch (index) {
			case 1:
				strncpy(_sensor_data.data_str, result, 4);
				_sensor_data.data_str[4] = '\0';
				break;
			case 2:
				strncpy(_sensor_data.data1_str, result, 4);
				_sensor_data.data1_str[4] = '\0';
				break;
		}
		result = strtok(nullptr, delimiter);
	}

	// 将字符串转换为浮点数
	_sensor_data.data = atof(_sensor_data.data_str);
	_sensor_data.data1 = atof(_sensor_data.data1_str);

	// 创建并发布消息
	sensor_simple_s msg{};
	msg.timestamp = hrt_absolute_time();

	// 清零数组
	for (int i = 0; i < 50; i++) {
		msg.data[i] = 0.0f;
	}

	// 将解析的数据放入消息中
	msg.data[0] = _sensor_data.data;
	msg.data[1] = _sensor_data.data1;

	// 发布消息
	_sensor_pub.publish(msg);
}

int MySensor::run_trampoline(int argc, char *argv[])
{
	MySensor *instance = reinterpret_cast<MySensor *>(_object.load());

	if (instance) {
		instance->run();
	}

	return 0;
}

int MySensor::task_spawn(int argc, char *argv[])
{
	// 创建驱动实例
	MySensor *instance = MySensor::instantiate(argc, argv);

	if (instance == nullptr) {
		PX4_ERR("Failed to create MySensor instance");
		return PX4_ERROR;
	}

	_object.store(instance);

	// 初始化驱动
	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		return PX4_ERROR;
	}

	// 启动任务
	_task_id = px4_task_spawn_cmd("my_sensor",
	                              SCHED_DEFAULT,
	                              SCHED_PRIORITY_DEFAULT,
	                              2048,
	                              (px4_main_t)&MySensor::run_trampoline,
	                              (char *const *)argv);

		if (_task_id < 0) {
		PX4_ERR("Failed to spawn my_sensor task");
		delete instance;
		_object.store(nullptr);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int MySensor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MySensor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MySensor driver for UART-based sensor data reception and parsing.

### Implementation
This driver reads data from a UART port, looks for 'r' packet markers,
collects 10 bytes of data, parses comma-separated values, and publishes
the results via uORB.

### Examples
Start the driver:
$ my_sensor start

Stop the driver:
$ my_sensor stop

Check status:
$ my_sensor status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("my_sensor", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// 主函数
extern "C" __EXPORT int my_sensor_main(int argc, char *argv[])
{
	return MySensor::main(argc, argv);
}
