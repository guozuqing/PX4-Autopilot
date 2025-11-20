#!/bin/sh
#
# SCH16T Debug Script for Sky V6X-RT
#
# 使用方法: 在 NSH 命令行运行此脚本
# nsh> sh /etc/init.d/debug_sch16t.sh
#

echo "=== SCH16T Sensor Debug ==="
echo ""

echo "1. 检查 SPI3 总线状态"
ls /dev/spi*

echo ""
echo "2. 检查传感器电源 (VDD_3V3_SENSORS3)"
# 读取 ADC 通道查看电压
adc test

echo ""
echo "3. 尝试启动 SCH16T 驱动"
sch16t stop
sleep 1
sch16t -s -b 3 -R 0 start

echo ""
echo "4. 检查驱动状态"
sleep 2
sch16t status

echo ""
echo "5. 检查传感器数据发布"
listener sensor_accel -n 5
listener sensor_gyro -n 5

echo ""
echo "6. 检查 SPI 通信错误"
dmesg | grep -i sch16t
dmesg | grep -i spi3
dmesg | grep -i error

echo ""
echo "=== Debug Complete ==="
