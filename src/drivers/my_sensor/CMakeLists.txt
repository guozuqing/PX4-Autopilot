############################################################################
# Simple MySensor Driver CMakeLists
############################################################################

px4_add_module(
	MODULE drivers__my_sensor
	MAIN my_sensor
	SRCS
		my_sensor.cpp
	DEPENDS
	)
