cmake_minimum_required(VERSION 2.8.3)
project(robot_wheel_odometry)


find_package(catkin REQUIRED
             std_msgs
	     message_generation
)

add_message_files(
	FILES
	WheelShift.msg
	WheelShifts.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)


