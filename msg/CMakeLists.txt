############################################################################
#
#   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

include(px4_list_make_absolute)

set(msg_files
	action_request.msg
	actuator_armed.msg
	actuator_controls.msg
	actuator_controls_status.msg
	actuator_motors.msg
	actuator_outputs.msg
	actuator_servos.msg
	actuator_servos_trim.msg
	actuator_test.msg
	adc_report.msg
	airspeed.msg
	airspeed_validated.msg
	airspeed_wind.msg
	autotune_attitude_control_status.msg
	battery_status.msg
	button_event.msg
	camera_capture.msg
	camera_status.msg
	camera_trigger.msg
	cellular_status.msg
	collision_constraints.msg
	collision_report.msg
	commander_state.msg
	control_allocator_status.msg
	cpuload.msg
	differential_pressure.msg
	distance_sensor.msg
	ekf2_timestamps.msg
	esc_report.msg
	esc_status.msg
	estimator_aid_source_1d.msg
	estimator_aid_source_2d.msg
	estimator_aid_source_3d.msg
	estimator_bias.msg
	estimator_event_flags.msg
	estimator_gps_status.msg
	estimator_innovations.msg
	estimator_selector_status.msg
	estimator_sensor_bias.msg
	estimator_states.msg
	estimator_status.msg
	estimator_status_flags.msg
	event.msg
	failure_detector_status.msg
	follow_target.msg
	follow_target_estimator.msg
	follow_target_status.msg
	generator_status.msg
	geofence_result.msg
	gimbal_device_attitude_status.msg
	gimbal_device_information.msg
	gimbal_device_set_attitude.msg
	gimbal_manager_information.msg
	gimbal_manager_set_attitude.msg
	gimbal_manager_set_manual_control.msg
	gimbal_manager_status.msg
	gps_dump.msg
	gps_inject_data.msg
	health_report.msg
	gripper.msg
	heater_status.msg
	home_position.msg
	hover_thrust_estimate.msg
	input_rc.msg
	internal_combustion_engine_status.msg
	iridiumsbd_status.msg
	irlock_report.msg
	landing_gear.msg
	landing_target_innovations.msg
	landing_target_pose.msg
	led_control.msg
	log_message.msg
	logger_status.msg
	mag_worker_data.msg
	magnetometer_bias_estimate.msg
	manual_control_setpoint.msg
	manual_control_switches.msg
	mavlink_log.msg
	mavlink_tunnel.msg
	mission.msg
	mission_result.msg
	mount_orientation.msg
	navigator_mission_item.msg
	npfg_status.msg
	obstacle_distance.msg
	offboard_control_mode.msg
	onboard_computer_status.msg
	orbit_status.msg
	parameter_update.msg
	ping.msg
	position_controller_landing_status.msg
	position_controller_status.msg
	position_setpoint.msg
	position_setpoint_triplet.msg
	power_button_state.msg
	power_monitor.msg
	pps_capture.msg
	pwm_input.msg
	px4io_status.msg
	radio_status.msg
	rate_ctrl_status.msg
	rc_channels.msg
	rc_parameter_map.msg
	rpm.msg
	rtl_time_estimate.msg
	satellite_info.msg
	sensor_accel.msg
	sensor_accel_fifo.msg
	sensor_baro.msg
	sensor_combined.msg
	sensor_correction.msg
	sensor_gnss_relative.msg
	sensor_gps.msg
	sensor_gyro.msg
	sensor_gyro_fft.msg
	sensor_gyro_fifo.msg
	sensor_hygrometer.msg
	sensor_mag.msg
	sensor_optical_flow.msg
	sensor_preflight_mag.msg
	sensor_selection.msg
	sensors_status.msg
	sensors_status_imu.msg
	system_power.msg
	takeoff_status.msg
	task_stack_info.msg
	tecs_status.msg
	telemetry_status.msg
	timesync.msg
	timesync_status.msg
	trajectory_bezier.msg
	trajectory_setpoint.msg
	trajectory_waypoint.msg
	transponder_report.msg
	tune_control.msg
	uavcan_parameter_request.msg
	uavcan_parameter_value.msg
	ulog_stream.msg
	ulog_stream_ack.msg
	uwb_distance.msg
	uwb_grid.msg
	vehicle_acceleration.msg
	vehicle_air_data.msg
	vehicle_angular_acceleration.msg
	vehicle_angular_acceleration_setpoint.msg
	vehicle_angular_velocity.msg
	vehicle_attitude.msg
	vehicle_attitude_setpoint.msg
	vehicle_command.msg
	vehicle_command_ack.msg
	vehicle_constraints.msg
	vehicle_control_mode.msg
	vehicle_global_position.msg
	vehicle_imu.msg
	vehicle_imu_status.msg
	vehicle_land_detected.msg
	vehicle_local_position.msg
	vehicle_local_position_setpoint.msg
	vehicle_magnetometer.msg
	vehicle_odometry.msg
	vehicle_optical_flow.msg
	vehicle_optical_flow_vel.msg
	vehicle_rates_setpoint.msg
	vehicle_roi.msg
	vehicle_status.msg
	vehicle_status_flags.msg
	vehicle_thrust_setpoint.msg
	vehicle_torque_setpoint.msg
	vehicle_trajectory_bezier.msg
	vehicle_trajectory_waypoint.msg
	vtol_vehicle_status.msg
	wheel_encoders.msg
	wind.msg
	yaw_estimator_status.msg
)
list(SORT msg_files)

px4_list_make_absolute(msg_files ${CMAKE_CURRENT_SOURCE_DIR} ${msg_files})

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	# Check that the msg directory and the CMakeLists.txt file exists
	if(EXISTS ${EXTERNAL_MODULES_LOCATION}/msg/CMakeLists.txt)
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/msg external_msg)

		# Add each of the external message files to the global msg_files list
		foreach(external_msg_file ${config_msg_list_external})
			list(APPEND msg_files ${EXTERNAL_MODULES_LOCATION}/msg/${external_msg_file})
		endforeach()
	endif()
endif()

# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)
set(ucdr_out_path ${PX4_BINARY_DIR}/uORB/ucdr)
set(msg_source_out_path ${CMAKE_CURRENT_BINARY_DIR}/topics_sources)

set(uorb_headers)
set(uorb_sources)
set(uorb_ucdr_headers)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)

	# Pascal case to snake case (MsgFile -> msg_file)
	string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" msg "${msg}")
	string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" msg "${msg}")
	string(TOLOWER "${msg}" msg)

	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cpp)
	list(APPEND uorb_ucdr_headers ${ucdr_out_path}/${msg}.h)
endforeach()

# set parent scope msg_files for other modules to consume (eg topic_listener)
set(msg_files ${msg_files} PARENT_SCOPE)

# Generate uORB headers
add_custom_command(
	OUTPUT
		${uorb_headers}
		${msg_out_path}/uORBTopics.hpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${msg_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.hpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

# Generate microcdr headers
add_custom_command(
	OUTPUT ${uorb_ucdr_headers}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${ucdr_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic ucdr headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_ucdr_headers DEPENDS ${uorb_ucdr_headers})

# Generate uORB sources
add_custom_command(
	OUTPUT
		${uorb_sources}
		${msg_source_out_path}/uORBTopics.cpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--sources
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR}
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic sources"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)

add_library(uorb_msgs ${uorb_headers} ${msg_out_path}/uORBTopics.hpp ${uorb_sources} ${msg_source_out_path}/uORBTopics.cpp)
target_link_libraries(uorb_msgs PRIVATE m)
add_dependencies(uorb_msgs prebuild_targets uorb_headers)

if(CONFIG_LIB_CDRSTREAM)
	set(uorb_cdr_idl)
	set(uorb_cdr_msg)
	set(uorb_cdr_idl_uorb)
	set(idl_include_path ${PX4_BINARY_DIR}/uORB/idl)
	set(idl_out_path ${idl_include_path}/px4/msg)
	set(idl_uorb_path ${PX4_BINARY_DIR}/msg/px4/msg)

	# Make sure that CycloneDDS has been checkout out
	execute_process(COMMAND git submodule sync src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )
	execute_process(COMMAND git submodule update --init --force src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )

	# CycloneDDS-tools doesn't ship with the cdrstream-desc feature thus we've to compile idlc from source
	MESSAGE(STATUS "Configuring idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds
			-DCMAKE_C_COMPILER=/usr/bin/gcc
			-DBUILD_EXAMPLES=OFF
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	MESSAGE(STATUS "Building idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} --build . --target idlc
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	list(APPEND CMAKE_PROGRAM_PATH "${CMAKE_CURRENT_BINARY_DIR}/idlc/bin")

	# Copy .msg files
	foreach(msg_file ${msg_files})
		get_filename_component(msg ${msg_file} NAME_WE)
		configure_file(${PX4_SOURCE_DIR}/msg/${msg}.msg ${idl_out_path}/${msg}.msg COPYONLY)
		list(APPEND uorb_cdr_idl ${idl_out_path}/${msg}.idl)
		list(APPEND uorb_cdr_msg ${idl_out_path}/${msg}.msg)
		list(APPEND uorb_cdr_idl_uorb ${idl_uorb_path}/${msg}.h)
	endforeach()

	# Generate IDL from .msg using rosidl_adapter
	# Note this a submodule inside PX4 hence no ROS2 installation required
	add_custom_command(
		OUTPUT ${uorb_cdr_idl}
		COMMAND ${CMAKE_COMMAND}
		        -E env "PYTHONPATH=${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_adapter:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_cli"
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/cdrstream/msg2idl.py
			${uorb_cdr_msg}
		DEPENDS
			${uorb_cdr_msg}
			git_cyclonedds
		COMMENT "Generating IDL from uORB topic headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)

	# Generate C definitions from IDL
	set(CYCLONEDDS_DIR ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds)
	include("${CYCLONEDDS_DIR}/cmake/Modules/Generate.cmake")
	idlc_generate(TARGET uorb_cdrstream
                  FEATURES "cdrstream-desc"
                  FILES ${uorb_cdr_idl}
                  INCLUDES ${idl_include_path}
                  BASE_DIR ${idl_include_path}
                  WARNINGS no-implicit-extensibility)
	target_link_libraries(uorb_cdrstream INTERFACE cdr)

	# Generate and overwrite IDL header with custom headers for uORB operatability
	# We typedef the IDL struct the uORB struct so that the IDL offset calculate
	# the offset of internal uORB struct for serialization/deserialization

	# In the future we might want to turn this around let the IDL struct be the leading ABI
	# However we need to remove the padding for logging and remove the re-ordering of fields

	add_custom_target(
		uorb_idl_header
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			--uorb-idl-header
			-f ${msg_files}
			-i ${CMAKE_CURRENT_SOURCE_DIR}
			-o ${idl_uorb_path}
			-e ${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream
		DEPENDS
			uorb_cdrstream
			${msg_files}
			${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream/uorb_idl_header.h.em
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
		COMMENT "Generating uORB compatible IDL headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
	)
	add_dependencies(uorb_msgs uorb_idl_header)

	# Compile all CDR compatible message defnitions
	target_link_libraries(uorb_msgs PRIVATE uorb_cdrstream )
endif()

if(CONFIG_MODULES_ZENOH)
	# Update kconfig file for topics
	execute_process(COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-config
			-f ${msg_files}
			-o ${PX4_SOURCE_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
		)
	add_custom_command(
		OUTPUT
			${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-pub-sub
			-f ${msg_files}
			-o ${PX4_BINARY_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
		DEPENDS
			${msg_files}
			${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh/uorb_pubsub_factory.hpp.em
			${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
		COMMENT "Generating Zenoh Topic Code"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)
		add_library(zenoh_topics ${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp)
		set_target_properties(zenoh_topics PROPERTIES LINKER_LANGUAGE CXX)
endif()
