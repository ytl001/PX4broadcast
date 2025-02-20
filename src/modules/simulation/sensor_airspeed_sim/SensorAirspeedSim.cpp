/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "SensorAirspeedSim.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>

using namespace matrix;

SensorAirspeedSim::SensorAirspeedSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

SensorAirspeedSim::~SensorAirspeedSim()
{
	perf_free(_loop_perf);
}

bool SensorAirspeedSim::init()
{
	ScheduleOnInterval(125_ms); // 8 Hz
	return true;
}

float SensorAirspeedSim::generate_wgn()
{
	// generate white Gaussian noise sample with std=1

	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

void SensorAirspeedSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	if (_vehicle_local_position_sub.updated() && _vehicle_global_position_sub.updated()
	    && _vehicle_attitude_sub.updated()) {

		check_failure_injection();

		if (_sim_failure.get() == 0 && !_airspeed_disconnected) {

			vehicle_local_position_s lpos{};
			_vehicle_local_position_sub.copy(&lpos);

			vehicle_global_position_s gpos{};
			_vehicle_global_position_sub.copy(&gpos);

			vehicle_attitude_s attitude{};
			_vehicle_attitude_sub.copy(&attitude);

			Vector3f local_velocity = Vector3f{lpos.vx, lpos.vy, lpos.vz};
			Vector3f body_velocity = Dcmf{Quatf{attitude.q}} .transpose() * local_velocity;

			// device id
			device::Device::DeviceId device_id;
			device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
			device_id.devid_s.bus = 0;
			device_id.devid_s.address = 0;
			device_id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SIM;

			const float alt_amsl = gpos.alt;
			const float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_amsl;
			const float density_ratio = powf(TEMPERATURE_MSL / temperature_local, 4.256f);
			const float air_density = AIR_DENSITY_MSL / density_ratio;

			// calculate differential pressure + noise in hPa
			float _noise_scale = _sih_noise_scale.get();
			const float diff_pressure_noise = _noise_scale * (float)generate_wgn() * 0.01f;

			// as before, always body "forward" direction, but that is wrong for TS
			// float body_speed = body_velocity(0);

			// hacky fix: just take norm of vel.
			// inaccurate because flying sideways would not give significant pitot tube readings.
			// but then again fixed wings can't really fly sideways at significant speed.
			// also, wind is not considered at all here...?
			float body_speed = body_velocity.norm();
			// even nicer would be to define the pitot tube direction and do:
			//   body_speed = pitot_direction.T @ body_velocity

			float diff_pressure = sign(body_speed) * 0.005f * air_density * body_speed * body_speed + diff_pressure_noise;



			// airspeed blockage scale. implementation copied from SimulatorMavlink.cpp, update_sensors.
			const float blockage_fraction = 0.7; // defines max blockage (fully ramped)
			const float airspeed_blockage_rampup_time = 1_s; // time it takes to go max blockage, linear ramp

			float airspeed_blockage_scale = 1.f;

			if (_airspeed_blocked_timestamp > 0) {
				airspeed_blockage_scale = math::constrain(1.f - (hrt_absolute_time() - _airspeed_blocked_timestamp) /
							  airspeed_blockage_rampup_time, 1.f - blockage_fraction, 1.f);
			}


			differential_pressure_s differential_pressure{};
			// report.timestamp_sample = time;
			differential_pressure.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
			differential_pressure.differential_pressure_pa = diff_pressure * 100.0f * airspeed_blockage_scale; // hPa to Pa;
			differential_pressure.temperature = temperature_local;
			differential_pressure.timestamp = hrt_absolute_time();
			_differential_pressure_pub.publish(differential_pressure);

		}
	}

	perf_end(_loop_perf);
}

void SensorAirspeedSim::check_failure_injection()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);

		if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_AIRSPEED) {

			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, Airspeed off");
				supported = true;
				_airspeed_disconnected = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				PX4_WARN("CMD_INJECT_FAILURE, airspeed wrong (simulate pitot blockage)");
				supported = true;
				_airspeed_blocked_timestamp = hrt_absolute_time();

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, Airspeed ok");
				supported = true;
				_airspeed_disconnected = false;
			}
		}

		if (handled) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = supported ?
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}
	}
}


int SensorAirspeedSim::task_spawn(int argc, char *argv[])
{
	SensorAirspeedSim *instance = new SensorAirspeedSim();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SensorAirspeedSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SensorAirspeedSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensor_arispeed_sim", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sensor_airspeed_sim_main(int argc, char *argv[])
{
	return SensorAirspeedSim::main(argc, argv);
}
