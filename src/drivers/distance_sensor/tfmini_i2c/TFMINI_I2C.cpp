/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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


// plan yeh hai, ek main parameter yeh dekhne ke liye ki kitne sensors hain.
// Uske baad har ek ke liye naya parameter jo ek list mein se orientation
// select karega. Asaan hoga ki har ek sensor ke liye pitch angle aur yaw angle
// ka option dedo enter karne ko

// baad ke liye yaad rakhna ki boardconfig mein yeh wala driver enable karna
// hota hai, nahi toh nahi compile hoga yeh firmware mein

#include "TFMINI_I2C.h"

#include <px4_platform_common/module_params.h> // following mappydot
#include <drivers/device/i2c.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <parameters/param.h>

#include <board_config.h>
#include <containers/Array.hpp>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>


#define TFMINII2C_BASE_ADDR 0x7F
#define TFMINII2C_MIN_ADDR 0x01

#define TFMINII2C_MIN_DISTANCE 0.04f // min should be 3 but is taken as 0.4m, due to EKF
#define TFMINII2C_MAX_DISTANCE 12.0f // issues

#define TFMINII2C_BUS_SPEED 100 // (100 ms) kuch bhi...

#define RANGE_FINDER_MAX_SENSORS 12
#define TFMINII2C_INTERVAL_BETWEEN_SUCCESSIVE_FIRES 150 // (150 ms) minimum is 100 ms

void
TFMINII2C::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	
}

void
TFMINII2C::start()
{
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(7_ms);
}


TFMINII2C::TFMINII2C(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{  // lightware laser serial aur i2c mein bhi aisa hi kuch kiya hai
	set_device_type(DRV_DIST_DEVTYPE_TFMINI);
}

TFMINII2C::~TFMINII2C()
{
	// Unadvertise the distance sensor topic.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// Free perf counters.
	perf_free(_comms_error);
	perf_free(_sample_perf);
}

int
TFMINII2C::get_sensor_rotation(const size_t index)
{
	switch (index) {
	case 0: return _p_sensor0_rot.get();

	case 1: return _p_sensor1_rot.get();

	case 2: return _p_sensor2_rot.get();

	case 3: return _p_sensor3_rot.get();

	case 4: return _p_sensor4_rot.get();

	case 5: return _p_sensor5_rot.get();

	case 6: return _p_sensor6_rot.get();

	case 7: return _p_sensor7_rot.get();

	case 8: return _p_sensor8_rot.get();

	case 9: return _p_sensor9_rot.get();

	case 10: return _p_sensor10_rot.get();

	case 11: return _p_sensor11_rot.get();

	default: return PX4_ERROR;
	}
}

int
TFMINII2C::init()
{	
	if (_p_sensor_enabled.get() == 0) {
		PX4_WARN("disabled");
		return PX4_ERROR;
	}

	// Initialize the I2C device
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	for (int i = 0; i <= RANGE_FINDER_MAX_SENSORS; i++) {
		set_device_address(TFMINII2C_BASE_ADDR + i);

		// Check if a sensor is present.
		if (probe() != PX4_OK) {
			break;
		}

		// Store I2C address
		_sensor_addresses[i] = TFMINII2C_BASE_ADDR + i;
		_sensor_rotations[i] = get_sensor_rotation(i);
		_sensor_count++;

		// Configure the sensor
		
		


		
		PX4_INFO("sensor %i at address 0x%02X added", i, get_device_address());
	}


	// Return an error if no sensors were detected.
	if (_sensor_count == 0) {
		PX4_ERR("no sensors discovered");
		return PX4_ERROR;
	}



	PX4_INFO("Total sensors connected: %i", _sensor_count);
	start();
	return PX4_OK;

	
}

int
TFMINII2C::collect()
{	// yaad rakhna, uorb alag alag honge alag alag sensors ke liye, 
	//  multiple ko implement karne mein yaad rahe
	uint8_t val[2] = {};
	perf_begin(_sample_perf);

	// Increment the sensor index, (limited to the number of sensors connected).
	for (int index = 0; index < _sensor_count; index++) {

		// Set address of the current sensor to collect data from.
		set_device_address(_sensor_addresses[index]);

		// Transfer data from the bus.
		int ret_val = transfer(nullptr, 0, &val[0], 2);

		if (ret_val < 0) {
			PX4_ERR("sensor %i read failed, address: 0x%02X", index, _sensor_addresses[index]);
			perf_count(_comms_error);
			perf_end(_sample_perf);
			return ret_val;
		}

		uint16_t distance_mm = uint16_t(val[0]) << 8 | val[1];
		float distance_m = static_cast<float>(distance_mm) / 1000.f;

		distance_sensor_s report {};
		report.current_distance = distance_m;
		report.device_id        = get_device_id();
		report.max_distance     = TFMINII2C_MAX_DISTANCE;
		report.min_distance     = TFMINII2C_MIN_DISTANCE;
		report.orientation      = _sensor_rotations[index];
		report.signal_quality   = -1;
		report.timestamp        = hrt_absolute_time();
		report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		report.variance         = 0;

		int instance_id;
		orb_publish_auto(ORB_ID(distance_sensor), &_distance_sensor_topic, &report, &instance_id);
	}

	perf_end(_sample_perf);
	return PX4_OK;
}



void
TFMINII2C::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

void
TFMINII2C::print_usage()
{
	

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
This is a template for a module running as a task in the background with start/stop/status functionality.
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "AAAAAAAA");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

}

void
TFMINII2C::RunImpl()
{
	// Collect the sensor data.
	if (collect() != PX4_OK) {
		PX4_INFO("collection error");
		// If an error occurred, restart the measurement state machine.
		start();
		return;
	}
}

int
tfmini_i2c_main(int argc, char *argv[])
{
	using ThisDriver = TFMINII2C;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = TFMINII2C_BUS_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	cli.i2c_address = TFMINII2C_BASE_ADDR;

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TFMINI);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "set_address")) {
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}