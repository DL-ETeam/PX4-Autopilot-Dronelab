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

// AAAAAAAAAAAAAAAAAAAAAAAAAA i2cdetect -b2 kyunki cubeorange pe port 2 , 1 nahi AAAAAA

#include "TFMINI_I2C.h"


// ------------------------------------
#undef PX4_DEBUG
#define PX4_DEBUG PX4_INFO
// ------------------------------------

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
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
	
}

void
TFMINII2C::start()
{
	// Fetch parameter values.
	ModuleParams::updateParams();

	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	// that (7_ms) is from tfmini^
	ScheduleOnInterval(5); // 5 us, from tf02pro (5)
}


TFMINII2C::TFMINII2C(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{  	// lightware laser serial aur i2c mein bhi aisa hi kuch kiya hai
	set_device_type(DRV_DIST_DEVTYPE_TFMINI);
}

TFMINII2C::~TFMINII2C()
{
	// Unadvertise the distance sensor topic.
	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	// Free perf counters.
	perf_free(_comms_errors);
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
	// aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
	PX4_DEBUG("hate karte hate karte");
	
	uint8_t obtain_Data_mm[5] = {0x5A, 0x05, 0x00, 0x06, 0x67};
	int ret = transfer(obtain_Data_mm, sizeof(obtain_Data_mm), nullptr, 0);

	if (_p_sensor_enabled.get() == 0) {
		PX4_WARN("disabled");
		return PX4_ERROR;
	}

	// Initialize the I2C device
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	for (int i = 0; i <= RANGE_FINDER_MAX_SENSORS; i++) {
		PX4_DEBUG("hate karte");
		set_device_address(TFMINII2C_BASE_ADDR + i);

		//uint8_t obtain_Data_mm[5] = {0x5A, 0x05, 0x00, 0x06, 0x65};

		ret = measure();

		
		// Check if a sensor is present.
		//if (probe() != PX4_OK) {
		//	break;
		//}

		if (ret == PX4_OK) {
			// Store I2C address
			_sensor_addresses[i] = TFMINII2C_BASE_ADDR + i;
			_sensor_rotations[i] = get_sensor_rotation(i);
			_sensor_count++;
		}

		

		// Configure the sensor
		
		

		px4_usleep(50000); // 50 ms ?
		
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

int TFMINII2C::measure()
{
	uint8_t obtain_Data_mm[5] = {0x5A, 0x05, 0x00, 0x06, 0x65};

	int ret = transfer(obtain_Data_mm, sizeof(obtain_Data_mm), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

/**
 * @brief
 *
 * @return int
 * @Note
 *   Receive Frame
 *   Byte0: 0x59, frame header, same for each frame
 *   Byte1: 0x59, frame header, same for each frame
 *   Byte2: Dist_L distance value low 8 bits
 *   Byte3: Dist_H distance value high 8 bits
 *   Byte4: Strength_L low 8 bits
 *   Byte5: Strength_H high 8 bits
 *   Byte6: Temp_L low 8 bits
 *   Byte7: Temp_H high 8 bits
 *   Byte8: Checksum is the lower 8 bits of the cumulative sum of the number of the first 8 bytes
 *
 */
int
TFMINII2C::collect()
{	// yaad rakhna, uorb alag alag honge alag alag sensors ke liye, 
	//  multiple ko implement karne mein yaad rahe

	// aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
	PX4_DEBUG("main mere dosh humhai gande ilake se");

	uint8_t val[9] {};
	perf_begin(_sample_perf);

	// Increment the sensor index, (limited to the number of sensors connected).
	for (int index = 0; index < _sensor_count; index++) {

		PX4_DEBUG("gande ilake se");

		// Set address of the current sensor to collect data from.
		set_device_address(_sensor_addresses[index]);

		uint8_t obtain_Data_mm[5] = {0x5A, 0x05, 0x00, 0x06, 0x65};
		// Transfer data from the bus.
		int ret_val = transfer(obtain_Data_mm, sizeof(obtain_Data_mm), val, sizeof(val));

		if (ret_val < 0) {
			PX4_ERR("sensor %i read failed, address: 0x%02X", index, _sensor_addresses[index]);
			perf_count( _comms_errors);
			perf_end(_sample_perf);
			return ret_val;
		}

		//uint16_t strength = val[5] << 8 | val[4];
		uint16_t distance_mm = val[3] << 8 | val[2];
		float distance_m = float(distance_mm) * 1e-3f;
		
		// if strength is worse than a limit, discard reading
		//if (strength >= 60u && distance_mm < 45000u) {

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
			
		
		//}
	}

	perf_end(_sample_perf);
	return PX4_OK;
}





void
TFMINII2C::print_usage()
{
	

	PRINT_MODULE_USAGE_NAME("Benewake Tfmini Plus I2C", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_COMMAND("set_address");
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
	int ch;
	using ThisDriver = TFMINII2C;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = TFMINII2C_BUS_SPEED;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TF02PRO);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}