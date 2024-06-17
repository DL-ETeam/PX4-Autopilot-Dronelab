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

#define TFMINII2C_BASE_ADDR 0x01 // 0x01 hona chahiyeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
//#define TFMINII2C_MAX_ADDR 0x07F // not used

#define TFMINII2C_MIN_DISTANCE 0.04f // min should be 3 but is taken as 0.4m, due to EKF
#define TFMINII2C_MAX_DISTANCE 12.0f // issues

#define TFMINII2C_BUS_SPEED 100000 // default frame rate of 100 kHz

#define RANGE_FINDER_MAX_SENSORS 12
#define TFMINII2C_INTERVAL_BETWEEN_SUCCESSIVE_FIRES 150 // (150 ms) minimum is 100 ms

// TFmini Plus Control Commands
uint8_t TFMINI_ENABLE_OUTPUT[] = {0x5A, 0x05, 0x07, 0x01, 0x67};
uint8_t TFMINI_FRAME_RATE_ZERO[] = {0x5A, 0x06, 0x03, 0x00, 0x00, 0x9D};
uint8_t TFMINI_FRAME_RATE_HUNDRED[] = {0x5A, 0x06, 0x03, 0x00, 0x64, 0xC7};
uint8_t TFMINI_OBTAIN_DATA_MM[] = {0x5A, 0x05, 0x00, 0x06, 0x65};
uint8_t TFMINI_FW_VERSION[] = {0x5A, 0x04, 0x01, 0x5F};
uint8_t TFMINI_SAVE_SETTINGS[] = {0x5A, 0x04, 0x11, 0x6F};
uint8_t TFMINI_RESET[] = {0x5A, 0x04, 0x02, 0x60};
uint8_t TFMINI_I2C[] = {0x5A, 0x05, 0x0A, 0x01, 0x6A};

// TFmini addresses
uint8_t TFMINI_ADD1[] = {0x5A, 0x05, 0x0B, 0x01, 0x6B};
uint8_t TFMINI_ADD2[] = {0x5A, 0x05, 0x0B, 0x02, 0x6C};
uint8_t TFMINI_ADD3[] = {0x5A, 0x05, 0x0B, 0x03, 0x6D};
uint8_t TFMINI_ADD4[] = {0x5A, 0x05, 0x0B, 0x04, 0x6E};
uint8_t TFMINI_ADD5[] = {0x5A, 0x05, 0x0B, 0x05, 0x6F};
uint8_t TFMINI_ADD6[] = {0x5A, 0x05, 0x0B, 0x06, 0x70};
uint8_t TFMINI_ADD7[] = {0x5A, 0x05, 0x0B, 0x07, 0x71};
uint8_t TFMINI_ADD8[] = {0x5A, 0x05, 0x0B, 0x08, 0x72};
uint8_t TFMINI_ADD9[] = {0x5A, 0x05, 0x0B, 0x09, 0x73};
uint8_t TFMINI_ADD10[] = {0x5A, 0x05, 0x0B, 0x10, 0x74};
uint8_t TFMINI_ADD11[] = {0x5A, 0x05, 0x0B, 0x11, 0x75};
uint8_t TFMINI_ADD12[] = {0x5A, 0x05, 0x0B, 0x12, 0x76};
uint8_t TFMINI_ADD13[] = {0x5A, 0x05, 0x0B, 0x13, 0x77};
uint8_t TFMINI_ADD14[] = {0x5A, 0x05, 0x0B, 0x14, 0x78};
uint8_t TFMINI_ADD15[] = {0x5A, 0x05, 0x0B, 0x15, 0x79};
uint8_t TFMINI_ADD16[] = {0x5A, 0x05, 0x0B, 0x16, 0x7A};


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
	//PX4_DEBUG("staaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaarrrrrrrrrrrrrrrrrrrtttt");
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
	
	if (_p_sensor_enabled.get() == 0) {
		PX4_WARN("disabled");
		return PX4_ERROR;
	}

	// aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
	PX4_DEBUG("ek din pyaar, fir dusre din tum hate karte");

	// Initialize the I2C device
	if (I2C::init() != OK) {
		PX4_DEBUG("hate karte");
		return PX4_ERROR;
		
		
	}
	PX4_DEBUG("init %i",I2C::init());
	//PX4_DEBUG("teesre din pe vaar, fir chauthe din tum chase karte");
	// Allow for sensor auto-addressing time
	px4_usleep(100_ms);

	//PX4_DEBUG("panchwe din tum gate par the");
	int jj = 1; // flag for counting number of sensors
	// checks for sensors from base address until a maximum of 12 sensors are found
	for (int i = 0; i <= 128; i++) {
		set_device_address(TFMINII2C_BASE_ADDR + i);
		//PX4_DEBUG("trend bana yahan pe 0x%02X ko hate karte", TFMINII2C_BASE_ADDR + i);

		//uint8_t obtain_Data_mm[5] = {0x5A, 0x05, 0x00, 0x06, 0x65};

		int ret = measure();

		
		// Check if a sensor is present.
		//if (probe() != PX4_OK) {
		//	PX4_DEBUG("pppeeeecccoooorrrraaa");
		//	break;
		//}

		//PX4_DEBUG("measure %i",ret);

		if (ret == PX4_OK && jj < RANGE_FINDER_MAX_SENSORS) {
			
			_sensor_count++;
			jj++;

			// Store I2C address
			_sensor_addresses[jj] = TFMINII2C_BASE_ADDR + i;
			_sensor_rotations[jj] = get_sensor_rotation(jj);
			PX4_DEBUG("address of sensor was 0x%02x",TFMINII2C_BASE_ADDR + i);
			uint8_t val[4] {};
			transfer(TFMINI_RESET, sizeof(TFMINI_RESET),nullptr,0);
			px4_usleep(1_s);
			transfer(TFMINI_FW_VERSION,sizeof(TFMINI_FW_VERSION), val, sizeof(val));
			px4_usleep(1_s);
			transfer(nullptr,0, val, sizeof(val));
			PX4_DEBUG("probing command reply 0x%02x",val[3]);

			//sensor_arrangement(_sensor_count,TFMINII2C_BASE_ADDR + i);
			PX4_DEBUG("address of sensor is 0x%02x",_sensor_addresses[jj]);

		}
		
	

		 //Configure the sensor
		
		px4_usleep(50000); // 50 ms ?
		
		//PX4_INFO("sensor %i at address 0x%02X added", i, get_device_address());

		//PX4_DEBUG("rotation %i",get_sensor_rotation(i));
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
	//uint8_t obtain_Data_mm[] = TFMINI_OBTAIN_DATA_MM;

	int ret = transfer(TFMINI_OBTAIN_DATA_MM, sizeof(TFMINI_OBTAIN_DATA_MM), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		//PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	PX4_DEBUG("i2c::transfer returned %d", ret);

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
	//PX4_DEBUG("main mere dosh hum hain gande ilake se");

	uint8_t val[9] {};
	perf_begin(_sample_perf);

	// Increment the sensor index, (limited to the number of sensors connected).
	for (int index = 0; index < _sensor_count; index++) {

		//PX4_DEBUG("gande ilake se");

		// Set address of the current sensor to collect data from.
		set_device_address(_sensor_addresses[index]);

		uint8_t obtain_Data_mm[5] = {0x5A, 0x05, 0x00, 0x06, 0x65};
		// Transfer data from the bus.
		int ret_val = transfer(obtain_Data_mm, sizeof(obtain_Data_mm), val, sizeof(val));

		if (ret_val < 0) {
			//PX4_ERR("sensor %i read failed, address: 0x%02X", index, _sensor_addresses[index]);
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
		//PX4_INFO("collection error");
		// If an error occurred, restart the measurement state machine.
		start();
		return;
	}
}


int
TFMINII2C::sensor_arrangement(const uint8_t address)
{
	// The default address of a Benewake Tfmini Plus is 0x10
	// In order to connect multiple sensors on the same bus,
	// this function can be used to change their address, one at a time

	//if (_sensor_count > 1) {
	//	PX4_INFO("multiple sensors are connected");
	//	return PX4_ERROR;
	//}

	//if (address < 0 || address > 128){
	//	PX4_ERR("Invalid Address, please select an address from 1 to 128");
	//	return PX4_ERROR;
	//}
	uint8_t val[4] {};
	

	for (int ii = 0; ii <= 128; ii++){
		set_device_address(TFMINII2C_BASE_ADDR+ii);	
		transfer(TFMINI_FW_VERSION,sizeof(TFMINI_FW_VERSION), val, sizeof(val));
		px4_usleep(500_ms);
		transfer(nullptr,0, val, sizeof(val));

		if (val[1] == 07 && val[2] == 01){
			oldaddr = TFMINII2C_BASE_ADDR+ii;
			break;
		}

	}

	set_device_address(oldaddr);


	uint8_t addr_val[5] {};
	uint8_t confirm_val[5] {};
	
	switch(address){
		case 1:{
			transfer(TFMINI_ADD1,sizeof(TFMINI_ADD1), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 2:{
			transfer(TFMINI_ADD2,sizeof(TFMINI_ADD2), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 3:{
			transfer(TFMINI_ADD3,sizeof(TFMINI_ADD3), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 4:{
			transfer(TFMINI_ADD4,sizeof(TFMINI_ADD4), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 5:{
			transfer(TFMINI_ADD5,sizeof(TFMINI_ADD5), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 6:{
			transfer(TFMINI_ADD6,sizeof(TFMINI_ADD6), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 7:{
			transfer(TFMINI_ADD7,sizeof(TFMINI_ADD7), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 8:{
			transfer(TFMINI_ADD8,sizeof(TFMINI_ADD8), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 9:{
			transfer(TFMINI_ADD9,sizeof(TFMINI_ADD9), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 10:{
			transfer(TFMINI_ADD10,sizeof(TFMINI_ADD10), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 11:{
			transfer(TFMINI_ADD11,sizeof(TFMINI_ADD11), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 12:{
			transfer(TFMINI_ADD12,sizeof(TFMINI_ADD12), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 13:{
			transfer(TFMINI_ADD13,sizeof(TFMINI_ADD13), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 14:{
			transfer(TFMINI_ADD14,sizeof(TFMINI_ADD14), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 15:{
			transfer(TFMINI_ADD15,sizeof(TFMINI_ADD15), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
  		case 16:{
			transfer(TFMINI_ADD16,sizeof(TFMINI_ADD16), addr_val, sizeof(addr_val));
			px4_usleep(1_s);
			transfer(nullptr,0, addr_val, sizeof(addr_val));
			PX4_DEBUG("address change 0x%02x",addr_val[3]);
			break;
		}
		
  		default:
    			// code block
			break;
	}
	

	transfer(TFMINI_SAVE_SETTINGS,sizeof(TFMINI_SAVE_SETTINGS), confirm_val, sizeof(confirm_val));
	px4_usleep(1_s);
	transfer(nullptr,0, confirm_val, sizeof(confirm_val));
	PX4_DEBUG("save settings 0x%02x",confirm_val[4]);
	
	//PX4_INFO("requested address: %u", address);

	return 0;
}


void
TFMINII2C::custom_method(const BusCLIArguments &cli)
{
	//set_address(cli.i2c_address)
	PX4_DEBUG("jaaaddddooooooo");
}

int
tfmini_i2c_main(int argc, char *argv[])
{
	using ThisDriver = TFMINII2C;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = TFMINII2C_BUS_SPEED;


	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

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