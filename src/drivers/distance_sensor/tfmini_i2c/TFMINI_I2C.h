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

#pragma once
// yaad rakhna, uorb alag alag honge alag alag sensors ke liye, multiple ko implement karne
// mein yaad rahe
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <containers/Array.hpp>
#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

extern "C" __EXPORT int tfmini_i2c_main(int argc, char *argv[]);

// neeche ModuleBase ki jagah shayad I2CSPIDriver hi lagana hai, magar dekh
// lena ek baar
class TFMINII2C : public device::I2C, public ModuleParams, public I2CSPIDriver<TFMINII2C>
{
public:
	TFMINII2C(const I2CSPIDriverConfig &config);

	virtual ~TFMINII2C();


	/**
	 * Initializes the sensors, advertises uORB topic,
	 * sets device addresses
	 */
	virtual int init() override;

	/** @see ModuleBase */
	static void print_usage();


	/** @see ModuleBase::print_status() */
	void print_status() override;

	/**
	 * Initializes the automatic measurement state machine and starts the driver.
	 */
	void start();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();

private:

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/**
	 * Gets the current sensor rotation value.
	 */
	int get_sensor_rotation(const size_t index);

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	static constexpr int RANGE_FINDER_MAX_SENSORS = 12;

	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_addresses {};
	px4::Array<uint8_t, RANGE_FINDER_MAX_SENSORS> _sensor_rotations {};

	int _sensor_count{0};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_error{perf_alloc(PC_ELAPSED, "tfminii2c_comms_error")};
	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, "tfminii2c_sample_perf")};


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_TFMI2C>)   _p_sensor_enabled,
		(ParamInt<px4::params::SENS_TFM_0_ORT>)  _p_sensor0_rot,
		(ParamInt<px4::params::SENS_TFM_1_ORT>)  _p_sensor1_rot,
		(ParamInt<px4::params::SENS_TFM_2_ORT>)  _p_sensor2_rot,
		(ParamInt<px4::params::SENS_TFM_3_ORT>)  _p_sensor3_rot,
		(ParamInt<px4::params::SENS_TFM_4_ORT>)  _p_sensor4_rot,
		(ParamInt<px4::params::SENS_TFM_5_ORT>)  _p_sensor5_rot,
		(ParamInt<px4::params::SENS_TFM_6_ORT>)  _p_sensor6_rot,
		(ParamInt<px4::params::SENS_TFM_7_ORT>)  _p_sensor7_rot,
		(ParamInt<px4::params::SENS_TFM_8_ORT>)  _p_sensor8_rot,
		(ParamInt<px4::params::SENS_TFM_9_ORT>)  _p_sensor9_rot,
		(ParamInt<px4::params::SENS_TFM_10_ORT>) _p_sensor10_rot,
		(ParamInt<px4::params::SENS_TFM_11_ORT>) _p_sensor11_rot
	);

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};
