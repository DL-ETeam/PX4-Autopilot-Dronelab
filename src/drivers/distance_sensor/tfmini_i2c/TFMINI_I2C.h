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

using namespace time_literals;

extern "C" __EXPORT int tfmini_i2c_main(int argc, char *argv[]);


class TFMINII2C : public ModuleBase<TFMINII2C>, public ModuleParams
{
public:
	TFMINII2C(int example_param, bool example_flag);

	virtual ~TFMINII2C() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TFMINII2C *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_TFMI2C>)   _p_sensor_enabled,
		(ParamInt<px4::params::SENS_EN_TFM0_OR>)  _p_sensor0_rot,
		(ParamInt<px4::params::SENS_EN_TFM1_OR>)  _p_sensor1_rot,
		(ParamInt<px4::params::SENS_EN_TFM2_OR>)  _p_sensor2_rot,
		(ParamInt<px4::params::SENS_EN_TFM3_OR>)  _p_sensor3_rot,
		(ParamInt<px4::params::SENS_EN_TFM4_OR>)  _p_sensor4_rot,
		(ParamInt<px4::params::SENS_EN_TFM5_OR>)  _p_sensor5_rot,
		(ParamInt<px4::params::SENS_EN_TFM6_OR>)  _p_sensor6_rot,
		(ParamInt<px4::params::SENS_EN_TFM7_OR>)  _p_sensor7_rot,
		(ParamInt<px4::params::SENS_EN_TFM8_OR>)  _p_sensor8_rot,
		(ParamInt<px4::params::SENS_EN_TFM9_OR>)  _p_sensor9_rot,
		(ParamInt<px4::params::SENS_EN_TFM10_OR>) _p_sensor10_rot,
		(ParamInt<px4::params::SENS_EN_TFM11_OR>) _p_sensor11_rot
	);

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};
