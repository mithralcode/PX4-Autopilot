/****************************************************************************
 *
 *   Copyright (c) 2025 Oliver Clarke. All rights reserved.
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

// standard C integer types - gives us uint32_t, int32_t etc
#include <stdint.h>

// standard C math - fabsf, M_PI etc
#include <math.h>

// ModuleBase - provides start/stop/status CLI handling
#include <px4_platform_common/module.h>

// ModuleParams - provides DEFINE_PARAMETERS and updateParams()
#include <px4_platform_common/module_params.h>

// ScheduledWorkItem - calls Run() on a fixed timer
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// work queue configurations - nav_and_controllers queue name
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

// PX4 logging - PX4_INFO, PX4_ERR, PX4_WARN macros
#include <px4_platform_common/log.h>

// hrt_absolute_time() - high resolution timer for timestamps
#include <drivers/drv_hrt.h>

// uORB read/write templates
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

// topic structs
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>

// PX4 math utilities - math::radians(), math::degrees()
#include <lib/mathlib/mathlib.h>

// matrix library - Quatf, Eulerf for quaternion conversion
#include <matrix/math.hpp>

// MODULE_NAME macro - defines the string name of this module
// used by logging and work queue registration
#define MODULE_NAME "flip_control"

class FlipControl : public ModuleBase<FlipControl>,
		    public ModuleParams,
		    public px4::ScheduledWorkItem
{
public:
	FlipControl();
	~FlipControl() override = default;

	static int	task_spawn(int argc, char *argv[]);
	static int	custom_command(int argc, char *argv[]);
	static int	print_usage(const char *reason = nullptr);

	void		Run() override;

private:

	enum class FlipState : uint8_t {
		DISABLED = 0,
		START,
		ROLL,
		RECOVER,
		FINISHED
	};

	FlipState	_flip_state{FlipState::DISABLED};

	uORB::Subscription	_rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<vehicle_rates_setpoint_s>
		_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};

	float	_last_channel_value{0.0f};
	float	_total_rotation_rad{0.0f};
	float	_last_roll_rad{0.0f};

	void	run_flip_state_machine(float roll_rad);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FLIP_ROLL_CW>)	_param_flip_roll_cw,
		(ParamInt<px4::params::FLIP_RC_CHAN>)		_param_flip_rc_chan
	)
};
