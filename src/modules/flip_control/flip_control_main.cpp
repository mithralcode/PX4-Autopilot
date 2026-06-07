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

#include "flip_control.hpp"

// --- CONSTANTS ---

static constexpr uint32_t	SCHEDULE_INTERVAL_US{4000};		// 250Hz
static constexpr float		TRANSITION_START_RAD{0.785398f};	// 45 deg in rad
static constexpr float		TRANSITION_ROLL_RAD{5.759587f};		// 330 deg in rad
static constexpr float		THROTTLE_START{-0.8f};
static constexpr float		THROTTLE_ROLL{-0.1f};
static constexpr float		THROTTLE_RECOVER{-0.9f};

// --- CONSTRUCTOR ---

FlipControl::FlipControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

// --- RUN ---

void FlipControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// refresh parameters if any changed
	if (_parameter_update_sub.updated()) {
		parameter_update_s pu{};
		_parameter_update_sub.copy(&pu);
		updateParams();
	}

	// read current roll angle from attitude quaternion
	float roll_rad{0.0f};

	vehicle_attitude_s attitude{};

	if (_vehicle_attitude_sub.copy(&attitude)) {
		matrix::Eulerf euler{matrix::Quatf{attitude.q}};
		roll_rad = euler.phi();
	}

	// button edge detection
	if (_rc_channels_sub.updated()) {
		rc_channels_s rc{};
		_rc_channels_sub.copy(&rc);

		int ch = _param_flip_rc_chan.get();

		if (ch > 0) {
			float val = rc.channels[ch - 1];

			bool rising_edge = (_last_channel_value < 0.5f) && (val > 0.5f);

			if (rising_edge && _flip_state == FlipState::DISABLED) {
				PX4_INFO("flip triggered");
				_total_rotation_rad	= 0.0f;
				_last_roll_rad		= roll_rad;
				_flip_state		= FlipState::START;
			}

			_last_channel_value = val;
		}
	}

	run_flip_state_machine(roll_rad);
}

// --- STATE MACHINE ---

void FlipControl::run_flip_state_machine(float roll_rad)
{
	// accumulate total rotation using delta between ticks
	// fabsf makes it always positive
	// wrap-around handling stops bad delta at +/-PI boundary
	if (_flip_state != FlipState::DISABLED &&
	    _flip_state != FlipState::FINISHED) {

		float delta = roll_rad - _last_roll_rad;

		if (delta > (float)M_PI) {
			delta -= 2.0f * (float)M_PI;

		} else if (delta < -(float)M_PI) {
			delta += 2.0f * (float)M_PI;
		}

		_total_rotation_rad	+= fabsf(delta);
		_last_roll_rad		= roll_rad;
	}

	const float roll_rate_rad = math::radians(_param_flip_roll_cw.get());

	vehicle_rates_setpoint_s rates_sp{};
	rates_sp.timestamp	= hrt_absolute_time();
	rates_sp.pitch		= 0.0f;
	rates_sp.yaw		= 0.0f;

	switch (_flip_state) {

	case FlipState::DISABLED:
		return;

	case FlipState::START:
		rates_sp.roll			= roll_rate_rad;
		rates_sp.thrust_body[2]		= THROTTLE_START;
		_rates_setpoint_pub.publish(rates_sp);

		if (_total_rotation_rad >= TRANSITION_START_RAD) {
			PX4_INFO("START -> ROLL");
			_flip_state = FlipState::ROLL;
		}

		break;

	case FlipState::ROLL:
		rates_sp.roll			= roll_rate_rad;
		rates_sp.thrust_body[2]		= THROTTLE_ROLL;
		_rates_setpoint_pub.publish(rates_sp);

		if (_total_rotation_rad >= TRANSITION_ROLL_RAD) {
			PX4_INFO("ROLL -> RECOVER");
			_flip_state = FlipState::RECOVER;
		}

		break;

	case FlipState::RECOVER:
		rates_sp.roll			= 0.0f;
		rates_sp.thrust_body[2]		= THROTTLE_RECOVER;
		_rates_setpoint_pub.publish(rates_sp);

		PX4_INFO("RECOVER");
		_flip_state = FlipState::FINISHED;
		break;

	case FlipState::FINISHED:
		PX4_INFO("flip complete");
		_total_rotation_rad	= 0.0f;
		_last_roll_rad		= 0.0f;
		_flip_state		= FlipState::DISABLED;
		break;
	}
}

// --- BOILERPLATE ---

int FlipControl::task_spawn(int argc, char *argv[])
{
	FlipControl *instance = new FlipControl();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleOnInterval(SCHEDULE_INTERVAL_US);

	return 0;
}

int FlipControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlipControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Flip control module. Executes a clockwise roll flip
when triggered via RC channel button press.
Configure FLIP_ROLL_CW for roll rate and FLIP_RC_CHAN
for trigger channel.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flip_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flip_control_main(int argc, char *argv[])
{
	return FlipControl::main(argc, argv);
}
