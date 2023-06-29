
#include "commander/commander.hpp"
#include "commander/config.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <math.h>

namespace commander
{
Commander::Commander(const std::string ref_traj_fname, const std::string mb_hostname,
                     const double kp, const double kd)
    : ref_traj_fname(ref_traj_fname), mb(mb_hostname), kp(kp), kd(kd)
{
	initialize();
	initialize_mb();
}

Commander::~Commander()
{
}

void
Commander::initialize()
{
	ref_traj.clear();
	traj.clear();

	ref_traj.reserve(t_dim_expected);
	readmatrix(ref_traj_fprefix + ref_traj_fname, ref_traj);
	t_size = ref_traj.size(); /** determine t_dim */

	traj.reserve(t_size);
	t_index = 0;
}

void
Commander::initialize_mb()
{
	mb.Init();

	for (size_t i = 0; i < driver_count; ++i) {
		mb.motor_drivers[i].motor1->SetCurrentReference(0);
		mb.motor_drivers[i].motor2->SetCurrentReference(0);
		mb.motor_drivers[i].motor1->Enable();
		mb.motor_drivers[i].motor2->Enable();
		mb.motor_drivers[i].EnablePositionRolloverError();
		mb.motor_drivers[i].SetTimeout(masterboard_timeout);
		mb.motor_drivers[i].Enable();

		// compare to example_pd.cpp, we plan to use the on-board PD controller
		mb.motor_drivers[i].motor1->set_kp(kp);
		mb.motor_drivers[i].motor2->set_kp(kp);
		mb.motor_drivers[i].motor1->set_kd(kd);
		mb.motor_drivers[i].motor2->set_kd(kd);
		mb.motor_drivers[i].motor1->set_current_sat(max_current);
		mb.motor_drivers[i].motor2->set_current_sat(max_current);
	}

	const double send_init_delay = 1e-3;
	auto prev_time = std::chrono::high_resolution_clock::now();

	while (!mb.IsTimeout() && !mb.IsAckMsgReceived()) {
		if (((std::chrono::duration<double>)(std::chrono::high_resolution_clock::now() -
		                                     prev_time))
		        .count() > send_init_delay) {
			prev_time = std::chrono::high_resolution_clock::now();
			mb.SendInit();
		}
	}

	if (mb.IsTimeout()) {
		printf("Timeout while waiting for ack.\n");
	}
}

void
Commander::print_all()
{
	mb.PrintIMU();
	mb.PrintADC();
	mb.PrintMotors();
	mb.PrintMotorDrivers();
	mb.PrintStats();
}

void
Commander::log_traj()
{
	writematrix(fprefix + traj_fname, traj);
}

bool
Commander::check_ready()
{
	mb.ParseSensorData();

	bool is_ready = true;

	for (size_t i = 0; i < motor_count; ++i) {
		if (!mb.motor_drivers[i / 2].is_connected) {
			// ignore the motors of a disconnected slave
			continue;
		}

		if (!(mb.motors[i].IsEnabled() && mb.motors[i].IsReady())) {
			is_ready = false;
		}
		init_pos[i] = mb.motors[i].GetPosition(); // initial position

		// not sure if this is needed?
		mb.motors[i].SetCurrentReference(0.);
		mb.motors[i].SetPositionReference(0.);
		mb.motors[i].SetVelocityReference(0.);
	}
	mb.SendCommand();

	return is_ready;
}

void
Commander::track(double (&pos_ref)[motor_count])
{
	mb.ParseSensorData();

	for (size_t i = 0; i < motor_count; ++i) {
		if (i % 2 == 0) {
			if (!mb.motor_drivers[i / 2].is_connected) {
				continue;
			}

			if (mb.motor_drivers[i / 2].error_code == 0xf) {
				continue;
			}
		}

		if (mb.motors[i].IsEnabled()) {
			mb.motors[i].SetPositionReference(pos_ref[i]);
		}
	}

	mb.SendCommand();
}


void
Commander::track(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count])
{
	mb.ParseSensorData();

	for (size_t i = 0; i < motor_count; ++i) {
		if (i % 2 == 0) {
			if (!mb.motor_drivers[i / 2].is_connected) {
				continue;
			}

			if (mb.motor_drivers[i / 2].error_code == 0xf) {
				continue;
			}
		}

		if (mb.motors[i].IsEnabled()) {
			mb.motors[i].SetPositionReference(pos_ref[i]);
			mb.motors[i].SetVelocityReference(vel_ref[i]);
		}
	}

	mb.SendCommand();
}

void
Commander::track_traj()
{
	double pos_ref[motor_count];
	double vel_ref[motor_count];

	for (size_t i = 0; i < motor_count; ++i) {
		pos_ref[i] = gear_ratio[motor2ref_idx[i]] * ref_traj[t_index][motor2ref_idx[i]];
		vel_ref[i] = gear_ratio[motor2ref_idx[i]] *
		    ref_traj[t_index][velocity_shift + motor2ref_idx[i]];
	}

	track(pos_ref, vel_ref);

	if (t_index < t_size - 1) {
		sample_traj();
		++t_index;
	}
}

void
Commander::sample_traj()
{
	Row<traj_dim> state;

	for (size_t i = 0; i < motor_count; ++i) {
		const double pos = mb.motors[i].GetPosition();
		const double vel = mb.motors[i].GetVelocity();
		state[ref2motor_idx[i]] = pos;
		state[ref2motor_idx[velocity_shift + i]] = vel;
	}

	traj.push_back(state);
}

void
Commander::command()
{
	if (!is_ready) {
		is_ready = check_ready();
		return;
	}

	switch (state) {
	case State::hold: {
		double pos_ref[motor_count];

		for (size_t i = 0; i < motor_count; ++i) {
			pos_ref[i] = gear_ratio[motor2ref_idx[i]] * ref_traj[0][motor2ref_idx[i]];;
		}
		track(pos_ref);
		break;
	}
	case State::track: {
		track_traj();
		break;
	}
	}
}

void
Commander::next_state()
{
	switch (state) {
	case State::hold: {
		state = State::track;
		break;
	}
	case State::track: {
		initialize();
		state = State::hold;
		break;
	}
	}
}

} // namespace commander
