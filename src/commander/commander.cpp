
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
Commander::print_state()
{
	printf("State | %.10s \n", state_to_name[state].c_str());
}

void
Commander::print_offset()
{
	bool header_printed = false;

	for (size_t i = 0; i < motor_count; ++i) {
		if (!mb.motor_drivers[i / 2].is_connected) {
			continue;
		}

		if (!header_printed) {
			printf("Motor | offset pos |\n");
			header_printed = true;
		}

		printf("%5.2ld | ", i);
		printf("%9.3g | ", index_offset[i]);
		printf("\n");
	}
}

void
Commander::print_traj()
{
	bool header_printed = false;

	for (size_t i = 0; i < motor_count; ++i) {
		if (!mb.motor_drivers[i / 2].is_connected) {
			continue;
		}

		if (!header_printed) {
			printf("Motor | Ref pos   | pos       | Ref vel   | vel       |\n");
			header_printed = true;
		}

		printf("%5.2d | ", i);
		printf("%9.3g | ", pos_ref[i]);
		printf("%9.3g | ", pos[i]);
		printf("%9.3g | ", vel_ref[i]);
		printf("%9.3g | ", vel[i]);
		printf("\n");
	}
}

void
Commander::print_all()
{
	print_state();
	print_offset();
	// print_traj();
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
			pos[i] = mb.motors[i].GetPosition();
			vel[i] = mb.motors[i].GetVelocity();

			mb.motors[i].SetPositionReference(pos_ref[i]);
			mb.motors[i].SetVelocityReference(vel_ref[i]);
		}
	}

	mb.SendCommand();
}

void
Commander::track_traj()
{
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
Commander::sweep_traj()
{
	//if (!was_offset_enabled) {
	//	was_offset_enabled = true;
	//	set_offset(index_offset);
	//}
	/* this does not work the second time? */
	// for (size_t i = 0; i < motor_count; ++i) {
	// 	pos_ref[i] = gear_ratio[motor2ref_idx[i]] * ref_traj[0][motor2ref_idx[i]];
	// 	vel_ref[i] = 0.;
	// }
	constexpr size_t t_sweep_size = static_cast<size_t>(1. / idx_sweep_freq * command_freq);
	bool all_ready = true;

	for (size_t i = 0; i < motor_count; i++) {
		if (!mb.motors[i].IsEnabled()) {
			continue;
		}

		if (was_index_detected[i]) {
			double des_pos = 0.;
			// mb.motors[i].SetCurrentReference(0.);
			mb.motors[i].SetPositionReference(des_pos);
			continue;
		}
		all_ready = false;

		if (mb.motors[i].HasIndexBeenDetected()) {
			was_index_detected[i] = true;
			index_offset[i] = mb.motors[i].GetPosition();
			/** enable offset */
			mb.motors[i].SetPositionOffset(index_offset[i]);
			// mb.motors[i].set_enable_index_offset_compensation(true);
			continue;
		}
		const double t =
		    static_cast<double>(t_sweep_index) / static_cast<double>(t_sweep_size);
		const double ref_pos = idx_sweep_ampl - idx_sweep_ampl * cos(2. * M_PI * t);
		const double ref_vel = -2. * M_PI * idx_sweep_ampl * sin(2. * M_PI * t);
		track(pos_ref, vel_ref);
	}
	++t_sweep_index;	
}

void
Commander::sample_traj()
{
	Row<traj_dim> state;

	for (size_t i = 0; i < motor_count; ++i) {

		state[ref2motor_idx[i]] = pos[i];
		state[ref2motor_idx[velocity_shift + i]] = vel[i];
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
		/* this does not work the second time? */
		for (size_t i = 0; i < motor_count; ++i) {
			pos_ref[i] =
			    0. * gear_ratio[motor2ref_idx[i]] * ref_traj[0][motor2ref_idx[i]];
			vel_ref[i] = 0.;
		}
		track(pos_ref, vel_ref);
		break;
	}
	case State::sweep: {
		sweep_traj();
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
		if (was_offset_enabled) {
			state = State::track;
		} else {
			state = State::sweep;
		}
		break;
	}
	case State::sweep: {
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

void
Commander::set_offset(double (&index_offset)[motor_count])
{
	for (size_t i = 0; i < motor_count; ++i) {
		mb.motors[i].SetPositionOffset(index_offset[i]);
		mb.motors[i].set_enable_index_offset_compensation(true);
	}
}

} // namespace commander
