#include "commander/commander.hpp"
#include "commander/config.hpp"
#include <math.h>

namespace commander
{
Commander::Commander(const std::string mb_if_name, const std::string ref_traj_fname)
    : mb(mb_if_name), ref_traj_fname(ref_traj_fname)
{
	reset();
}

Commander::~Commander()
{
}

/* reset the commander */
void
Commander::reset()
{
	command_timing_stats.reset();
	print_timing_stats.reset();

	ref_traj.clear();
	traj.clear();

	ref_traj.reserve(t_dim_expected);
	ref_traj_reader(ref_traj_fprefix + ref_traj_fname, ref_traj);
	t_size = ref_traj.size(); /* determine t_dim */

	traj.reserve(t_size);
	t_index = 0;
}

/* initialize the masterboard interface */
void
Commander::initialize_masterboard()
{
	mb.Init();

	for (size_t i = 0; i < driver_count; ++i) {
		mb.motor_drivers[i].motor1->SetCurrentReference(0);
		mb.motor_drivers[i].motor2->SetCurrentReference(0);
		mb.motor_drivers[i].motor1->Enable();
		mb.motor_drivers[i].motor2->Enable();

		// Set the gains for the PD controller running on the cards.
		mb.motor_drivers[i].motor1->set_kp(kp);
		mb.motor_drivers[i].motor2->set_kp(kp);
		mb.motor_drivers[i].motor1->set_kd(kd);
		mb.motor_drivers[i].motor2->set_kd(kd);

		// Set the maximum current controlled by the card.
		mb.motor_drivers[i].motor1->set_current_sat(max_current);
		mb.motor_drivers[i].motor2->set_current_sat(max_current);

		mb.motor_drivers[i].EnablePositionRolloverError();
		mb.motor_drivers[i].SetTimeout(masterboard_timeout);
		mb.motor_drivers[i].Enable();
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
	} else {
		is_masterboard_connected = true;
	}
}

/* main loop

Do NOT modify!
Timing is important, modify instead: command(), print_info() and change_to_next_state()
*/
void
Commander::loop(std::atomic_bool &is_running, std::atomic_bool &is_changing_state)
{
	std::chrono::high_resolution_clock::time_point now_time;

	auto command_time = std::chrono::high_resolution_clock::now();
	auto print_time = std::chrono::high_resolution_clock::now();

	while (is_running) {
		now_time = std::chrono::high_resolution_clock::now();

		/* command every command period */
		if (now_time >= command_time) {
			command(); // *** modify me ***
			//  when the command was finished
			auto finish_time = std::chrono::high_resolution_clock::now();
			// when the command should have been finished by
			auto deadline = now_time +
			    std::chrono::nanoseconds(static_cast<size_t>(
				std::nano::den * commander::command_period));
			/* save timing stats */

			// margin = deadline - finish time
			double margin;
			if (finish_time < deadline) { // we are good, margin is positive
				margin =
				    std::chrono::duration<double>(deadline - finish_time).count();
			} else { // we are late, margin is negative
				command_timing_stats.violation_count++;
				margin =
				    -std::chrono::duration<double>(finish_time - deadline).count();
			}
			// elapsed = finish time - start time
			const double elapsed =
			    std::chrono::duration<double>(finish_time - now_time).count();

			command_timing_stats.update(margin, elapsed);
			command_time = deadline; // the next command time is the current deadline.
		}

		/* print every print period */
		if (now_time >= print_time) {
			printf("\33[H\33[2J"); //* clear screen

			print_info(); // *** modify me ***

			auto finish_time = std::chrono::high_resolution_clock::now();
			const double elapsed =
			    std::chrono::duration<double>(finish_time - now_time).count();

			/* print does not need detailed stats */
			print_timing_stats.update(0, elapsed);

			// skip if needed
			while (print_time < now_time) {
				print_timing_stats.skip_count++;
				print_time += std::chrono::nanoseconds(
				    static_cast<size_t>(std::nano::den * commander::print_period));
			}
		}

		/* change state if needed */
		if (is_changing_state) {
			is_changing_state = false;
			change_to_next_state(); // *** modify me ***
		}
	}
}

void
Commander::enable_hard_calibration()
{
	is_hard_calibrating = true;
}

void
Commander::disable_onboard_pd()
{
	using_masterboard_pd = false;
}

/* controls state flow */
void
Commander::change_to_next_state()
{
	switch (state) {
	case State::not_ready: {
		if (is_masterboard_ready) {
			state = State::sweep;
		} else {
			printf("Not ready!\n");
		}
		break;
	}
	case State::hold: {
		state = State::track;
		traj_is_sampled = false;
		break;
	}
	case State::sweep: {
		state = State::hold;
		break;
	}
	case State::track: {
		state = State::hold;
		break;
	}
	}
}

/************************/
/* commanding functions */
/************************/

/* commands the masterboard */
void
Commander::command()
{
	if (mb.IsTimeout()) {
		is_masterboard_connected = false;
	}

	update_stats();

	switch (state) {
	case State::not_ready: {
		if (command_check_ready()) { // change to next stage if ready
			is_masterboard_ready = true;
			change_to_next_state();
		};
		break;
	}
	case State::hold: {
		get_hold_reference(pos_ref, vel_ref);
		command_reference(pos_ref, vel_ref);
		break;
	}
	case State::sweep: {
		generate_sweep_command();
		break;
	}
	case State::track: {
		generate_track_command();
		break;
	}
	}
}

/* command ready up sequence */
bool
Commander::command_check_ready()
{
	if (!is_masterboard_connected) {
		return false;
	}

	all_index_detected = true;
	bool is_ready = true;

	mb.ParseSensorData();

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

		if (mb.motors[i].HasIndexBeenDetected()) {
			was_index_detected[i] = true;
		} else {
			all_index_detected = false;
		}
	}
	mb.SendCommand();

	return is_ready;
}

/* command pos and vel reference for the onboard PD controller */
void
Commander::command_reference(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count])
{
	if (!is_masterboard_connected) {
		return;
	}

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

			saturate_reference(pos_ref);          // position saturation for safety
			mb.motors[i].SetCurrentReference(0.); // signifies we are using onboard PD
			mb.motors[i].SetPositionReference(pos_ref[i]);
			mb.motors[i].SetVelocityReference(vel_ref[i]);
		}
	}
	mb.SendCommand();
}

/* UNIMPLEMENTED: command current */
void
Commander::command_current(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count])
{
	if (!is_masterboard_connected) {
		return;
	}

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

			// to be implemented
			// calculate current here !!!!!
			double current = 0;
			mb.motors[i].SetCurrentReference(current);
		}
	}

	mb.SendCommand();
}

/******************/
/* Read reference */
/******************/

void
Commander::get_hold_reference(double (&pos_ref)[motor_count], double (&vel_ref)[motor_count])
{
	for (size_t i = 0; i < motor_count; ++i) {
		if (hip_offset_flag) {
			if (i == 0 || i == 1 || i == 6 || i == 7) {
				pos_ref[i] = gear_ratio[motor2ref_idx[i]] *
				    (ref_traj[0][motor2ref_idx[i]] + hip_offset_position[i]);

			} else {
				pos_ref[i] = gear_ratio[motor2ref_idx[i]] *
				    ref_hold_position[motor2ref_idx[i]];
			}
		} else {
			pos_ref[i] =
			    gear_ratio[motor2ref_idx[i]] * ref_hold_position[motor2ref_idx[i]];
		}

		/* override velocity */
		vel_ref[i] = 0.;
	}
}

void
Commander::get_reference(const size_t t_index, double (&pos_ref)[motor_count],
                         double (&vel_ref)[motor_count])
{
	for (size_t i = 0; i < motor_count; ++i) {

		if (hip_offset_flag) {
			if (i == 0 || i == 1 || i == 6 || i == 7) {
				pos_ref[i] = gear_ratio[motor2ref_idx[i]] *
				    (ref_traj[t_index][motor2ref_idx[i]] + hip_offset_position[i]);

			} else {
				pos_ref[i] = gear_ratio[motor2ref_idx[i]] *
				    ref_traj[t_index][motor2ref_idx[i]];
			}
		} else {
			pos_ref[i] =
			    gear_ratio[motor2ref_idx[i]] * ref_traj[t_index][motor2ref_idx[i]];
		}
		vel_ref[i] = gear_ratio[motor2ref_idx[i]] *
		    ref_traj[t_index][velocity_shift + motor2ref_idx[i]];
	}
}

/********************************/
/* command generation functions */
/********************************/

void
Commander::generate_sweep_command()
{

	if (all_index_detected) {
		/* read index position from file */
		matrix_rw::Reader<motor_count> index_pos_reader;
		std::vector<Row<motor_count>> index_pos_vec;
		index_pos_reader(fprefix + index_pos_fname, index_pos_vec);
		for (size_t i = 0; i < motor_count; ++i) {
			index_pos[i] = index_pos_vec[0][i];
		}
	}

	constexpr size_t t_sweep_size = static_cast<size_t>(1. / idx_sweep_freq * command_freq);
	bool all_ready = true;

	for (size_t i = 0; i < motor_count; i++) {

		if (was_index_detected[i]) {
			pos_ref[i] = index_pos[i];
			vel_ref[i] = 0;
			continue;
		}

		all_ready = false;

		if (mb.motors[i].HasIndexBeenDetected()) {
			was_index_detected[i] = true;
			index_pos[i] = mb.motors[i].GetPosition();
			continue;
		}
		const double t =
		    static_cast<double>(t_sweep_index) / static_cast<double>(t_sweep_size);
		pos_ref[i] = gear_ratio[motor2ref_idx[i]] *
		    (idx_sweep_ampl - idx_sweep_ampl * cos(2. * M_PI * t)) *
		    get_sign(gear_ratio[i]);
		vel_ref[i] = gear_ratio[motor2ref_idx[i]] *
		    (-2. * M_PI * idx_sweep_ampl * sin(2. * M_PI * t)) * get_sign(gear_ratio[i]);

		command_reference(pos_ref, vel_ref);
	}
	++t_sweep_index;

	if (all_ready) {
		all_index_detected = true;
		is_masterboard_ready = true;
		is_sweep_done = true;

		for (size_t i = 0; i < motor_count; i++) {
			if (is_hard_calibrating) {
				mb.motors[i].set_enable_index_offset_compensation(true);
			} else {
				mb.motors[i].SetPositionOffset(index_offset[i]);
				mb.motors[i].set_enable_index_offset_compensation(true);
			}
		}

		if (is_hard_calibrating) {
			for (size_t i = 0; i < motor_count; ++i) {
				pos_ref[i] = -index_pos[i];
				vel_ref[i] = 0.0;
			}
		} else {
			Row<motor_count> index_pos_row;

			for (size_t i = 0; i < motor_count; ++i) {
				index_pos_row[i] = index_pos[i];
				pos_ref[i] = 0.0;
				vel_ref[i] = 0.0;
			}
			/* write index position to file */
			std::vector<Row<motor_count>> index_pos_vec;
			index_pos_vec.push_back(index_pos_row);
			matrix_rw::Writer<motor_count> write_index_pos;
			write_index_pos(fprefix + index_pos_fname, index_pos_vec);
		}
	}
	command_reference(pos_ref, vel_ref);
}

void
Commander::generate_track_command()
{
	if (t_index < t_size - 1) {
		get_reference(t_index, pos_ref, vel_ref);
	} else {
		get_hold_reference(pos_ref, vel_ref);
	}

	if (using_masterboard_pd) {
		command_reference(pos_ref, vel_ref);
	} else {
		command_current(pos_ref, vel_ref);
	}

	// track_error(pos_ref, vel_ref);

	if (t_index < t_size - 1) {
		++t_index;

		/* sample only during the first trajectory */
		if (!traj_is_sampled) {
			sample_traj();
		}
	} else if (is_looping_traj) {
		traj_is_sampled = true;
		auto thread = std::thread([&] {
			log_traj();
			/* clear the old trajectory */
			// traj.clear();
			ref_traj.clear();
			/* read new trajectory */
			ref_traj.reserve(t_dim_expected);
			ref_traj_reader(ref_traj_fprefix + ref_traj_fname, ref_traj);

			t_size = ref_traj.size(); /* determine t_dim */
			traj.reserve(t_size);
		});
		thread.detach();
		t_index = 0;
	}
}

/*********************/
/* utility functions */
/*********************/

void
Commander::update_stats()
{
	/* records the max amp from motor */
	for (size_t i = 0; i < driver_count; ++i) {
		if (mb.motor_drivers[i].adc[0] > max_amp_stat ||
		    mb.motor_drivers[i].adc[1] > max_amp_stat) {
			max_amp_stat = (mb.motor_drivers[i].adc[0] > mb.motor_drivers[i].adc[1])
			    ? mb.motor_drivers[i].adc[0]
			    : mb.motor_drivers[i].adc[1];
		}
	}
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
Commander::log_traj()
{
	writematrix(fprefix + traj_fname, traj);
}

void
Commander::saturate_reference(double (&pos_ref)[motor_count])
{
	for (size_t i = 0; i < motor_count; ++i) {
		if (pos_ref[i] > max_pos[i]) {
			pos_ref[i] = max_pos[i];
		} else if (pos_ref[i] < min_pos[i]) {
			pos_ref[i] = min_pos[i];
		}
	}
}

/*******************/
/* Print functions */
/*******************/

/* Prints info */
void
Commander::print_info()
{
	printf("State:\n");
	print_state();

	if (commander::print_stats) {
		printf("Stats:\n");
		print_stats();
	}

	if (commander::print_command_timing) {
		printf("Command timing:\n");
		command_timing_stats.print();
	}
	if (commander::print_print_timing) {
		printf("Print timing:\n");
		print_timing_stats.print();
	}

	if (commander::print_offset) {
		printf("Offset:\n");
		print_offset();
	}

	if (commander::print_masterboard) {
		printf("Masterboard:\n");
		print_masterboard();
	}

	if (commander::print_traj) {
		printf("Trajectory:\n");
		print_traj();
	}
}

/* Prints state */
void
Commander::print_state()
{
	if (is_hard_calibrating) {
		printf("Offset Values: {");
		for (size_t i = 0; i < motor_count; i++) {
			if (i == motor_count - 1) {
				printf("%g", index_pos[i]);
			} else {
				printf("%g, ", index_pos[i]);
			}
		}
		printf("} \n");
	}

	if (mb.IsTimeout()) {
		printf("ERROR TIMEOUT\n");
	}

	bool motor_error_header_printed = false;
	for (size_t i = 0; i < motor_count; i++) {
		if (!(mb.motor_drivers[i / 2].is_connected)) {
			if (!motor_error_header_printed) {
				printf("ERROR MOTOR ID: ");
				motor_error_header_printed = true;
			}
			printf("%zu ", i);
		}
	}
	if (motor_error_header_printed) {
		printf("\n");
	}

	bool spi_error_header_printed = false;
	for (size_t i = 0; i < motor_count; i++) {
		if (mb.motor_drivers[i / 2].error_code == 0xf) {
			if (!spi_error_header_printed) {
				printf("ERROR SPI ID: ");
				spi_error_header_printed = true;
			}
			printf("%zu ", i);
		}
	}
	if (spi_error_header_printed) {
		printf("\n");
	}

	printf("| %12s | %12s | %12s | %12s | %12s |\n", "Masterboard", "Robot state",
	       "Index detect", "Hard Calibr.", "Track PD");
	printf("| %12s | %12s | %12s | %12s | %12s | \n",
	       (is_masterboard_connected) ? "Connected" : "Disconnected",
	       state_to_name[state].c_str(), (all_index_detected) ? "All found" : "Not Done",
	       (is_hard_calibrating) ? "True" : "False",
	       (using_masterboard_pd) ? "Onboard" : "Current");
}

/* Prints stats */
void
Commander::print_stats()
{
	printf("| %8s |\n", "Max Amp");
	printf("| %8.2f |\n", max_amp_stat);
}

/* Prints offset info */
void
Commander::print_offset()
{
	bool header_printed = false;

	for (size_t i = 0; i < motor_count; ++i) {
		if (!mb.motor_drivers[i / 2].is_connected) {
			continue;
		}

		if (!header_printed) {
			printf("Motor |  offset   |  idx pos  |   flag    | \n");
			header_printed = true;
		}

		printf("%5.2ld | ", i);
		printf("%9.3g | ", index_offset[i]);
		printf("%9.3g | ", index_pos[i]);
		printf("%9d | ", was_index_detected[i]);
		printf("\n");
	}
}

/* Prints trajectory info */
void
Commander::print_traj()
{
	bool header_printed = false;

	printf("index: %zu ------\n", t_index);

	for (size_t i = 0; i < motor_count; ++i) {
		if (!mb.motor_drivers[i / 2].is_connected) {
			continue;
		}

		if (!header_printed) {
			printf("| %5s | %10s | %10s | %10s | %10s |\n", "Motor", "Ref pos", "pos",
			       "Ref vel", "vel");
			header_printed = true;
		}
		printf("| %5.2zu | ", i);
		printf("%10.3g | ", pos_ref[i]);
		printf("%10.3g | ", pos[i]);
		printf("%10.3g | ", vel_ref[i]);
		printf("%10.3g | ", vel[i]);
		printf("\n");
	}
}

/* Prints masterboard printing functions */
void
Commander::print_masterboard()
{
	mb.PrintIMU();
	mb.PrintADC();
	mb.PrintMotors();
	mb.PrintMotorDrivers();
	mb.PrintStats();
}

} // namespace commander
