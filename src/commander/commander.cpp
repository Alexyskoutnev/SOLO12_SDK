
#include "commander/commander.hpp"
#include "commander/config.hpp"

#include <math.h>
#include <iostream>
#include <thread>
#include <cstdlib>

namespace commander
{
Commander::Commander(const std::string ref_traj_fname, const std::string mb_hostname,
                     const double kp, const double kd)
    : ref_traj_fname(ref_traj_fname), mb(mb_hostname), kp(kp), kd(kd)
{
	initialize();

	for (Size i = 0; i < motor_count; ++i) {
		index_pos[i] = 0.;
		was_index_detected[i] = false;
	}
}

Commander::~Commander()
{
}

void
Commander::initialize()
{
	mb.Init();	
	for (size_t i = 0; i < driver_count; ++i) {
		mb.motor_drivers[i].motor1->SetCurrentReference(0);
		mb.motor_drivers[i].motor2->SetCurrentReference(0);
		mb.motor_drivers[i].motor1->Enable();
		mb.motor_drivers[i].motor2->Enable();
		//* Set the gains for the PD controller running on the cards.
		mb.motor_drivers[i].motor1->set_kp(kp);
		mb.motor_drivers[i].motor2->set_kp(kp);
		mb.motor_drivers[i].motor1->set_kd(kd);
		mb.motor_drivers[i].motor2->set_kd(kd);
		//* Set the maximum current controlled by the card.
		mb.motor_drivers[i].motor1->set_current_sat(max_current);
		mb.motor_drivers[i].motor2->set_current_sat(max_current);
		mb.motor_drivers[i].EnablePositionRolloverError();
		mb.motor_drivers[i].SetTimeout(masterboard_timeout);
		mb.motor_drivers[i].Enable();
	}

	ref_traj.clear();
	traj.clear();

	ref_traj.reserve(t_dim_expected);
	readmatrix(ref_traj_fprefix + ref_traj_fname, ref_traj);
	t_size = ref_traj.size(); /** determine t_dim */

	traj.reserve(t_size);
	t_index = 0;

	is_ready = true;

	for (Size i = 0; i < motor_count; ++i) {
		if (!was_index_detected[i]) {
			is_ready = false;
			break;
		}
	}
}

void
Commander::print()
{
	sample();
	mb.PrintIMU();
	mb.PrintADC();
	mb.PrintMotors();
	mb.PrintMotorDrivers();
	mb.PrintStats();
}

void
Commander::sample()
{
	mb.ParseSensorData();
}

void
Commander::command()
{
	mb.SendCommand();
}

void
Commander::log()
{
	writematrix(fprefix + traj_fname, traj);
}

void
Commander::send_init()
{
	if (!mb.IsTimeout() && !mb.IsAckMsgReceived()) {
		mb.SendInit();
	}
}

void
Commander::standby()
{
	/** do nothing */
}

void
Commander::hold()
{
	sample();

	for (Size i = 0; i < motor_count; ++i) {
		if (!mb.motors[i].IsEnabled()) {
			continue;
		}
		// mb.motors[i].SetCurrentReference(0.);
		mb.motors[i].SetPositionReference(gear_ratio[motor2ref_idx[i]] *
		                                  ref_hold_position[motor2ref_idx[i]]);
		// mb.motors[i].SetVelocityReference(0.);
	}
	command();
}

bool
Commander::check_ready()
{
	return is_ready;
}

void
Commander::enable_calibration()
{
	is_calibrating = true;
}

void
Commander::track()
{
	if (t_index >= t_size) {
		hold();
		return;
	}
	Row<traj_dim> state;
	sample();

	for (Size i = 0; i < motor_count; ++i) {
		if (!mb.motors[i].IsEnabled()) {
			continue;
		}
		const double pos = mb.motors[i].GetPosition();
		const double vel = mb.motors[i].GetVelocity();
		state[ref2motor_idx[i]] = pos;
		state[ref2motor_idx[velocity_shift + i]] = vel;

		const double ref_pos =
		    gear_ratio[motor2ref_idx[i]] * ref_traj[t_index][motor2ref_idx[i]];
		const double ref_vel = gear_ratio[motor2ref_idx[i]] *
		    ref_traj[t_index][velocity_shift + motor2ref_idx[i]];

		// mb.motors[i].SetCurrentReference(0.);
		mb.motors[i].SetPositionReference(ref_pos);
		mb.motors[i].SetVelocityReference(ref_vel);
	}
	command();
	traj.push_back(state);
	++t_index;
}

void
Commander::sweep()
{
	

	constexpr Size t_sweep_size = static_cast<Size>((1. / idx_sweep_freq) * track_freq);
	
	bool all_ready = true;

	sample();

	// print_12arr("dectection array", was_index_detected);
	// print_12arr("motor angle ", motor_ang);
	// for (Size i = 0; i < 6; ++i) {
	// 	mb.motor_drivers[i].Print();
	// }

	for (Size i = 0; i < motor_count; i++) {
		motor_ang[i] = mb.motors[i].GetPosition();
		if (!mb.motors[i].IsEnabled()) {
			continue;
		}

		if (was_index_detected[i]) {
			double des_pos = 0.;
			double des_vel = 0.;
			// mb.motors[i].SetCurrentReference(0.);
			mb.motors[i].SetPositionReference(des_pos);
			mb.motors[i].SetVelocityReference(des_vel);
			continue;
		}
		all_ready = false;
		if (mb.motors[i].HasIndexBeenDetected()) {
			was_index_detected[i] = true;
			index_pos[i] = mb.motors[i].GetPosition();
			/** enable offset */
			// mb.motors[i].set_enable_index_offset_compensation(true);
			// mb.motors[i].SetPositionOffset(index_offset[i]); //Change to index_pos
			// mb.motors[i].SetPositionOffset(index_pos[i]);
			continue;
		}
		const double t =
		    static_cast<double>(t_sweep_index) / static_cast<double>(t_sweep_size);
		double ref_pos;
		double ref_vel;
		// ref_pos = idx_sweep_ampl - idx_sweep_ampl * cos(2. * M_PI * t);
		ref_pos = idx_sweep_ampl * sin(2. * M_PI * t);
		ref_vel = 2. * M_PI * idx_sweep_ampl * cos(2. * M_PI * t);
		mb.motors[i].SetCurrentReference(0.);
		mb.motors[i].SetPositionReference(gear_ratio[motor2ref_idx[i]] * ref_pos);
		mb.motors[i].SetVelocityReference(gear_ratio[motor2ref_idx[i]] * ref_vel);
	}
	command();
	++t_sweep_index;
	if (all_ready == true){
		// print_12arr("offset ", index_pos);
		is_ready = true;
	}
}

void 
Commander::print_12arr(std::string msg, double arr[]){
	//msg += " | ";
	printf("%16s |", msg.c_str());

	for (int i = 0; i < 12; i++){
		printf("[%2d] %10.3g | ", i, arr[i]);
		//msg += "[ " + std::to_string(i) + " ] " + std::to_string(arr[i]) + " | ";
	}
	printf("\n");
	
	//.std::cout << msg << std::endl;
}

void 
Commander::print_12arr(std::string msg, bool arr[]){
	msg += " | ";
	for (int i = 0; i < 12; i++){
		msg += "[ " + std::to_string(i) + " ] " + std::to_string(arr[i]) + " | ";
	}
	std::cout << msg << std::endl;
}

} // namespace commander
