
#include "commander/commander.hpp"

namespace commander
{
Commander::Commander(const std::string ref_traj_fname, const std::string mb_hostname,
                     const double kp, const double kd)
    : ref_traj_fname(ref_traj_fname), mb(mb_hostname), kp(kp), kd(kd)
{
	initialize();
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
		mb.motor_drivers[i].SetTimeout(5);
		mb.motor_drivers[i].Enable();
	}

	ref_traj.clear();
	traj.clear();

	ref_traj.reserve(t_dim_expected);
	readmatrix(ref_traj_fprefix + ref_traj_fname, ref_traj);
	t_size = ref_traj.size(); /** determine t_dim */

	traj.reserve(t_size);
	t_index = 0;
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
		mb.motors[i].SetCurrentReference(0.);
		mb.motors[i].SetPositionReference(0.);
		mb.motors[i].SetVelocityReference(0.);
	}
	command();
}

void
Commander::track()
{
	if (t_index >= t_size) {
		hold();
		return;
	}

#ifndef DRY_BUILD
	Row<traj_dim> state;
	sample();

	for (Size i = 0; i < motor_count; i++) {
		if (mb.motors[i].IsEnabled()) {
			const double pos = mb.motors[i].GetPosition();
			const double vel = mb.motors[i].GetVelocity();
			state[ref2motor_idx[i]] = pos;
			state[ref2motor_idx[velocity_shift + i]] = vel;

			const double ref_pos =
			    gear_ratio[motor2ref_idx[i]] * ref_traj[t_index][motor2ref_idx[i]];
			const double ref_vel = gear_ratio[motor2ref_idx[i]] *
			    ref_traj[t_index][velocity_shift + motor2ref_idx[i]];

			mb.motors[i].SetCurrentReference(0.);
			mb.motors[i].SetPositionReference(ref_pos);
			mb.motors[i].SetVelocityReference(ref_vel);
		}
	}
	command();
	traj.push_back(state);
#endif
	++t_index;
}
} // namespace commander
