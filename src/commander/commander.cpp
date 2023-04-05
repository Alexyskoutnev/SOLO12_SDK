
#include "commander/commander.hpp"

namespace commander
{

Commander::Commander(const std::string ref_traj_fname, const char mb_hostname[], const double kp,
                     const double kd)
    : ref_traj_fname(ref_traj_fname), kp(kp), kd(kd)
{

#ifndef DRY_BUILD
	/** not yet supported by rt_timer */
	nice(-20);
	mb(mb_hostname);

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
		mb.motor_drivers[i].motor1->set_current_sat(current_sat);
		mb.motor_drivers[i].motor2->set_current_sat(current_sat);
		mb.motor_drivers[i].EnablePositionRolloverError();
		mb.motor_drivers[i].SetTimeout(timeout);
		mb.motor_drivers[i].Enable();
	}
#endif
	initialize();
}

Commander::~Commander(){};

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
Commander::sample()
{
#ifndef DRY_BUILD
	mb.ParseSensorData();
#endif
}

void
Commander::log()
{
	sample();
	writematrix(fprefix + traj_fname, traj);
}

void
Commander::send_init()
{
#ifndef DRY_BUILD
	if (!mb.IsTimeout() && !mb.IsAckMsgReceived()) {
		mb.SendInit();
	}
#endif
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
	is_ready = true;
	for (Size i = 0; i < motor_count; ++i) {
#ifndef DRY_BUILD
		if (!(mb.motors[i].IsEnabled() && mb.motors[i].IsReady())) {
			is_ready = false;
		}
		mb.motors[i].SetCurrentReference(0.);
		mb.motors[i].SetPositionReference(0.);
		mb.motors[i].SetVelocityReference(0.);
#endif
	}
};

void
Commander::track()
{
	sample();

	if (is_ready && t_index >= t_size) {
		hold();
		return;
	}

#ifndef DRY_BUILD
	Row<traj_dim> state;

	for (Size i = 0; i < motor_count; ++i) {
		const double pos = mb.motors[i].GetPosition();
		const double vel = mb.motors[i].GetVelocity();
		state[ref2motor_idx[i]] = pos;
		state[ref2motor_idx[velocity_shift + i]] = vel;

		const double ref_pos = ref_traj[t_index][motor2ref_idx[i]];
		const double ref_vel = ref_traj[t_index][velocity_shift + motor2ref_idx[i]];
		if (mb.motors[i].IsEnabled()) {
			mb.motors[i].SetCurrentReference(0.);
			mb.motors[i].SetPositionReference(ref_pos);
			mb.motors[i].SetVelocityReference(ref_vel);
		}
	}
	traj.push_back(state);
#endif
	++t_index;
};
} // namespace commander
