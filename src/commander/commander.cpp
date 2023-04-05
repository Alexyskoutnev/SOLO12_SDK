
#include "commander/commander.hpp"

namespace commander
{

Commander::Commander(const std::string ref_traj_fname, const std::string mb_hostname,
                     const double kp, const double kd)
    : ref_traj_fname(ref_traj_fname), kp(kp), kd(kd)
#ifndef DRY_BUILD
      ,
      mb(mb_hostname)
#endif
{
#ifndef DRY_BUILD
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
	auto last = std::chrono::system_clock::now();
	auto try_wait = 0.1;

	while (!mb.IsTimeout() && !mb.IsAckMsgReceived()) {
		if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last))
		        .count() > try_wait) {
			last = std::chrono::system_clock::now();
			mb.SendInit();
		}
	}

	if (mb.IsTimeout()) {
		printf("Timeout while waiting for ack.\n");
	}
#endif
	initialize();
}

Commander::~Commander()
{
}

void
Commander::initialize()
{
	// auto last = std::chrono::system_clock::now();
	// auto try_wait = 0.1;

	// while (!mb.IsTimeout() && !mb.IsAckMsgReceived()) {
	//	if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last))
	//	        .count() > try_wait) {
	//		last = std::chrono::system_clock::now();
	//		mb.SendInit();
	//	}
	// }

	// if (mb.IsTimeout()) {
	//	printf("Timeout while waiting for ack.\n");
	// }

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
#ifndef DRY_BUILD
	mb.PrintIMU();
	mb.PrintADC();
	mb.PrintMotors();
	mb.PrintMotorDrivers();
	mb.PrintStats();
#endif
}

void
Commander::sample()
{
#ifndef DRY_BUILD
	if (!mb.IsTimeout()) {
		// mb.ParseSensorData();
	}
#endif
}

void
Commander::log()
{
	// sample();
	writematrix(fprefix + traj_fname, traj);
}

void
Commander::send_init()
{
#ifndef DRY_BUILD
	if (!mb.IsTimeout() && !mb.IsAckMsgReceived()) {
		// printf("1\n");
		mb.SendInit();
	}
#endif
}

void
Commander::standby()
{
	if (mb.IsTimeout()) {
		printf("Timeout while waiting for ack.\n");
	}
	// mb.SendCommand();
	/** do nothing */
}

void
Commander::hold()
{
	// double init_pos[motor_count];
	// is_ready = true;

	// if (!mb.IsTimeout()) {
	mb.ParseSensorData();
	for (int i = 0; i < motor_count; i++) {
		// if (!mb.motor_drivers[i / 2].is_connected)
		//	continue; // ignoring the motors of a disconnected
		//  slave

		// if (!(mb.motors[i].IsEnabled() && mb.motors[i].IsReady())) {
		//	is_ready = false;
		// }
		//  init_pos[i] = mb.motors[i].GetPosition(); // initial position

		// Use the current state as target for the PD controller.
		mb.motors[i].SetCurrentReference(0.);
		mb.motors[i].SetPositionReference(0.);
		mb.motors[i].SetVelocityReference(0.);
	}
	mb.SendCommand();
	//}
}

void
Commander::track()
{
	// sample();

	// if (t_index >= t_size) {
	//	hold();
	//	return;
	// }

#ifndef DRY_BUILD

	// if (!mb.IsTimeout()) {
	mb.ParseSensorData();
	Row<traj_dim> state;

	// closed loop, position
	for (int i = 0; i < motor_count * 2; i++) {
		if (i % 2 == 0) {
			if (!mb.motor_drivers[i / 2].is_connected)
				continue; // ignoring the motors of a
				          // disconnected slave

			// making sure that the transaction with the
			// corresponding µdriver board succeeded
			if (mb.motor_drivers[i / 2].error_code == 0xf) {
				// printf("Transaction with SPI%d failed\n",
				// i / 2);
				continue; // user should decide what to do
				          // in that case, here we ignore
				          // that motor
			}
		}

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
	mb.SendCommand();
	traj.push_back(state);
	//}
#endif
	++t_index;
}
} // namespace commander
