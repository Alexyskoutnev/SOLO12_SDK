
#include "traj_track/interface.hpp"

Interface::Interface(char *name, size_t t_dim, size_t x_dim, double ref_traj[], double kp,
                     double kd, double current_sat, size_t timeout)
    : masterboard(name), t_dim(t_dim), x_dim(x_dim), ref_traj(ref_traj), kp(kp), kd(kd),
      current_sat(current_sat), timeout(timeout)
{
	masterboard.Init();

	for (size_t i = 0; i < driver_count; ++i) {
		masterboard.motor_drivers[i].motor1->SetCurrentReference(0);
		masterboard.motor_drivers[i].motor2->SetCurrentReference(0);
		masterboard.motor_drivers[i].motor1->Enable();
		masterboard.motor_drivers[i].motor2->Enable();
		//* Set the gains for the PD controller running on the cards.
		masterboard.motor_drivers[i].motor1->set_kp(kp);
		masterboard.motor_drivers[i].motor2->set_kp(kp);
		masterboard.motor_drivers[i].motor1->set_kd(kd);
		masterboard.motor_drivers[i].motor2->set_kd(kd);
		//* Set the maximum current controlled by the card.
		masterboard.motor_drivers[i].motor1->set_current_sat(current_sat);
		masterboard.motor_drivers[i].motor2->set_current_sat(current_sat);
		masterboard.motor_drivers[i].EnablePositionRolloverError();
		masterboard.motor_drivers[i].SetTimeout(timeout);
		masterboard.motor_drivers[i].Enable();
	}
	auto last = std::chrono::system_clock::now();
	auto try_wait = 0.1;

	while (!masterboard.IsTimeout() && !masterboard.IsAckMsgReceived()) {
		if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last))
		        .count() > try_wait) {
			last = std::chrono::system_clock::now();
			masterboard.SendInit();
		}
	}

	if (masterboard.IsTimeout()) {
		printf("Timeout while waiting for ack.\n");
	}
}

Interface::~Interface()
{
	masterboard.Stop();
}

void
Interface::update()
{
	if (!masterboard.IsTimeout()) {
		//* read sensors
		masterboard.ParseSensorData();

		if (is_ready) {

			for (size_t i = 0; i < motor_count; ++i) {
				//* select row of reference trajectory
				//*
				if (step_counter < t_dim) {
					masterboard.motors[i].SetCurrentReference(0.);
					masterboard.motors[ref_idx[i]].SetPositionReference(
					    gear_ratio[ref_idx[i]] * ref_traj[step_counter * x_dim + i]);
					masterboard.motors[i].SetVelocityReference(0.);
				} else {
					masterboard.motors[i].SetCurrentReference(0.);
					masterboard.motors[i].SetPositionReference(init_pos[i]);
					masterboard.motors[i].SetVelocityReference(0.);
				}
			}
			masterboard.SendCommand();
			++step_counter;
		} else {
			is_ready = check_ready();
		}
	} else {
		printf("Timeout while update.\n");
	}
}

bool
Interface::check_ready()
{
	bool is_ready = true;

	for (size_t i = 0; i < motor_count; ++i) {
		if (!masterboard.motor_drivers[i / 2].is_connected) {
			continue;
		}

		if (!(masterboard.motors[i].IsEnabled() && masterboard.motors[i].IsReady())) {
			is_ready = false;
			// printf("Still not ready...\n");
		}
		init_pos[i] = masterboard.motors[i].GetPosition();
		masterboard.motors[i].SetCurrentReference(0.);
		masterboard.motors[i].SetPositionReference(init_pos[i]);
		masterboard.motors[i].SetVelocityReference(0.);
	}
	masterboard.SendCommand();

	return is_ready;
}

void
Interface::print()
{
	masterboard.PrintIMU();
	masterboard.PrintADC();
	masterboard.PrintMotors();
	masterboard.PrintMotorDrivers();
	masterboard.PrintStats();
}

size_t
Interface::get_step_count()
{
	return step_counter;
}